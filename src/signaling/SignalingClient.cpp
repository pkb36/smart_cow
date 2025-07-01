#include "SignalingClient.h"
#include "../utils/Logger.h"
#include <libsoup/soup.h>
#include <json-glib/json-glib.h>
#include <sstream>
#include <fstream>
#include <sys/statvfs.h>

SignalingClient::SignalingClient(const std::string& serverUrl, const std::string& cameraId)
    : serverUrl_(serverUrl)
    , cameraId_(cameraId)
    , session_(nullptr)
    , connection_(nullptr)
    , state_(ConnectionState::DISCONNECTED)
    , autoReconnect_(true)
    , reconnectInterval_(5)
    , statusTimerId_(0)
    , statusInterval_(30) {

    serverUrl_ = serverUrl + "/signaling/" + cameraId + "/?token=test&peerType=camera";
    
    LOG_INFO("SignalingClient created for camera %s, server: %s",
             cameraId.c_str(), serverUrl.c_str());
}

SignalingClient::~SignalingClient() {
    stopStatusReporting();
    disconnect();
    
    if (session_) {
        g_object_unref(session_);
        session_ = nullptr;
    }
}

bool SignalingClient::connect() {
    std::lock_guard<std::mutex> lock(connectionMutex_);
    
    if (state_ == ConnectionState::CONNECTED) {
        LOG_WARN("Already connected to signaling server");
        return true;
    }
    
    state_ = ConnectionState::CONNECTING;
    
    // Soup 세션 생성
    if (!session_) {
        session_ = soup_session_new_with_options(
            SOUP_SESSION_SSL_STRICT, FALSE,
            SOUP_SESSION_SSL_USE_SYSTEM_CA_FILE, TRUE,
            nullptr);
    }
    
    // WebSocket 연결 메시지 생성
    SoupMessage* msg = soup_message_new(SOUP_METHOD_GET, serverUrl_.c_str());
    if (!msg) {
        LOG_ERROR("Failed to create WebSocket message for %s", serverUrl_.c_str());
        state_ = ConnectionState::DISCONNECTED;
        return false;
    }
    
    // WebSocket 연결
    soup_session_websocket_connect_async(
        session_, msg, nullptr, nullptr, nullptr,
        (GAsyncReadyCallback)SignalingClient::onConnected, this);
    
    g_object_unref(msg);
    
    LOG_INFO("Connecting to signaling server: %s", serverUrl_.c_str());
    return true;
}

void SignalingClient::disconnect() {
    std::lock_guard<std::mutex> lock(connectionMutex_);
    
    autoReconnect_ = false;
    
    if (reconnectThread_.joinable()) {
        reconnectThread_.join();
    }
    
    if (connection_) {
        soup_websocket_connection_close(connection_, SOUP_WEBSOCKET_CLOSE_NORMAL, nullptr);
        g_object_unref(connection_);
        connection_ = nullptr;
    }
    
    state_ = ConnectionState::DISCONNECTED;
    LOG_INFO("Disconnected from signaling server");
}

bool SignalingClient::isWebSocketOpen() const {
    if (!connection_) return false;
    
    SoupWebsocketState state = soup_websocket_connection_get_state(connection_);
    return state == SOUP_WEBSOCKET_STATE_OPEN;
}

bool SignalingClient::isConnected() const {
    return state_ == ConnectionState::CONNECTED && isWebSocketOpen();
}

bool SignalingClient::sendMessage(const std::string& type, const std::string& data) {
    if (state_ != ConnectionState::CONNECTED || !connection_) {
        LOG_ERROR("Not connected to signaling server");
        return false;
    }
    
    // JSON 메시지 생성
    JsonBuilder* builder = json_builder_new();
    json_builder_begin_object(builder);
    json_builder_set_member_name(builder, "type");
    json_builder_add_string_value(builder, type.c_str());
    json_builder_set_member_name(builder, "data");
    json_builder_add_string_value(builder, data.c_str());
    json_builder_end_object(builder);
    
    JsonGenerator* generator = json_generator_new();
    JsonNode* root = json_builder_get_root(builder);
    json_generator_set_root(generator, root);
    
    gchar* message = json_generator_to_data(generator, nullptr);
    
    // 메시지 전송
    soup_websocket_connection_send_text(connection_, message);
    
    LOG_INFO("Sent message: type=%s, data_len=%zu json=%s", type.c_str(), data.length(), message);
    
    g_free(message);
    g_object_unref(generator);
    g_object_unref(builder);
    
    return true;
}

bool SignalingClient::sendToPeer(const std::string& peerId, const std::string& type,
                                const std::string& data) {
    // WebSocket 연결 상태 확인
    if (!connection_) {
        LOG_ERROR("WebSocket connection is null");
        return false;
    }
    
    SoupWebsocketState wsState = soup_websocket_connection_get_state(connection_);
    if (wsState != SOUP_WEBSOCKET_STATE_OPEN) {
        LOG_ERROR("WebSocket is not open (state=%d)", wsState);
        return false;
    }
    
    // JSON 메시지 생성
    JsonBuilder* builder = json_builder_new();
    json_builder_begin_object(builder);
    
    // action 필드
    json_builder_set_member_name(builder, "action");
    json_builder_add_string_value(builder, type.c_str());
    
    // peerType 필드 (camera 고정)
    json_builder_set_member_name(builder, "peerType");
    json_builder_add_string_value(builder, "camera");
    
    // message 필드
    json_builder_set_member_name(builder, "message");
    json_builder_begin_object(builder);
    
    // peer_id 추가
    json_builder_set_member_name(builder, "peer_id");
    json_builder_add_string_value(builder, peerId.c_str());
    
    // type에 따라 적절한 필드에 data 추가
    if (g_strcmp0(type.c_str(), "offer") == 0 || g_strcmp0(type.c_str(), "answer") == 0) {
        // SDP 메시지인 경우 - data는 SDP 문자열
        json_builder_set_member_name(builder, "sdp");
        json_builder_begin_object(builder);
        json_builder_set_member_name(builder, "type");
        json_builder_add_string_value(builder, type.c_str());
        json_builder_set_member_name(builder, "sdp");
        json_builder_add_string_value(builder, data.c_str());
        json_builder_end_object(builder);
        
    } else if (g_strcmp0(type.c_str(), "ice_candidate") == 0) {
        // ICE candidate인 경우 - data는 이미 JSON 문자열
        JsonParser* parser = json_parser_new();
        GError* error = nullptr;
        
        if (json_parser_load_from_data(parser, data.c_str(), -1, &error)) {
            JsonNode* dataNode = json_parser_get_root(parser);
            json_builder_set_member_name(builder, "ice");
            json_builder_add_value(builder, json_node_copy(dataNode));
        } else {
            // ICE candidate 파싱 실패 - 수동으로 생성
            LOG_WARN("Failed to parse ICE candidate JSON: %s", 
                     error ? error->message : "unknown error");
            if (error) g_error_free(error);
            
            // 간단한 문자열로 처리
            json_builder_set_member_name(builder, "ice");
            json_builder_begin_object(builder);
            json_builder_set_member_name(builder, "candidate");
            json_builder_add_string_value(builder, data.c_str());
            json_builder_set_member_name(builder, "sdpMLineIndex");
            json_builder_add_int_value(builder, 0);
            json_builder_end_object(builder);
        }
        g_object_unref(parser);
        
    } else {
        // 기타 메시지 - 일반 문자열로 처리
        json_builder_set_member_name(builder, "data");
        json_builder_add_string_value(builder, data.c_str());
    }
    
    json_builder_end_object(builder);  // message 객체 종료
    json_builder_end_object(builder);  // 최상위 객체 종료
    
    // JSON 생성
    JsonGenerator* generator = json_generator_new();
    JsonNode* root = json_builder_get_root(builder);
    json_generator_set_root(generator, root);
    
    gchar* message = json_generator_to_data(generator, nullptr);
    
    // 전송
    soup_websocket_connection_send_text(connection_, message);
    LOG_DEBUG("Sent to peer %s (type=%s): %s", peerId.c_str(), type.c_str(), message);
    
    g_free(message);
    g_object_unref(generator);
    g_object_unref(builder);
    
    return true;
}

void SignalingClient::setMessageCallback(MessageCallback callback) {
    messageCallback_ = callback;
}

void SignalingClient::setStateCallback(StateCallback callback) {
    stateCallback_ = callback;
}

bool SignalingClient::registerCamera() {
    // JSON 메시지 생성
    JsonBuilder* builder = json_builder_new();
    json_builder_begin_object(builder);
    json_builder_set_member_name(builder, "action");
    json_builder_add_string_value(builder, "register");
    json_builder_set_member_name(builder, "peerType");
    json_builder_add_string_value(builder, "camera");
    json_builder_set_member_name(builder, "message");
    json_builder_begin_object(builder);
        json_builder_set_member_name(builder, "name");
        json_builder_add_string_value(builder, "udpsink-webrtc");
        json_builder_set_member_name(builder, "fw_version");
        json_builder_add_string_value(builder, "1.0.0");
        json_builder_set_member_name(builder, "ai_version");
        json_builder_add_string_value(builder, "0.1.0");
    json_builder_end_object(builder);
    json_builder_end_object(builder);
    
    JsonGenerator* generator = json_generator_new();
    JsonNode* root = json_builder_get_root(builder);
    json_generator_set_root(generator, root);
    
    gchar* message = json_generator_to_data(generator, nullptr);
    
    // WebSocket으로 직접 전송
    soup_websocket_connection_send_text(connection_, message);
    
    LOG_INFO("Camera registration sent: %s", message);
    
    g_free(message);
    g_object_unref(generator);
    g_object_unref(builder);
    
    return true;
}

void SignalingClient::sendSdpOffer(const std::string& peerId, const std::string& sdp)
{
    JsonBuilder* builder = json_builder_new();
    json_builder_begin_object(builder);
    json_builder_set_member_name(builder, "action");
    json_builder_add_string_value(builder, "offer");
    json_builder_set_member_name(builder, "peerType");
    json_builder_add_string_value(builder, "camera");
    json_builder_set_member_name(builder, "message");
    json_builder_begin_object(builder);
        json_builder_set_member_name(builder, "peer_id");
        json_builder_add_string_value(builder, peerId.c_str());
        json_builder_set_member_name(builder, "sdp");
        json_builder_begin_object(builder);
            json_builder_set_member_name(builder, "type");
            json_builder_add_string_value(builder, "offer");
            json_builder_set_member_name(builder, "sdp");
            json_builder_add_string_value(builder, sdp.c_str());
        json_builder_end_object(builder);
    json_builder_end_object(builder);
    json_builder_end_object(builder);

    JsonGenerator* generator = json_generator_new();
    JsonNode* root = json_builder_get_root(builder);
    json_generator_set_root(generator, root);
    
    gchar* message = json_generator_to_data(generator, nullptr);
    
    // WebSocket으로 직접 전송
    soup_websocket_connection_send_text(connection_, message);
    
    LOG_INFO("Camera SdpOffer sent: %s", message);
    
    g_free(message);
    g_object_unref(generator);
    g_object_unref(builder);
}

void SignalingClient::sendIceCandidate(const std::string& peerId, int mlineindex, const std::string& candidate)
{
    JsonBuilder* builder = json_builder_new();
    json_builder_begin_object(builder);
    json_builder_set_member_name(builder, "action");
    json_builder_add_string_value(builder, "candidate");
    json_builder_set_member_name(builder, "peerType");
    json_builder_add_string_value(builder, "camera");
    json_builder_set_member_name(builder, "message");
    json_builder_begin_object(builder);
        json_builder_set_member_name(builder, "peer_id");
        json_builder_add_string_value(builder, peerId.c_str());
        json_builder_set_member_name(builder, "ice");
        json_builder_begin_object(builder);
            json_builder_set_member_name(builder, "candidate");
            json_builder_add_string_value(builder, candidate.c_str());
            json_builder_set_member_name(builder, "sdpMLineIndex");
            json_builder_add_int_value(builder, mlineindex);
            json_builder_set_member_name(builder, "sdpMid");
            gchar* sdpMid = g_strdup_printf("video%d", mlineindex);
            json_builder_add_string_value(builder, sdpMid);
            g_free(sdpMid);
        json_builder_end_object(builder);
    json_builder_end_object(builder);
    json_builder_end_object(builder);

    JsonGenerator* generator = json_generator_new();
    JsonNode* root = json_builder_get_root(builder);
    json_generator_set_root(generator, root);
    
    gchar* message = json_generator_to_data(generator, nullptr);
    
    // WebSocket으로 직접 전송
    soup_websocket_connection_send_text(connection_, message);
    
    LOG_INFO("Camera IceCandidate sent: %s", message);
    
    g_free(message);
    g_object_unref(generator);
    g_object_unref(builder);
}

void SignalingClient::sendCameraStatus()
{
    if(!connection_) {
        LOG_ERROR("Not connected to signaling server, cannot send camera status");
        return;
    }
    JsonBuilder* builder = json_builder_new();
    json_builder_begin_object(builder);
    json_builder_set_member_name(builder, "action");
    json_builder_add_string_value(builder, "camstatus");
    json_builder_set_member_name(builder, "peerType");
    json_builder_add_string_value(builder, "camera");
    json_builder_set_member_name(builder, "message");
    json_builder_begin_object(builder);
        json_builder_set_member_name(builder, "rec_status");
        json_builder_add_string_value(builder, "On");
        json_builder_set_member_name(builder, "cpu_temperature");
        json_builder_add_double_value(builder, 65.5);
        json_builder_set_member_name(builder, "gpu_temperature");
        json_builder_add_double_value(builder, 72.3);
        json_builder_set_member_name(builder, "rec_usage");
        json_builder_add_int_value(builder, 80);
    json_builder_end_object(builder);
    json_builder_end_object(builder);

    JsonGenerator* generator = json_generator_new();
    JsonNode* root = json_builder_get_root(builder);
    json_generator_set_root(generator, root);
    
    gchar* message = json_generator_to_data(generator, nullptr);
    
    // WebSocket으로 직접 전송
    soup_websocket_connection_send_text(connection_, message);
    
    LOG_INFO("Camera Status sent: %s", message);
    
    g_free(message);
    g_object_unref(generator);
    g_object_unref(builder);
}

void SignalingClient::enableAutoReconnect(bool enable) {
    autoReconnect_ = enable;
    LOG_INFO("Auto-reconnect %s", enable ? "enabled" : "disabled");
}

void SignalingClient::setReconnectInterval(int seconds) {
    reconnectInterval_ = seconds;
}

void SignalingClient::onConnected(SoupSession* session, GAsyncResult* result, 
                                 gpointer userData) {
    SignalingClient* self = static_cast<SignalingClient*>(userData);
    GError* error = nullptr;
    
    self->connection_ = soup_session_websocket_connect_finish(session, result, &error);
    
    if (error) {
        LOG_ERROR("WebSocket connection failed: %s", error->message);
        g_error_free(error);
        
        self->state_ = ConnectionState::DISCONNECTED;
        if (self->stateCallback_) {
            self->stateCallback_(ConnectionState::DISCONNECTED);
        }
        
        // 재연결 시도
        if (self->autoReconnect_) {
            self->reconnect();
        }
        return;
    }
    
    LOG_INFO("Connected to signaling server");
    
    self->state_ = ConnectionState::CONNECTED;
    if (self->stateCallback_) {
        self->stateCallback_(ConnectionState::CONNECTED);
    }
    
    // 시그널 연결
    g_signal_connect(self->connection_, "message",
                     G_CALLBACK(SignalingClient::onMessage), self);
    g_signal_connect(self->connection_, "closed",
                     G_CALLBACK(SignalingClient::onClosed), self);
    g_signal_connect(self->connection_, "error",
                     G_CALLBACK(SignalingClient::onError), self);
    
    // 카메라 등록
    self->registerCamera();
}

void SignalingClient::onMessage(SoupWebsocketConnection* conn, gint type,
                               GBytes* message, gpointer userData) {
    SignalingClient* self = static_cast<SignalingClient*>(userData);
    
    if (type != SOUP_WEBSOCKET_DATA_TEXT) {
        return;
    }
    
    gsize size;
    const gchar* data = static_cast<const gchar*>(g_bytes_get_data(message, &size));
    std::string messageStr(data, size);
    
    self->handleMessage(messageStr);
}

void SignalingClient::onClosed(SoupWebsocketConnection* conn, gpointer userData) {
    SignalingClient* self = static_cast<SignalingClient*>(userData);
    
    LOG_INFO("WebSocket connection closed");
    
    std::lock_guard<std::mutex> lock(self->connectionMutex_);
    self->state_ = ConnectionState::DISCONNECTED;
    
    if (self->stateCallback_) {
        self->stateCallback_(ConnectionState::DISCONNECTED);
    }
    
    // 자동 재연결
    if (self->autoReconnect_) {
        self->reconnect();
    }
}

void SignalingClient::onError(SoupWebsocketConnection* conn, GError* error,
                             gpointer userData) {
    SignalingClient* self = static_cast<SignalingClient*>(userData);
    
    LOG_ERROR("WebSocket error: %s", error->message);
}

void SignalingClient::handleMessage(const std::string& message) {
    // JSON 파싱
    JsonParser* parser = json_parser_new();
    GError* error = nullptr;
    
    if (!json_parser_load_from_data(parser, message.c_str(), -1, &error)) {
        LOG_ERROR("Failed to parse message: %s", error->message);
        g_error_free(error);
        g_object_unref(parser);
        return;
    }
    
    JsonNode* root = json_parser_get_root(parser);
    JsonObject* obj = json_node_get_object(root);
    
    // 메시지 타입 추출
    const gchar* action = json_object_get_string_member(obj, "action");
    if (!action) {
        LOG_ERROR("Message has no action field");
        g_object_unref(parser);
        return;
    }
    
    // Message 구조체 생성
    Message msg;
    msg.type = action;
    
    // peer_id 추출
    JsonObject* messageObj = nullptr;
    if (json_object_has_member(obj, "message")) {
        JsonNode* messageNode = json_object_get_member(obj, "message");
        if (JSON_NODE_TYPE(messageNode) == JSON_NODE_OBJECT) {
            messageObj = json_node_get_object(messageNode);
        }
    }

    // peer_id는 message 객체 내부에서 추출
    if (messageObj && json_object_has_member(messageObj, "peer_id")) {
        msg.peerId = json_object_get_string_member(messageObj, "peer_id");
    }
    
    // 데이터 추출
    if (messageObj) {
        if (action && strcmp(action, "answer") == 0) {
            // SDP answer 처리
            if (json_object_has_member(messageObj, "sdp")) {
                JsonObject* sdpObj = json_object_get_object_member(messageObj, "sdp");
                if (sdpObj && json_object_has_member(sdpObj, "sdp")) {
                    msg.data = json_object_get_string_member(sdpObj, "sdp");
                }
            }
        } else if (action && strcmp(action, "candidate") == 0) {
            // ICE candidate 처리
            if (json_object_has_member(messageObj, "ice")) {
                JsonNode* iceNode = json_object_get_member(messageObj, "ice");
                JsonGenerator* gen = json_generator_new();
                json_generator_set_root(gen, iceNode);
                gchar* iceStr = json_generator_to_data(gen, nullptr);
                msg.data = iceStr;
                g_free(iceStr);
                g_object_unref(gen);
            }
        } else {
            // 기타 message 객체 전체를 data로
            JsonGenerator* gen = json_generator_new();
            json_generator_set_root(gen, json_object_get_member(obj, "message"));
            gchar* dataStr = json_generator_to_data(gen, nullptr);
            msg.data = dataStr;
            g_free(dataStr);
            g_object_unref(gen);
        }
    }
    
    LOG_DEBUG("Received message: type=%s, peer=%s", 
              msg.type.c_str(), msg.peerId.c_str());
    
    // 콜백 호출
    if (messageCallback_) {
        messageCallback_(msg);
    }
    
    g_object_unref(parser);
}

void SignalingClient::reconnect() {
    if (state_ == ConnectionState::RECONNECTING) {
        return;
    }
    
    state_ = ConnectionState::RECONNECTING;
    
    // 재연결 스레드 시작
    if (reconnectThread_.joinable()) {
        reconnectThread_.join();
    }
    
    reconnectThread_ = std::thread(&SignalingClient::reconnectThread, this);
}

void SignalingClient::reconnectThread() {
    LOG_INFO("Starting reconnection attempts (interval: %d seconds)", reconnectInterval_);
    
    while (autoReconnect_ && state_ != ConnectionState::CONNECTED) {
        std::this_thread::sleep_for(std::chrono::seconds(reconnectInterval_));
        
        if (!autoReconnect_) {
            break;
        }
        
        LOG_INFO("Attempting to reconnect...");
        connect();
        
        // 연결 시도 후 잠시 대기
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

void SignalingClient::startStatusReporting(int intervalSeconds) {
    std::lock_guard<std::mutex> lock(statusMutex_);
    
    // 기존 타이머가 있으면 제거
    if (statusTimerId_ > 0) {
        g_source_remove(statusTimerId_);
        statusTimerId_ = 0;
    }
    
    statusInterval_ = intervalSeconds;
    
    // 즉시 한 번 전송
    sendCameraStatus();
    
    // 주기적 타이머 설정
    statusTimerId_ = g_timeout_add_seconds(statusInterval_, 
                                          statusTimerCallback, 
                                          this);
    
    LOG_INFO("Started camera status reporting every %d seconds", intervalSeconds);
}

void SignalingClient::stopStatusReporting() {
    std::lock_guard<std::mutex> lock(statusMutex_);
    
    if (statusTimerId_ > 0) {
        g_source_remove(statusTimerId_);
        statusTimerId_ = 0;
        LOG_INFO("Stopped camera status reporting");
    }
}

void SignalingClient::updateCameraStatus(const std::string& recStatus, 
                                       double cpuTemp, 
                                       double gpuTemp, 
                                       int recUsage) {
    std::lock_guard<std::mutex> lock(statusMutex_);
    
    cameraStatus_.recStatus = recStatus;
    cameraStatus_.cpuTemperature = cpuTemp;
    cameraStatus_.gpuTemperature = gpuTemp;
    cameraStatus_.recUsage = recUsage;
}

gboolean SignalingClient::statusTimerCallback(gpointer userData) {
    SignalingClient* self = static_cast<SignalingClient*>(userData);
    
    if (self && self->isConnected()) {
        self->sendCameraStatus();
        return G_SOURCE_CONTINUE;  // 계속 반복
    }
    
    return G_SOURCE_REMOVE;  // 타이머 제거
}

double SignalingClient::getCpuTemperature() {
    // Jetson Xavier의 경우
    std::ifstream file("/sys/devices/virtual/thermal/thermal_zone0/temp");
    if (file.is_open()) {
        int temp;
        file >> temp;
        file.close();
        return temp / 1000.0;  // millidegree to degree
    }
    return 0.0;
}

double SignalingClient::getGpuTemperature() {
    // Jetson Xavier의 경우
    std::ifstream file("/sys/devices/virtual/thermal/thermal_zone1/temp");
    if (file.is_open()) {
        int temp;
        file >> temp;
        file.close();
        return temp / 1000.0;
    }
    return 0.0;
}

int SignalingClient::getDiskUsage() {
    struct statvfs stat;
    if (statvfs("/", &stat) == 0) {
        unsigned long total = stat.f_blocks * stat.f_frsize;
        unsigned long available = stat.f_bavail * stat.f_frsize;
        unsigned long used = total - available;
        return (int)((used * 100) / total);
    }
    return 0;
}