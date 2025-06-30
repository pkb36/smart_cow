#include "SignalingClient.h"
#include "../utils/Logger.h"
#include <libsoup/soup.h>
#include <json-glib/json-glib.h>
#include <sstream>

SignalingClient::SignalingClient(const std::string& serverUrl, const std::string& cameraId)
    : serverUrl_(serverUrl)
    , cameraId_(cameraId)
    , session_(nullptr)
    , connection_(nullptr)
    , state_(ConnectionState::DISCONNECTED)
    , autoReconnect_(true)
    , reconnectInterval_(5) {
    
    LOG_INFO("SignalingClient created for camera %s, server: %s",
             cameraId.c_str(), serverUrl.c_str());
}

SignalingClient::~SignalingClient() {
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
            SOUP_SESSION_HTTPS_ALIASES, soup_session_get_features(SOUP_SESSION(session_)),
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

bool SignalingClient::isConnected() const {
    return state_ == ConnectionState::CONNECTED;
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
    
    LOG_DEBUG("Sent message: type=%s, data_len=%zu", type.c_str(), data.length());
    
    g_free(message);
    g_object_unref(generator);
    g_object_unref(builder);
    
    return true;
}

bool SignalingClient::sendToPeer(const std::string& peerId, const std::string& type, 
                                const std::string& data) {
    if (state_ != ConnectionState::CONNECTED || !connection_) {
        LOG_ERROR("Not connected to signaling server");
        return false;
    }
    
    // JSON 메시지 생성
    JsonBuilder* builder = json_builder_new();
    json_builder_begin_object(builder);
    json_builder_set_member_name(builder, "type");
    json_builder_add_string_value(builder, type.c_str());
    json_builder_set_member_name(builder, "peer_id");
    json_builder_add_string_value(builder, peerId.c_str());
    json_builder_set_member_name(builder, "data");
    json_builder_add_string_value(builder, data.c_str());
    json_builder_end_object(builder);
    
    JsonGenerator* generator = json_generator_new();
    JsonNode* root = json_builder_get_root(builder);
    json_generator_set_root(generator, root);
    
    gchar* message = json_generator_to_data(generator, nullptr);
    
    // 메시지 전송
    soup_websocket_connection_send_text(connection_, message);
    
    LOG_DEBUG("Sent message to peer %s: type=%s", peerId.c_str(), type.c_str());
    
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
    // 카메라 등록 메시지 생성
    JsonBuilder* builder = json_builder_new();
    json_builder_begin_object(builder);
    json_builder_set_member_name(builder, "msg_type");
    json_builder_add_string_value(builder, "client_msg");
    json_builder_set_member_name(builder, "client_type");
    json_builder_add_string_value(builder, "camera");
    json_builder_set_member_name(builder, "peer_id");
    json_builder_add_string_value(builder, cameraId_.c_str());
    json_builder_end_object(builder);
    
    JsonGenerator* generator = json_generator_new();
    JsonNode* root = json_builder_get_root(builder);
    json_generator_set_root(generator, root);
    
    gchar* message = json_generator_to_data(generator, nullptr);
    bool result = sendMessage("register", message);
    
    g_free(message);
    g_object_unref(generator);
    g_object_unref(builder);
    
    if (result) {
        LOG_INFO("Camera registration sent: %s", cameraId_.c_str());
    }
    
    return result;
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
    
    self->state_ = ConnectionState::DISCONNECTED;
    if (self->stateCallback_) {
        self->stateCallback_(ConnectionState::DISCONNECTED);
    }
    
    // 재연결 시도
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
    const gchar* msgType = json_object_get_string_member(obj, "msg_type");
    if (!msgType) {
        msgType = json_object_get_string_member(obj, "type");
    }
    
    if (!msgType) {
        LOG_ERROR("Message has no type field");
        g_object_unref(parser);
        return;
    }
    
    // Message 구조체 생성
    Message msg;
    msg.type = msgType;
    
    // peer_id 추출
    if (json_object_has_member(obj, "peer_id")) {
        msg.peerId = json_object_get_string_member(obj, "peer_id");
    }
    
    // 데이터 추출
    if (json_object_has_member(obj, "data")) {
        JsonNode* dataNode = json_object_get_member(obj, "data");
        
        if (JSON_NODE_TYPE(dataNode) == JSON_NODE_OBJECT) {
            // 객체인 경우 다시 문자열로 변환
            JsonGenerator* gen = json_generator_new();
            json_generator_set_root(gen, dataNode);
            gchar* dataStr = json_generator_to_data(gen, nullptr);
            msg.data = dataStr;
            g_free(dataStr);
            g_object_unref(gen);
        } else {
            msg.data = json_object_get_string_member(obj, "data");
        }
    } else {
        // 전체 메시지를 data로
        msg.data = message;
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