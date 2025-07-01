#include "PeerManager.h"
#include "WebRTCSenderProcess.h"
#include "../pipeline/Pipeline.h"
#include "../pipeline/CameraSource.h"
#include "../utils/Logger.h"
#include <json-glib/json-glib.h>

PeerManager::PeerManager(Pipeline* pipeline, int maxPeers)
    : pipeline_(pipeline)
    , signalingClient_(nullptr)
    , maxPeers_(maxPeers)
    , baseStreamPort_(0)
    , commSocketBasePort_(0) {
    
    LOG_INFO("PeerManager created (max peers: %d)", maxPeers);
}

PeerManager::~PeerManager() {
    stopAllProcesses();
}

bool PeerManager::init(int baseStreamPort, int commSocketBasePort, 
                      const std::string& codecName) {
    baseStreamPort_ = baseStreamPort;
    commSocketBasePort_ = commSocketBasePort;
    codecName_ = codecName;
    
    // 포트 할당 테이블 초기화
    portAllocated_.resize(maxPeers_, false);
    commSocketAllocated_.resize(maxPeers_, false);
    
    LOG_INFO("PeerManager initialized: stream_port_base=%d, comm_port_base=%d, codec=%s",
             baseStreamPort, commSocketBasePort, codecName.c_str());
    
    return true;
}

bool PeerManager::addPeer(const std::string& peerId) {
    std::lock_guard<std::mutex> lock(peersMutex_);
    
    // 이미 존재하는지 확인
    if (peers_.find(peerId) != peers_.end()) {
        LOG_WARN("Peer %s already exists", peerId.c_str());
        return false;
    }
    
    // 피어 수 제한 확인
    if (peers_.size() >= static_cast<size_t>(maxPeers_)) {
        LOG_ERROR("Maximum number of peers reached (%d)", maxPeers_);
        return false;
    }
    
    // 포트 할당
    int streamPort = allocateStreamPort();
    int commSocket = allocateCommSocket();
    
    if (streamPort < 0 || commSocket < 0) {
        LOG_ERROR("Failed to allocate ports for peer %s", peerId.c_str());
        if (streamPort >= 0) releaseStreamPort(streamPort);
        if (commSocket >= 0) releaseCommSocket(commSocket);
        return false;
    }
    
    // 각 카메라에 대한 UDP 출력 추가
    const int DEVICE_COUNT = 2;  // RGB + Thermal
    bool cameraOutputsAdded = true;
    
    for (int camIdx = 0; camIdx < DEVICE_COUNT; camIdx++) {
        // 각 카메라별 포트 계산
        int camStreamPort = streamPort + camIdx;
        
        // Pipeline을 통해 CameraSource에 접근
        if (pipeline_) {
            auto camera = pipeline_->getCamera(camIdx);
            if (camera) {
                if (!camera->addPeerOutput(peerId, camStreamPort)) {
                    LOG_ERROR("Failed to add peer output for camera %d", camIdx);
                    cameraOutputsAdded = false;
                    
                    // 이미 추가된 카메라 출력 제거
                    for (int i = 0; i < camIdx; i++) {
                        auto prevCamera = pipeline_->getCamera(i);
                        if (prevCamera) {
                            prevCamera->removePeerOutput(peerId);
                        }
                    }
                    break;
                }
            }
        }
    }
    
    if (!cameraOutputsAdded) {
        releaseStreamPort(streamPort);
        releaseCommSocket(commSocket);
        return false;
    }
    
    // WebRTC sender 프로세스 생성
    auto sender = std::make_unique<WebRTCSenderProcess>(peerId, streamPort, commSocket);
    
    // 메시지 콜백 설정
    sender->setMessageCallback([this, peerId](const std::string& message) {
        handlePeerMessage(peerId, message);
    });
    
    // 프로세스 시작
    if (!sender->start(DEVICE_COUNT, codecName_)) {
        LOG_ERROR("Failed to start WebRTC sender for peer %s", peerId.c_str());
        
        // 카메라 출력 제거
        for (int camIdx = 0; camIdx < DEVICE_COUNT; camIdx++) {
            auto camera = pipeline_->getCamera(camIdx);
            if (camera) {
                camera->removePeerOutput(peerId);
            }
        }
        
        releaseStreamPort(streamPort);
        releaseCommSocket(commSocket);
        return false;
    }
    
    peers_[peerId] = std::move(sender);
    
    LOG_INFO("Added peer %s (stream_port=%d, comm_port=%d)",
             peerId.c_str(), streamPort, commSocket);
    
    return true;
}

bool PeerManager::removePeer(const std::string& peerId) {
    std::lock_guard<std::mutex> lock(peersMutex_);
    
    auto it = peers_.find(peerId);
    if (it == peers_.end()) {
        LOG_WARN("Peer %s not found", peerId.c_str());
        return false;
    }
    
    // WebRTCSenderProcess 정보 가져오기
    WebRTCSenderProcess* sender = it->second.get();
    
    // 프로세스 정보 로깅
    LOG_INFO("Removing peer %s (pid=%d, state=%d)", 
             peerId.c_str(), 
             sender->getPid(),
             static_cast<int>(sender->getState()));
    
    // 포트 정보 가져오기 (WebRTCSenderProcess에 getter 추가 필요)
    int streamPort = sender->getStreamPort();
    int commPort = sender->getCommPort();
    
    // 각 카메라에서 피어 출력 제거
    const int DEVICE_COUNT = 2;
    for (int camIdx = 0; camIdx < DEVICE_COUNT; camIdx++) {
        if (pipeline_) {
            auto camera = pipeline_->getCamera(camIdx);
            if (camera) {
                camera->removePeerOutput(peerId);
            }
        }
    }
    
    // 명시적으로 stop 호출
    sender->stop();
    
    // 잠시 대기 (프로세스 종료 시간)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 맵에서 제거 (unique_ptr이므로 자동 삭제)
    peers_.erase(it);
    
    // 포트 해제
    if (streamPort >= baseStreamPort_) {
        releaseStreamPort(streamPort);
    }
    if (commPort >= commSocketBasePort_) {
        releaseCommSocket(commPort);
    }
    
    LOG_INFO("Removed peer %s successfully", peerId.c_str());
    return true;
}

bool PeerManager::hasPeer(const std::string& peerId) const {
    std::lock_guard<std::mutex> lock(peersMutex_);
    return peers_.find(peerId) != peers_.end();
}

size_t PeerManager::getPeerCount() const {
    std::lock_guard<std::mutex> lock(peersMutex_);
    return peers_.size();
}

void PeerManager::handleSignalingMessage(const SignalingClient::Message& message) {
    LOG_INFO("Handling signaling message: type=%s, peer=%s",
              message.type.c_str(), message.peerId.c_str());
    
    if (message.type == "offer") {
        handleOffer(message.peerId, message.data);
    } else if (message.type == "answer") {
        handleAnswer(message.peerId, message.data);
    } else if (message.type == "candidate") {
        // ICE candidate 파싱
        JsonParser* parser = json_parser_new();
        if (json_parser_load_from_data(parser, message.data.c_str(), -1, nullptr)) {
            JsonNode* root = json_parser_get_root(parser);
            JsonObject* obj = json_node_get_object(root);
            
            const gchar* candidate = json_object_get_string_member(obj, "candidate");
            gint64 sdpMLineIndex = json_object_get_int_member(obj, "sdpMLineIndex");
            
            if (candidate) {
                handleIceCandidate(message.peerId, candidate, std::to_string(sdpMLineIndex));
            }
        }
        g_object_unref(parser);
    } else if (message.type == "ROOM_PEER_JOINED") {
        LOG_INFO("Peer %s joined the room", message.peerId.c_str());
        // 새 피어 참가
        if (!hasPeer(message.peerId)) {
            addPeer(message.peerId);
        }
    } else if (message.type == "ROOM_PEER_LEFT") {
        // 피어 퇴장
        removePeer(message.peerId);
    }
}

void PeerManager::broadcast(const std::string& type, const std::string& data) {
    std::lock_guard<std::mutex> lock(peersMutex_);
    
    for (const auto& pair : peers_) {
        pair.second->sendMessage(data);
    }
    
    LOG_DEBUG("Broadcast message to %zu peers: type=%s", peers_.size(), type.c_str());
}

void PeerManager::handleOffer(const std::string& peerId, const std::string& sdp) {
    std::lock_guard<std::mutex> lock(peersMutex_);
    
    auto it = peers_.find(peerId);
    if (it == peers_.end()) {
        LOG_ERROR("Peer %s not found for offer", peerId.c_str());
        return;
    }
    
    // webrtc_sender가 기대하는 형식으로 변환
    // {"sdp": {"type": "offer", "sdp": "..."}}
    JsonBuilder* builder = json_builder_new();
    json_builder_begin_object(builder);
    json_builder_set_member_name(builder, "sdp");
    json_builder_begin_object(builder);
        json_builder_set_member_name(builder, "type");
        json_builder_add_string_value(builder, "offer");
        json_builder_set_member_name(builder, "sdp");
        json_builder_add_string_value(builder, sdp.c_str());
    json_builder_end_object(builder);
    json_builder_end_object(builder);
    
    JsonGenerator* generator = json_generator_new();
    JsonNode* root = json_builder_get_root(builder);
    json_generator_set_root(generator, root);
    
    gchar* message = json_generator_to_data(generator, nullptr);
    it->second->sendMessage(message);
    
    g_free(message);
    g_object_unref(generator);
    g_object_unref(builder);
    
    LOG_INFO("Forwarded offer to webrtc_sender for peer %s", peerId.c_str());
}

void PeerManager::handleAnswer(const std::string& peerId, const std::string& sdp) {
    std::lock_guard<std::mutex> lock(peersMutex_);
    
    auto it = peers_.find(peerId);
    if (it == peers_.end()) {
        LOG_ERROR("Peer %s not found for answer", peerId.c_str());
        return;
    }
    
    // webrtc_sender가 기대하는 형식으로 변환
    // {"sdp": {"type": "answer", "sdp": "..."}}
    JsonBuilder* builder = json_builder_new();
    json_builder_begin_object(builder);
    json_builder_set_member_name(builder, "sdp");
    json_builder_begin_object(builder);
        json_builder_set_member_name(builder, "type");
        json_builder_add_string_value(builder, "answer");
        json_builder_set_member_name(builder, "sdp");
        json_builder_add_string_value(builder, sdp.c_str());
    json_builder_end_object(builder);
    json_builder_end_object(builder);
    
    JsonGenerator* generator = json_generator_new();
    JsonNode* root = json_builder_get_root(builder);
    json_generator_set_root(generator, root);
    
    gchar* message = json_generator_to_data(generator, nullptr);
    it->second->sendMessage(message);
    
    g_free(message);
    g_object_unref(generator);
    g_object_unref(builder);
    
    LOG_INFO("Forwarded answer to webrtc_sender for peer %s", peerId.c_str());
}

void PeerManager::handleIceCandidate(const std::string& peerId, const std::string& candidate, 
                                   const std::string& sdpMLineIndex) {
    std::lock_guard<std::mutex> lock(peersMutex_);
    
    auto it = peers_.find(peerId);
    if (it == peers_.end()) {
        LOG_ERROR("Peer %s not found for ICE candidate", peerId.c_str());
        return;
    }
    
    // webrtc_sender가 기대하는 형식으로 변환
    // {"ice": {"candidate": "...", "sdpMLineIndex": 0}}
    JsonBuilder* builder = json_builder_new();
    json_builder_begin_object(builder);
    json_builder_set_member_name(builder, "ice");
    json_builder_begin_object(builder);
        json_builder_set_member_name(builder, "candidate");
        json_builder_add_string_value(builder, candidate.c_str());
        json_builder_set_member_name(builder, "sdpMLineIndex");
        json_builder_add_int_value(builder, std::stoi(sdpMLineIndex));
    json_builder_end_object(builder);
    json_builder_end_object(builder);
    
    JsonGenerator* generator = json_generator_new();
    JsonNode* root = json_builder_get_root(builder);
    json_generator_set_root(generator, root);
    
    gchar* message = json_generator_to_data(generator, nullptr);
    it->second->sendMessage(message);
    
    g_free(message);
    g_object_unref(generator);
    g_object_unref(builder);
    
    LOG_DEBUG("Forwarded ICE candidate to webrtc_sender for peer %s", peerId.c_str());
}

// 포트 할당/해제 함수 구현
int PeerManager::allocateStreamPort() {
    std::lock_guard<std::mutex> lock(portMutex_);
    
    for (size_t i = 0; i < portAllocated_.size(); i++) {
        if (!portAllocated_[i]) {
            portAllocated_[i] = true;
            // 각 피어는 DEVICE_COUNT개의 연속된 포트를 사용
            return baseStreamPort_ + (i * 2);  // 2 = DEVICE_COUNT
        }
    }
    return -1;  // 사용 가능한 포트 없음
}

void PeerManager::releaseStreamPort(int port) {
    std::lock_guard<std::mutex> lock(portMutex_);
    
    int index = (port - baseStreamPort_) / 2;  // 2 = DEVICE_COUNT
    if (index >= 0 && index < static_cast<int>(portAllocated_.size())) {
        portAllocated_[index] = false;
    }
}

int PeerManager::allocateCommSocket() {
    std::lock_guard<std::mutex> lock(portMutex_);
    
    for (size_t i = 0; i < commSocketAllocated_.size(); i++) {
        if (!commSocketAllocated_[i]) {
            commSocketAllocated_[i] = true;
            return commSocketBasePort_ + i;
        }
    }
    return -1;
}

void PeerManager::releaseCommSocket(int socket) {
    std::lock_guard<std::mutex> lock(portMutex_);
    
    int index = socket - commSocketBasePort_;
    if (index >= 0 && index < static_cast<int>(commSocketAllocated_.size())) {
        commSocketAllocated_[index] = false;
    }
}

void PeerManager::handlePeerMessage(const std::string& peerId, const std::string& message) {
    LOG_DEBUG("Message from webrtc_sender (peer %s): %s", 
              peerId.c_str(), message.c_str());
    
    // webrtc_sender로부터 받은 메시지를 시그널링 서버로 전달
    JsonParser* parser = json_parser_new();
    if (!json_parser_load_from_data(parser, message.c_str(), -1, nullptr)) {
        LOG_ERROR("Failed to parse JSON message");
        g_object_unref(parser);
        return;
    }
    
    JsonNode* root = json_parser_get_root(parser);
    if (!JSON_NODE_HOLDS_OBJECT(root)) {
        LOG_ERROR("JSON root is not an object");
        g_object_unref(parser);
        return;
    }
    
    JsonObject* obj = json_node_get_object(root);
    
    // webrtc_sender의 메시지 형식 확인
    // 형식: { "peerType": "camera", "action": "...", "message": {...} }
    
    const gchar* action = json_object_get_string_member(obj, "action");
    if (!action) {
        LOG_ERROR("No action field in message");
        g_object_unref(parser);
        return;
    }
    
    // message 필드가 있는지 확인
    if (!json_object_has_member(obj, "message")) {
        LOG_ERROR("No message field");
        g_object_unref(parser);
        return;
    }
    
    JsonObject* msgObj = json_object_get_object_member(obj, "message");
    if (!msgObj) {
        LOG_ERROR("Message field is not an object");
        g_object_unref(parser);
        return;
    }
    
    if (g_strcmp0(action, "answer") == 0) {
        // answer 처리
        if (json_object_has_member(msgObj, "sdp")) {
            JsonObject* sdpObj = json_object_get_object_member(msgObj, "sdp");
            if (sdpObj) {
                const gchar* sdpType = json_object_get_string_member(sdpObj, "type");
                const gchar* sdpStr = json_object_get_string_member(sdpObj, "sdp");
                
                if (sdpType && sdpStr && signalingClient_) {
                    signalingClient_->sendToPeer(peerId, "answer", sdpStr);
                }
            }
        }
    } 
    else if (g_strcmp0(action, "candidate") == 0) {
        // ICE candidate 처리
        if (json_object_has_member(msgObj, "ice")) {
            JsonObject* iceObj = json_object_get_object_member(msgObj, "ice");
            if (iceObj) {
                const gchar* candidate = json_object_get_string_member(iceObj, "candidate");
                gint64 sdpMLineIndex = json_object_get_int_member(iceObj, "sdpMLineIndex");
                
                if (candidate && signalingClient_) {
                    // ICE candidate를 시그널링 서버로 전달
                    JsonBuilder* builder = json_builder_new();
                    json_builder_begin_object(builder);
                    json_builder_set_member_name(builder, "candidate");
                    json_builder_add_string_value(builder, candidate);
                    json_builder_set_member_name(builder, "sdpMLineIndex");
                    json_builder_add_int_value(builder, sdpMLineIndex);
                    json_builder_end_object(builder);
                    
                    JsonGenerator* gen = json_generator_new();
                    JsonNode* node = json_builder_get_root(builder);
                    json_generator_set_root(gen, node);
                    
                    gchar* data = json_generator_to_data(gen, nullptr);
                    signalingClient_->sendToPeer(peerId, "ice_candidate", data);
                    
                    g_free(data);
                    g_object_unref(gen);
                    g_object_unref(builder);
                }
            }
        }
    }
    else if (g_strcmp0(action, "offer") == 0) {
        // offer 처리 (필요한 경우)
        if (json_object_has_member(msgObj, "sdp")) {
            JsonObject* sdpObj = json_object_get_object_member(msgObj, "sdp");
            if (sdpObj) {
                const gchar* sdpType = json_object_get_string_member(sdpObj, "type");
                const gchar* sdpStr = json_object_get_string_member(sdpObj, "sdp");
                
                if (sdpType && sdpStr && signalingClient_) {
                    signalingClient_->sendToPeer(peerId, "offer", sdpStr);
                }
            }
        }
    }
    
    g_object_unref(parser);
}

void PeerManager::stopAllProcesses() {
    std::lock_guard<std::mutex> lock(peersMutex_);
    
    LOG_INFO("Stopping all peer processes...");
    peers_.clear();  // 소멸자에서 자동으로 stop() 호출됨
}