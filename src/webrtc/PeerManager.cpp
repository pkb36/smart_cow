#include "PeerManager.h"
#include "WebRTCSenderProcess.h"
#include "../pipeline/Pipeline.h"
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
    
    // WebRTC sender 프로세스 생성
    auto sender = std::make_unique<WebRTCSenderProcess>(peerId, streamPort, commSocket);
    
    // 메시지 콜백 설정
    sender->setMessageCallback([this, peerId](const std::string& message) {
        handlePeerMessage(peerId, message);
    });
    
    // 프로세스 시작
    if (!sender->start(2, codecName_)) {  // device_count = 2
        LOG_ERROR("Failed to start WebRTC sender for peer %s", peerId.c_str());
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
    
    // 포트 해제
    // TODO: WebRTCSenderProcess에서 포트 정보 가져오기
    
    // 프로세스 정지 (소멸자에서 자동으로 처리됨)
    peers_.erase(it);
    
    LOG_INFO("Removed peer %s", peerId.c_str());
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
    LOG_DEBUG("Handling signaling message: type=%s, peer=%s",
              message.type.c_str(), message.peerId.c_str());
    
    if (message.type == "offer") {
        handleOffer(message.peerId, message.data);
    } else if (message.type == "answer") {
        handleAnswer(message.peerId, message.data);
    } else if (message.type == "ice_candidate") {
        // ICE candidate 파싱
        JsonParser* parser = json_parser_new();
        if (json_parser_load_from_data(parser, message.data.c_str(), -1, nullptr)) {
            JsonNode* root = json_parser_get_root(parser);
            JsonObject* obj = json_node_get_object(root);
            
            const gchar* candidate = json_object_get_string_member(obj, "candidate");
            const gchar* sdpMLineIndex = json_object_get_string_member(obj, "sdpMLineIndex");
            
            if (candidate && sdpMLineIndex) {
                handleIceCandidate(message.peerId, candidate, sdpMLineIndex);
            }
        }
        g_object_unref(parser);
    } else if (message.type == "peer_join") {
        // 새 피어 참가
        if (!hasPeer(message.peerId)) {
            addPeer(message.peerId);
        }
    } else if (message.type == "peer_leave") {
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
    
    // WebRTC sender 프로세스로 offer 전달
    JsonBuilder* builder = json_builder_new();
    json_builder_begin_object(builder);
    json_builder_set_member_name(builder, "type");
    json_builder_add_string_value(builder, "offer");
    json_builder_set_member_name(builder, "sdp");
    json_builder_add_string_value(builder, sdp.c_str());
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
    // Offer와 동일한 방식으로 처리
    std::lock_guard<std::mutex> lock(peersMutex_);
    
    auto it = peers_.find(peerId);
    if (it == peers_.end()) {
        LOG_ERROR("Peer %s not found for answer", peerId.c_str());
        return;
    }
    
    JsonBuilder* builder = json_builder_new();
    json_builder_begin_object(builder);
    json_builder_set_member_name(builder, "type");
    json_builder_add_string_value(builder, "answer");
    json_builder_set_member_name(builder, "sdp");
    json_builder_add_string_value(builder, sdp.c_str());
    json_builder_end_object(builder);
    
    JsonGenerator* generator = json_generator_new();
    JsonNode* root = json_builder_get_root(builder);
    json_generator_set_root(generator, root);
    
    gchar* message = json_generator_to_data(generator, nullptr);
    it->second->sendMessage(message);
    
    g_free(message);
    g_object_unref(generator);
    g_object_unref(builder);
}

void PeerManager::handleIceCandidate(const std::string& peerId, 
                                    const std::string& candidate,
                                    const std::string& sdpMLineIndex) {
    std::lock_guard<std::mutex> lock(peersMutex_);
    
    auto it = peers_.find(peerId);
    if (it == peers_.end()) {
        LOG_ERROR("Peer %s not found for ICE candidate", peerId.c_str());
        return;
    }
    
    JsonBuilder* builder = json_builder_new();
    json_builder_begin_object(builder);
    json_builder_set_member_name(builder, "type");
    json_builder_add_string_value(builder, "ice_candidate");
    json_builder_set_member_name(builder, "candidate");
    json_builder_add_string_value(builder, candidate.c_str());
    json_builder_set_member_name(builder, "sdpMLineIndex");
    json_builder_add_string_value(builder, sdpMLineIndex.c_str());
    json_builder_end_object(builder);
    
    JsonGenerator* generator = json_generator_new();
    JsonNode* root = json_builder_get_root(builder);
    json_generator_set_root(generator, root);
    
    gchar* message = json_generator_to_data(generator, nullptr);
    it->second->sendMessage(message);
    
    g_free(message);
    g_object_unref(generator);
    g_object_unref(builder);
}

int PeerManager::allocateStreamPort() {
    std::lock_guard<std::mutex> lock(portMutex_);
    
    for (size_t i = 0; i < portAllocated_.size(); i++) {
        if (!portAllocated_[i]) {
            portAllocated_[i] = true;
            return baseStreamPort_ + i;
        }
    }
    
    return -1;
}

void PeerManager::releaseStreamPort(int port) {
    std::lock_guard<std::mutex> lock(portMutex_);
    
    int index = port - baseStreamPort_;
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
    // 메시지 타입에 따라 처리
    JsonParser* parser = json_parser_new();
    if (json_parser_load_from_data(parser, message.c_str(), -1, nullptr)) {
        JsonNode* root = json_parser_get_root(parser);
        JsonObject* obj = json_node_get_object(root);
        
        const gchar* type = json_object_get_string_member(obj, "type");
        
        if (g_strcmp0(type, "answer") == 0) {
            // answer를 시그널링 서버로 전달
            const gchar* sdp = json_object_get_string_member(obj, "sdp");
            if (sdp && signalingClient_) {
                signalingClient_->sendToPeer(peerId, "answer", sdp);
            }
        } else if (g_strcmp0(type, "ice_candidate") == 0) {
            // ICE candidate를 시그널링 서버로 전달
            const gchar* candidate = json_object_get_string_member(obj, "candidate");
            const gchar* sdpMLineIndex = json_object_get_string_member(obj, "sdpMLineIndex");
            
            if (candidate && sdpMLineIndex && signalingClient_) {
                JsonBuilder* builder = json_builder_new();
                json_builder_begin_object(builder);
                json_builder_set_member_name(builder, "candidate");
                json_builder_add_string_value(builder, candidate);
                json_builder_set_member_name(builder, "sdpMLineIndex");
                json_builder_add_string_value(builder, sdpMLineIndex);
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
    g_object_unref(parser);
}

void PeerManager::stopAllProcesses() {
    std::lock_guard<std::mutex> lock(peersMutex_);
    
    LOG_INFO("Stopping all peer processes...");
    peers_.clear();  // 소멸자에서 자동으로 stop() 호출됨
}