#ifndef PEER_MANAGER_H
#define PEER_MANAGER_H

#include <memory>
#include <unordered_map>
#include <mutex>
#include <string>
#include <sys/types.h>
#include "../signaling/SignalingClient.h"

class WebRTCSenderProcess;
class Pipeline;

class PeerManager {
public:
    PeerManager(Pipeline* pipeline, int maxPeers);
    ~PeerManager();
    
    bool init(int baseStreamPort, int commSocketBasePort, const std::string& codecName);
    
    // Peer 관리
    bool addPeer(const std::string& peerId);
    bool removePeer(const std::string& peerId);
    bool hasPeer(const std::string& peerId) const;
    size_t getPeerCount() const;
    
    // 시그널링 메시지 처리
    void handleSignalingMessage(const SignalingClient::Message& message);
    
    // 모든 peer에게 메시지 전송
    void broadcast(const std::string& type, const std::string& data);
    
private:
    void handleOffer(const std::string& peerId, const std::string& sdp);
    void handleAnswer(const std::string& peerId, const std::string& sdp);
    void handleIceCandidate(const std::string& peerId, const std::string& candidate, 
                           const std::string& sdpMLineIndex);
    
    int allocateStreamPort();
    void releaseStreamPort(int port);
    int allocateCommSocket();
    void releaseCommSocket(int socket);
    
private:
    Pipeline* pipeline_;
    int maxPeers_;
    int baseStreamPort_;
    int commSocketBasePort_;
    std::string codecName_;
    
    mutable std::mutex peersMutex_;
    std::unordered_map<std::string, std::unique_ptr<WebRTCSenderProcess>> peers_;
    
    // 포트 할당 관리
    std::vector<bool> portAllocated_;
    std::vector<bool> commSocketAllocated_;
    std::mutex portMutex_;
};

#endif // PEER_MANAGER_H