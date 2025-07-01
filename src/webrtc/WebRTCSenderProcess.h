#ifndef WEBRTC_SENDER_PROCESS_H
#define WEBRTC_SENDER_PROCESS_H

#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <functional>
#include <sys/types.h>
#include "../utils/SocketCommUDP.h" 

// Forward declarations
typedef struct _SOCKETINFO SOCKETINFO;

class WebRTCSenderProcess {
public:
    enum class State {
        NEW,
        STARTING,
        RUNNING,
        STOPPING,
        STOPPED
    };
    
    WebRTCSenderProcess(const std::string& peerId, int streamPort, int commSocketPort);
    ~WebRTCSenderProcess();
    
    bool start(int deviceCount, const std::string& codecName);
    void stop();
    
    // 프로세스 상태
    State getState() const;
    pid_t getPid() const;
    bool isRunning() const;
    
    // 통신 소켓을 통한 메시지 전송
    bool sendMessage(const std::string& message);
    
    // 메시지 수신 콜백
    using MessageCallback = std::function<void(const std::string&)>;
    void setMessageCallback(MessageCallback callback);
    
    const std::string& getPeerId() const { return peerId_; }
    int getStreamPort() const { return streamPort_; }
    int getCommPort() const { return commSocketPort_; }
    
public:
    int streamPort_;
    int commSocketPort_;
    
private:
    std::string peerId_;
    State state_;
    
    pid_t childPid_;
    std::unique_ptr<SocketCommUDP> socketComm_;
    
    std::atomic<bool> running_;
    MessageCallback messageCallback_;
};

#endif // WEBRTC_SENDER_PROCESS_H