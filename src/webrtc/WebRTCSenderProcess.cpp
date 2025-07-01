#include "WebRTCSenderProcess.h"
#include "../utils/SocketCommUDP.h"
#include "../utils/ProcessManager.h"
#include "../utils/Logger.h"
#include <sstream>
#include <unistd.h>
#include <arpa/inet.h>  
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstring>
#include <csignal>

extern "C" {
    SOCKETINFO* init_socket_comm_server(int port);
    void close_socket_comm(SOCKETINFO* info);
}

WebRTCSenderProcess::WebRTCSenderProcess(const std::string& peerId, 
                                       int streamPort, int commSocketPort)
    : peerId_(peerId)
    , streamPort_(streamPort)
    , commSocketPort_(commSocketPort)
    , state_(State::NEW)
    , childPid_(-1)
    , socketComm_(nullptr)
    , running_(false) {
    
    LOG_INFO("WebRTCSenderProcess created: peer=%s, stream_port=%d, comm_port=%d",
             peerId.c_str(), streamPort, commSocketPort);
}

WebRTCSenderProcess::~WebRTCSenderProcess() {
    LOG_INFO("WebRTCSenderProcess destructor called for peer %s", peerId_.c_str());
    
    // stop이 아직 호출되지 않았다면 호출
    if (state_ != State::STOPPED) {
        stop();
    }
    
    // 프로세스가 여전히 실행 중인지 확인
    if (childPid_ > 0) {
        // kill 시그널 전송
        if (kill(childPid_, 0) == 0) {
            LOG_WARN("Process %d still running, sending SIGTERM", childPid_);
            kill(childPid_, SIGTERM);
            
            // 잠시 대기
            usleep(100000);  // 100ms
            
            // 여전히 살아있으면 SIGKILL
            if (kill(childPid_, 0) == 0) {
                LOG_WARN("Process %d still running, sending SIGKILL", childPid_);
                kill(childPid_, SIGKILL);
            }
        }
    }
}

bool WebRTCSenderProcess::start(int deviceCount, const std::string& codecName) {
    if (state_ != State::NEW && state_ != State::STOPPED) {
        LOG_WARN("Cannot start WebRTCSenderProcess in state %d", static_cast<int>(state_));
        return false;
    }
    
    state_ = State::STARTING;

    // 소켓 서버 시작 (webrtc_sender와 통신용)
    socketComm_ = std::make_unique<SocketCommUDP>(SocketCommUDP::Type::SERVER, commSocketPort_);
    socketComm_->setMessageCallback([this](const std::string& message, const struct sockaddr_in& fromAddr) {
        LOG_DEBUG("Received message from webrtc_sender (%s:%d): %s", 
                  inet_ntoa(fromAddr.sin_addr), 
                  ntohs(fromAddr.sin_port),
                  message.c_str());
        
        if (messageCallback_) {
            messageCallback_(message);
        }
    });
    
    if (!socketComm_->startServer()) {
        LOG_ERROR("Failed to start socket server on port %d", commSocketPort_);
        stop();
        return false;
    }
    
    // webrtc_sender 프로세스 실행 명령 구성
    std::ostringstream cmd;
    cmd << "./webrtc_sender"
        << " --peer_id=" << peerId_
        << " --stream_cnt=" << deviceCount
        << " --stream_base_port=" << streamPort_
        << " --comm_socket_port=" << commSocketPort_
        << " --codec_name=" << codecName;
    
    // 프로세스 시작
    ProcessManager& pm = ProcessManager::getInstance();
    childPid_ = pm.startProcess("webrtc_sender_" + peerId_, cmd.str());
    
    if (childPid_ <= 0) {
        LOG_ERROR("Failed to start webrtc_sender process");
        state_ = State::STOPPED;
        return false;
    }
    
    LOG_INFO("Started webrtc_sender process: pid=%d, cmd=%s", childPid_, cmd.str().c_str());
    
    state_ = State::RUNNING;
    
    return true;
}

void WebRTCSenderProcess::stop() {
    if (state_ == State::STOPPED || state_ == State::STOPPING) {
        return;
    }
    
    state_ = State::STOPPING;
    running_ = false;
    
    // 리스너 스레드 종료
    if (listenerThread_.joinable()) {
        listenerThread_.join();
    }
    
    // 소켓 닫기
    socketComm_.reset();
    
    // 프로세스 종료
    if (childPid_ > 0) {
        ProcessManager& pm = ProcessManager::getInstance();
        pm.stopProcess(childPid_);
        childPid_ = -1;
    }
    
    state_ = State::STOPPED;
    LOG_INFO("WebRTCSenderProcess stopped for peer %s", peerId_.c_str());
}

WebRTCSenderProcess::State WebRTCSenderProcess::getState() const {
    return state_;
}

pid_t WebRTCSenderProcess::getPid() const {
    return childPid_;
}

bool WebRTCSenderProcess::isRunning() const {
    if (childPid_ <= 0) {
        return false;
    }
    
    ProcessManager& pm = ProcessManager::getInstance();
    return pm.isProcessRunning(childPid_);
}

bool WebRTCSenderProcess::sendMessage(const std::string& message) {
    if (!socketComm_ || !socketComm_->isConnected()) {
        LOG_ERROR("Socket not connected for peer %s", peerId_.c_str());
        return false;
    }
    return socketComm_->sendMessage(message);
}

void WebRTCSenderProcess::setMessageCallback(MessageCallback callback) {
    messageCallback_ = callback;
}