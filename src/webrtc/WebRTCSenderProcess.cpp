#include "WebRTCSenderProcess.h"
#include "../utils/SocketComm.h"
#include "../utils/ProcessManager.h"
#include "../utils/Logger.h"
#include <sstream>
#include <unistd.h>
#include <arpa/inet.h>  
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstring>

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
    , socket_(nullptr)
    , running_(false) {
    
    LOG_INFO("WebRTCSenderProcess created: peer=%s, stream_port=%d, comm_port=%d",
             peerId.c_str(), streamPort, commSocketPort);
}

WebRTCSenderProcess::~WebRTCSenderProcess() {
    stop();
}

bool WebRTCSenderProcess::start(int deviceCount, const std::string& codecName) {
    if (state_ != State::NEW && state_ != State::STOPPED) {
        LOG_WARN("Cannot start WebRTCSenderProcess in state %d", static_cast<int>(state_));
        return false;
    }
    
    state_ = State::STARTING;
    
    // webrtc_sender 프로세스 실행 명령 구성
    std::ostringstream cmd;
    cmd << "./webrtc_sender"
        << " --peer_id=" << peerId_
        << " --device_cnt=" << deviceCount
        << " --stream_port=" << streamPort_
        << " --comm_port=" << commSocketPort_
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
    
    // 소켓 서버 시작 (webrtc_sender와 통신용)
    socket_ = init_socket_comm_server(commSocketPort_);
    if (!socket_) {
        LOG_ERROR("Failed to create comm socket on port %d", commSocketPort_);
        stop();
        return false;
    }
    
    socket_->call_fun = &WebRTCSenderProcess::socketMessageCallback;
    socket_->data = this;
    
    // 리스너 스레드 시작
    running_ = true;
    listenerThread_ = std::thread(&WebRTCSenderProcess::socketListenerThread, this);
    
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
    if (socket_) {
        close_socket_comm(socket_);
        socket_ = nullptr;
    }
    
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
    if (!socket_ || !socket_->connect) {
        LOG_ERROR("Socket not connected for peer %s", peerId_.c_str());
        return false;
    }
    
    // 메시지 길이 + 메시지 전송
    uint32_t msgLen = message.length();
    uint32_t netLen = htonl(msgLen);
    
    if (send(socket_->socket, &netLen, sizeof(netLen), 0) != sizeof(netLen)) {
        LOG_ERROR("Failed to send message length");
        return false;
    }
    
    if (send(socket_->socket, message.c_str(), msgLen, 0) != static_cast<ssize_t>(msgLen)) {
        LOG_ERROR("Failed to send message");
        return false;
    }
    
    LOG_DEBUG("Sent message to webrtc_sender: %s", message.c_str());
    return true;
}

void WebRTCSenderProcess::setMessageCallback(MessageCallback callback) {
    messageCallback_ = callback;
}

void WebRTCSenderProcess::socketListenerThread() {
    LOG_DEBUG("Socket listener thread started for peer %s", peerId_.c_str());
    
    while (running_) {
        // Accept 연결
        struct sockaddr_in clientAddr;
        socklen_t clientLen = sizeof(clientAddr);
        
        int clientSocket = accept(socket_->socket, (struct sockaddr*)&clientAddr, &clientLen);
        if (clientSocket < 0) {
            if (running_ && errno != EINTR) {
                LOG_ERROR("Accept failed: %s", strerror(errno));
            }
            continue;
        }
        
        LOG_INFO("webrtc_sender connected on comm socket");
        
        // 기존 연결 닫기
        if (socket_->connect) {
            close(socket_->socket);
        }
        
        socket_->socket = clientSocket;
        socket_->connect = 1;
        
        // 메시지 수신 루프
        char buffer[4096];
        while (running_ && socket_->connect) {
            // 메시지 길이 읽기
            uint32_t msgLen;
            ssize_t ret = recv(clientSocket, &msgLen, sizeof(msgLen), MSG_WAITALL);
            if (ret != sizeof(msgLen)) {
                if (ret == 0) {
                    LOG_INFO("webrtc_sender disconnected");
                } else if (ret < 0 && running_) {
                    LOG_ERROR("Failed to receive message length: %s", strerror(errno));
                }
                break;
            }
            
            msgLen = ntohl(msgLen);
            if (msgLen > sizeof(buffer) - 1) {
                LOG_ERROR("Message too large: %u bytes", msgLen);
                break;
            }
            
            // 메시지 읽기
            ret = recv(clientSocket, buffer, msgLen, MSG_WAITALL);
            if (ret != static_cast<ssize_t>(msgLen)) {
                LOG_ERROR("Failed to receive complete message");
                break;
            }
            
            buffer[msgLen] = '\0';
            
            // 콜백 호출
            if (socket_->call_fun) {
                socket_->call_fun(buffer, msgLen, socket_->data);
            }
        }
        
        socket_->connect = 0;
        close(clientSocket);
    }
    
    LOG_DEBUG("Socket listener thread ended for peer %s", peerId_.c_str());
}

void WebRTCSenderProcess::socketMessageCallback(char* data, int len, void* arg) {
    WebRTCSenderProcess* self = static_cast<WebRTCSenderProcess*>(arg);
    
    if (self->messageCallback_) {
        self->messageCallback_(std::string(data, len));
    }
}