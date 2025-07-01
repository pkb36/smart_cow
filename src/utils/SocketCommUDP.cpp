#include "SocketCommUDP.h"
#include "Logger.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <errno.h>

SocketCommUDP::SocketCommUDP(Type type, int port)
    : type_(type)
    , port_(port)
    , socket_(-1)
    , hasClient_(false)
    , running_(false) {
    memset(&serverAddr_, 0, sizeof(serverAddr_));
    memset(&clientAddr_, 0, sizeof(clientAddr_));
}

SocketCommUDP::~SocketCommUDP() {
    close();
}

bool SocketCommUDP::startServer() {
    if (type_ != Type::SERVER) {
        LOG_ERROR("Cannot start server on client socket");
        return false;
    }
    
    // UDP 소켓 생성
    socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0) {
        LOG_ERROR("Failed to create UDP socket: %s", strerror(errno));
        return false;
    }
    
    // SO_REUSEADDR 설정
    int opt = 1;
    if (setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        LOG_ERROR("Failed to set SO_REUSEADDR: %s", strerror(errno));
        ::close(socket_);
        socket_ = -1;
        return false;
    }
    
    // 바인드
    serverAddr_.sin_family = AF_INET;
    serverAddr_.sin_addr.s_addr = INADDR_ANY;
    serverAddr_.sin_port = htons(port_);
    
    if (bind(socket_, (struct sockaddr*)&serverAddr_, sizeof(serverAddr_)) < 0) {
        LOG_ERROR("Failed to bind on port %d: %s", port_, strerror(errno));
        ::close(socket_);
        socket_ = -1;
        return false;
    }
    
    // 수신 스레드 시작
    running_ = true;
    receiveThread_ = std::thread(&SocketCommUDP::receiveThread, this);
    
    LOG_INFO("UDP server started on port %d", port_);
    return true;
}

bool SocketCommUDP::connectToServer(const std::string& host) {
    if (type_ != Type::CLIENT) {
        LOG_ERROR("Cannot connect on server socket");
        return false;
    }
    
    // UDP 소켓 생성
    socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0) {
        LOG_ERROR("Failed to create UDP socket: %s", strerror(errno));
        return false;
    }
    
    // 서버 주소 설정
    serverAddr_.sin_family = AF_INET;
    serverAddr_.sin_port = htons(port_);
    
    if (inet_pton(AF_INET, host.c_str(), &serverAddr_.sin_addr) <= 0) {
        LOG_ERROR("Invalid address: %s", host.c_str());
        ::close(socket_);
        socket_ = -1;
        return false;
    }
    
    // UDP는 연결 개념이 없으므로 CONNECT 메시지 전송
    sendMessage("CONNECT");
    
    // 수신 스레드 시작
    running_ = true;
    receiveThread_ = std::thread(&SocketCommUDP::receiveThread, this);
    
    LOG_INFO("UDP client initialized for %s:%d", host.c_str(), port_);
    return true;
}

bool SocketCommUDP::sendMessage(const std::string& message) {
    if (socket_ < 0) {
        LOG_ERROR("Socket not initialized");
        return false;
    }
    
    struct sockaddr_in* targetAddr = nullptr;
    
    if (type_ == Type::SERVER) {
        if (!hasClient_) {
            LOG_ERROR("No client connected");
            return false;
        }
        targetAddr = &clientAddr_;
    } else {
        targetAddr = &serverAddr_;
    }
    
    ssize_t sent = sendto(socket_, message.c_str(), message.length(), 0,
                         (struct sockaddr*)targetAddr, sizeof(*targetAddr));
    
    if (sent < 0) {
        LOG_ERROR("Failed to send message: %s", strerror(errno));
        return false;
    }
    
    LOG_INFO("Sent %zd bytes: %s", sent, message.c_str());
    return true;
}

bool SocketCommUDP::sendMessageTo(const std::string& message, const struct sockaddr_in& addr) {
    if (socket_ < 0) {
        LOG_ERROR("Socket not initialized");
        return false;
    }
    
    ssize_t sent = sendto(socket_, message.c_str(), message.length(), 0,
                         (struct sockaddr*)&addr, sizeof(addr));
    
    if (sent < 0) {
        LOG_ERROR("Failed to send message: %s", strerror(errno));
        return false;
    }
    
    return true;
}

void SocketCommUDP::setMessageCallback(MessageCallback callback) {
    messageCallback_ = callback;
}

void SocketCommUDP::close() {
    running_ = false;
    
    if (socket_ >= 0) {
        // EXIT 메시지 전송 (호환성)
        sendMessage("EXIT");
        ::close(socket_);
        socket_ = -1;
    }
    
    if (receiveThread_.joinable()) {
        receiveThread_.join();
    }
}

void SocketCommUDP::receiveThread() {
    char buffer[4096];
    struct sockaddr_in fromAddr;
    socklen_t fromLen;
    
    LOG_DEBUG("UDP receive thread started");
    
    while (running_) {
        fromLen = sizeof(fromAddr);
        ssize_t recvLen = recvfrom(socket_, buffer, sizeof(buffer) - 1, 0,
                                  (struct sockaddr*)&fromAddr, &fromLen);
        
        if (recvLen < 0) {
            if (running_ && errno != EINTR) {
                LOG_ERROR("recvfrom failed: %s", strerror(errno));
            }
            continue;
        }
        
        buffer[recvLen] = '\0';
        LOG_DEBUG("Received %zd bytes from %s:%d: %s", 
                 recvLen, inet_ntoa(fromAddr.sin_addr), ntohs(fromAddr.sin_port), buffer);
        
        // CONNECT 메시지 처리 (서버인 경우)
        if (type_ == Type::SERVER && strcmp(buffer, "CONNECT") == 0) {
            std::lock_guard<std::mutex> lock(mutex_);
            clientAddr_ = fromAddr;
            hasClient_ = true;
            LOG_INFO("Client connected from %s:%d", 
                    inet_ntoa(fromAddr.sin_addr), ntohs(fromAddr.sin_port));
            continue;
        }
        
        // EXIT 메시지 처리
        if (strcmp(buffer, "EXIT") == 0) {
            LOG_INFO("Received EXIT message");
            break;
        }
        
        // 콜백 호출
        if (messageCallback_) {
            messageCallback_(std::string(buffer, recvLen), fromAddr);
        }
    }
    
    LOG_DEBUG("UDP receive thread ended");
}

bool SocketCommUDP::isConnected() const {
    return running_;
}