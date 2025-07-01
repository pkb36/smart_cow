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
    LOG_DEBUG("SocketCommUDP::close() called");
    
    // 1. 먼저 running_ 플래그 설정
    running_ = false;
    
    // 2. 소켓을 즉시 닫아서 recvfrom을 중단시킴
    if (socket_ >= 0) {
        LOG_DEBUG("Shutting down socket %d", socket_);
        
        // shutdown으로 읽기 중단 (EXIT 메시지 전송 전에!)
        shutdown(socket_, SHUT_RD);
        
        // EXIT 메시지 전송 시도 (실패해도 무시)
        try {
            if ((type_ == Type::SERVER && hasClient_) || type_ == Type::CLIENT) {
                sendMessage("EXIT");
            }
        } catch (...) {
            LOG_DEBUG("Failed to send EXIT message, ignoring");
        }
        
        // 소켓 완전히 닫기
        shutdown(socket_, SHUT_RDWR);
        ::close(socket_);
        socket_ = -1;
    }
    
    // 3. 수신 스레드 종료 대기 (타임아웃 포함)
    if (receiveThread_.joinable()) {
        LOG_DEBUG("Waiting for receive thread to finish...");
        
        // C++11 방식의 타임아웃
        auto start = std::chrono::steady_clock::now();
        while (receiveThread_.joinable()) {
            if (std::chrono::steady_clock::now() - start > std::chrono::seconds(2)) {
                LOG_ERROR("Receive thread join timeout!");
                // detach하고 포기
                receiveThread_.detach();
                break;
            }
            
            // joinable 상태 확인
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            
            // join 시도
            if (receiveThread_.joinable()) {
                try {
                    receiveThread_.join();
                    LOG_DEBUG("Receive thread joined successfully");
                    break;
                } catch (...) {
                    // join 실패 시 계속 시도
                }
            }
        }
    }
    
    LOG_DEBUG("SocketCommUDP::close() completed");
}

void SocketCommUDP::receiveThread() {
    char buffer[4096];
    struct sockaddr_in fromAddr;
    socklen_t fromLen;
    
    LOG_DEBUG("UDP receive thread started");
    
    while (running_ && socket_ >= 0) {  // socket_ 체크 추가
        fromLen = sizeof(fromAddr);
        
        // select 사용하여 타임아웃 설정
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(socket_, &readfds);
        
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 500000;  // 500ms
        
        int ret = select(socket_ + 1, &readfds, NULL, NULL, &tv);
        
        if (ret < 0) {
            if (errno == EBADF) {  // 소켓이 닫힘
                LOG_DEBUG("Socket closed, exiting receive thread");
                break;
            }
            if (errno != EINTR) {
                LOG_ERROR("select error: %s", strerror(errno));
                break;
            }
            continue;
        } else if (ret == 0) {
            // 타임아웃 - running_ 체크
            continue;
        }
        
        // 소켓이 닫혔는지 재확인
        if (socket_ < 0) {
            LOG_DEBUG("Socket closed during select, exiting");
            break;
        }
        
        ssize_t recvLen = recvfrom(socket_, buffer, sizeof(buffer) - 1, 0,
                                  (struct sockaddr*)&fromAddr, &fromLen);
        
        if (recvLen < 0) {
            if (errno == EBADF || errno == ENOTCONN) {
                LOG_DEBUG("Socket error, exiting receive thread");
                break;
            }
            if (running_ && errno != EINTR && errno != EAGAIN) {
                LOG_ERROR("recvfrom failed: %s", strerror(errno));
            }
            continue;
        }
        
        if (recvLen == 0) {
            LOG_DEBUG("Received 0 bytes, connection closed?");
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