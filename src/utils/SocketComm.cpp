#include "SocketComm.h"
#include "Logger.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <errno.h>

SocketComm::SocketComm(Type type, int port)
    : type_(type)
    , port_(port)
    , listenSocket_(-1)
    , clientSocket_(-1)
    , running_(false)
    , connected_(false) {
}

SocketComm::~SocketComm() {
    close();
}

bool SocketComm::startServer() {
    if (type_ != Type::SERVER) {
        LOG_ERROR("Cannot start server on client socket");
        return false;
    }
    
    // 리스닝 소켓 생성
    listenSocket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listenSocket_ < 0) {
        LOG_ERROR("Failed to create socket: %s", strerror(errno));
        return false;
    }
    
    // SO_REUSEADDR 설정
    int opt = 1;
    if (setsockopt(listenSocket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        LOG_ERROR("Failed to set SO_REUSEADDR: %s", strerror(errno));
        ::close(listenSocket_);
        listenSocket_ = -1;
        return false;
    }
    
    // 바인드
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port_);
    
    if (bind(listenSocket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        LOG_ERROR("Failed to bind on port %d: %s", port_, strerror(errno));
        ::close(listenSocket_);
        listenSocket_ = -1;
        return false;
    }
    
    // 리슨
    if (listen(listenSocket_, 5) < 0) {  // 백로그 5
        LOG_ERROR("Failed to listen: %s", strerror(errno));
        ::close(listenSocket_);
        listenSocket_ = -1;
        return false;
    }
    
    // 서버 스레드 시작
    running_ = true;
    serverThread_ = std::thread(&SocketComm::serverThread, this);
    
    LOG_INFO("Socket server started on port %d", port_);
    return true;
}

bool SocketComm::connectToServer(const std::string& host) {
    if (type_ != Type::CLIENT) {
        LOG_ERROR("Cannot connect on server socket");
        return false;
    }
    
    // 클라이언트 소켓 생성
    clientSocket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket_ < 0) {
        LOG_ERROR("Failed to create socket: %s", strerror(errno));
        return false;
    }
    
    // 서버 주소 설정
    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port_);
    
    if (inet_pton(AF_INET, host.c_str(), &serverAddr.sin_addr) <= 0) {
        LOG_ERROR("Invalid address: %s", host.c_str());
        ::close(clientSocket_);
        clientSocket_ = -1;
        return false;
    }
    
    // 연결
    if (connect(clientSocket_, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        LOG_ERROR("Connection to %s:%d failed: %s", host.c_str(), port_, strerror(errno));
        ::close(clientSocket_);
        clientSocket_ = -1;
        return false;
    }
    
    connected_ = true;
    if (connectionCallback_) {
        connectionCallback_(true);
    }
    
    // 수신 스레드 시작
    running_ = true;
    receiveThread_ = std::thread(&SocketComm::receiveMessages, this, clientSocket_);
    
    LOG_INFO("Connected to %s:%d", host.c_str(), port_);
    return true;
}

bool SocketComm::sendMessage(const std::string& message) {
    if (!connected_) {
        LOG_ERROR("Not connected");
        return false;
    }
    
    int socket = (type_ == Type::SERVER) ? clientSocket_ : clientSocket_;
    if (socket < 0) {
        LOG_ERROR("Invalid socket");
        return false;
    }
    
    std::lock_guard<std::mutex> lock(sendMutex_);
    
    // 프로토콜: [4바이트 길이][메시지]
    uint32_t msgLen = message.length();
    uint32_t netLen = htonl(msgLen);
    
    // 길이 전송
    if (send(socket, &netLen, sizeof(netLen), MSG_NOSIGNAL) != sizeof(netLen)) {
        LOG_ERROR("Failed to send message length: %s", strerror(errno));
        return false;
    }
    
    // 메시지 전송
    size_t totalSent = 0;
    while (totalSent < message.length()) {
        ssize_t sent = send(socket, message.c_str() + totalSent, 
                           message.length() - totalSent, MSG_NOSIGNAL);
        if (sent < 0) {
            LOG_ERROR("Failed to send message: %s", strerror(errno));
            return false;
        }
        totalSent += sent;
    }
    
    LOG_DEBUG("Sent message (%zu bytes)", message.length());
    return true;
}

void SocketComm::setMessageCallback(MessageCallback callback) {
    messageCallback_ = callback;
}

void SocketComm::setConnectionCallback(ConnectionCallback callback) {
    connectionCallback_ = callback;
}

void SocketComm::close() {
    running_ = false;
    connected_ = false;
    
    // 소켓 닫기
    if (listenSocket_ >= 0) {
        ::close(listenSocket_);
        listenSocket_ = -1;
    }
    
    if (clientSocket_ >= 0) {
        ::close(clientSocket_);
        clientSocket_ = -1;
    }
    
    // 스레드 종료 대기
    if (serverThread_.joinable()) {
        serverThread_.join();
    }
    
    if (receiveThread_.joinable()) {
        receiveThread_.join();
    }
}

bool SocketComm::isConnected() const {
    return connected_;
}

void SocketComm::serverThread() {
    LOG_DEBUG("Server thread started");
    
    while (running_) {
        struct sockaddr_in clientAddr;
        socklen_t clientLen = sizeof(clientAddr);
        
        // 클라이언트 연결 대기
        int newClientSocket = accept(listenSocket_, 
                                    (struct sockaddr*)&clientAddr, &clientLen);
        if (newClientSocket < 0) {
            if (running_ && errno != EINTR) {
                LOG_ERROR("Accept failed: %s", strerror(errno));
            }
            continue;
        }
        
        LOG_INFO("Client connected from %s:%d", 
                 inet_ntoa(clientAddr.sin_addr), ntohs(clientAddr.sin_port));
        
        // 기존 클라이언트 연결이 있으면 닫기
        if (clientSocket_ >= 0) {
            LOG_INFO("Closing previous client connection");
            connected_ = false;
            ::close(clientSocket_);
        }
        
        // 새 클라이언트 설정
        clientSocket_ = newClientSocket;
        connected_ = true;
        
        if (connectionCallback_) {
            connectionCallback_(true);
        }
        
        // 별도 스레드에서 클라이언트 처리
        std::thread clientThread(&SocketComm::clientHandler, this, newClientSocket);
        clientThread.detach();
    }
    
    LOG_DEBUG("Server thread ended");
}

void SocketComm::clientHandler(int socket) {
    receiveMessages(socket);
    
    // 연결 종료 처리
    if (socket == clientSocket_) {
        connected_ = false;
        clientSocket_ = -1;
        
        if (connectionCallback_) {
            connectionCallback_(false);
        }
    }
}

void SocketComm::receiveMessages(int socket) {
    char buffer[4096];
    
    while (running_ && connected_) {
        // 메시지 길이 읽기
        uint32_t msgLen;
        ssize_t ret = recv(socket, &msgLen, sizeof(msgLen), MSG_WAITALL);
        if (ret != sizeof(msgLen)) {
            if (ret == 0) {
                LOG_INFO("Connection closed by peer");
            } else if (ret < 0) {
                LOG_ERROR("Failed to receive message length: %s", strerror(errno));
            }
            break;
        }
        
        msgLen = ntohl(msgLen);
        if (msgLen == 0 || msgLen > sizeof(buffer) - 1) {
            LOG_ERROR("Invalid message length: %u", msgLen);
            break;
        }
        
        // 메시지 읽기
        ret = recv(socket, buffer, msgLen, MSG_WAITALL);
        if (ret != static_cast<ssize_t>(msgLen)) {
            LOG_ERROR("Failed to receive complete message (got %zd, expected %u)", 
                     ret, msgLen);
            break;
        }
        
        buffer[msgLen] = '\0';
        
        // 콜백 호출
        if (messageCallback_) {
            messageCallback_(std::string(buffer, msgLen));
        }
    }
    
    LOG_DEBUG("Receive loop ended");
}