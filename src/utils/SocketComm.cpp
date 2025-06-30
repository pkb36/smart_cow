#include "SocketComm.h"
#include "Logger.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <signal.h>
#include <unistd.h>

// 기존 C 함수들 (extern "C"로 래핑)
extern "C" {
    
SOCKETINFO* init_socket_comm_server(int port) {
    SOCKETINFO* info = new SOCKETINFO;
    memset(info, 0, sizeof(SOCKETINFO));
    info->port = port;
    
    // 서버 소켓 생성
    info->socket = socket(AF_INET, SOCK_STREAM, 0);
    if (info->socket < 0) {
        delete info;
        return nullptr;
    }
    
    // SO_REUSEADDR 설정
    int opt = 1;
    setsockopt(info->socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    // 바인드
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);
    
    if (bind(info->socket, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(info->socket);
        delete info;
        return nullptr;
    }
    
    // 리슨
    if (listen(info->socket, 1) < 0) {
        close(info->socket);
        delete info;
        return nullptr;
    }
    
    return info;
}

void close_socket_comm(SOCKETINFO* info) {
    if (info) {
        if (info->socket >= 0) {
            close(info->socket);
        }
        delete info;
    }
}

}  // extern "C"

// SocketComm 클래스 구현
SocketComm::SocketComm(Type type, int port)
    : type_(type)
    , port_(port)
    , socketInfo_(nullptr)
    , running_(false) {
}

SocketComm::~SocketComm() {
    Close();
}

bool SocketComm::startServer() {
    if (type_ != Type::SERVER) {
        LOG_ERROR("Cannot start server on client socket");
        return false;
    }
    
    socketInfo_ = init_socket_comm_server(port_);
    if (!socketInfo_) {
        LOG_ERROR("Failed to create server socket on port %d", port_);
        return false;
    }
    
    // 콜백 설정
    socketInfo_->call_fun = &SocketComm::cCallbackWrapper;
    socketInfo_->data = this;
    
    // 서버 스레드 시작
    running_ = true;
    thread_ = std::thread(&SocketComm::serverThread, this);
    
    LOG_INFO("Socket server started on port %d", port_);
    return true;
}

bool SocketComm::connectToServer(const std::string& host) {
    if (type_ != Type::CLIENT) {
        LOG_ERROR("Cannot connect on server socket");
        return false;
    }
    
    socketInfo_ = new SOCKETINFO;
    memset(socketInfo_, 0, sizeof(SOCKETINFO));
    socketInfo_->port = port_;
    
    // 클라이언트 소켓 생성
    socketInfo_->socket = socket(AF_INET, SOCK_STREAM, 0);
    if (socketInfo_->socket < 0) {
        LOG_ERROR("Failed to create client socket");
        delete socketInfo_;
        socketInfo_ = nullptr;
        return false;
    }
    
    // 서버 주소 설정
    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port_);
    
    if (inet_pton(AF_INET, host.c_str(), &serverAddr.sin_addr) <= 0) {
        LOG_ERROR("Invalid address: %s", host.c_str());
        close(socketInfo_->socket);
        delete socketInfo_;
        socketInfo_ = nullptr;
        return false;
    }
    
    // 연결
    if (connect(socketInfo_->socket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        LOG_ERROR("Connection to %s:%d failed", host.c_str(), port_);
        close(socketInfo_->socket);
        delete socketInfo_;
        socketInfo_ = nullptr;
        return false;
    }
    
    socketInfo_->connect = 1;
    socketInfo_->call_fun = &SocketComm::cCallbackWrapper;
    socketInfo_->data = this;
    
    // 수신 스레드 시작
    running_ = true;
    thread_ = std::thread(&SocketComm::receiveThread, this);
    
    LOG_INFO("Connected to %s:%d", host.c_str(), port_);
    return true;
}

bool SocketComm::sendMessage(const std::string& message) {
    if (!socketInfo_ || !socketInfo_->connect) {
        LOG_ERROR("Socket not connected");
        return false;
    }
    
    // 메시지 길이 + 메시지 전송 (프로토콜에 따라)
    uint32_t msgLen = message.length();
    
    // 길이 전송 (네트워크 바이트 순서)
    uint32_t netLen = htonl(msgLen);
    if (send(socketInfo_->socket, &netLen, sizeof(netLen), 0) != sizeof(netLen)) {
        LOG_ERROR("Failed to send message length");
        return false;
    }
    
    // 메시지 전송
    size_t sent = 0;
    while (sent < message.length()) {
        ssize_t ret = send(socketInfo_->socket, message.c_str() + sent, 
                          message.length() - sent, 0);
        if (ret < 0) {
            LOG_ERROR("Failed to send message: %s", strerror(errno));
            return false;
        }
        sent += ret;
    }
    
    LOG_DEBUG("Sent message (%zu bytes): %s", message.length(), message.c_str());
    return true;
}

void SocketComm::setMessageCallback(MessageCallback callback) {
    callback_ = callback;
}

void SocketComm::Close() {
    running_ = false;
    
    if (thread_.joinable()) {
        thread_.join();
    }
    
    if (socketInfo_) {
        close_socket_comm(socketInfo_);
        socketInfo_ = nullptr;
    }
}

bool SocketComm::isConnected() const {
    return socketInfo_ && socketInfo_->connect;
}

void SocketComm::serverThread() {
    while (running_) {
        struct sockaddr_in clientAddr;
        socklen_t clientLen = sizeof(clientAddr);
        
        // Accept 연결
        int clientSocket = accept(socketInfo_->socket, 
                                 (struct sockaddr*)&clientAddr, &clientLen);
        if (clientSocket < 0) {
            if (running_) {
                LOG_ERROR("Accept failed: %s", strerror(errno));
            }
            continue;
        }
        
        LOG_INFO("Client connected from %s:%d", 
                 inet_ntoa(clientAddr.sin_addr), ntohs(clientAddr.sin_port));
        
        // 기존 연결 종료
        if (socketInfo_->connect) {
            close(socketInfo_->socket);
        }
        
        // 새 연결 설정
        socketInfo_->socket = clientSocket;
        socketInfo_->connect = 1;
        
        // 수신 처리
        receiveThread();
        
        // 연결 종료
        socketInfo_->connect = 0;
        close(clientSocket);
    }
}

void SocketComm::receiveThread() {
    char buffer[4096];
    
    while (running_ && socketInfo_->connect) {
        // 메시지 길이 읽기
        uint32_t msgLen;
        ssize_t ret = recv(socketInfo_->socket, &msgLen, sizeof(msgLen), MSG_WAITALL);
        if (ret != sizeof(msgLen)) {
            if (ret == 0) {
                LOG_INFO("Connection closed by peer");
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
        ret = recv(socketInfo_->socket, buffer, msgLen, MSG_WAITALL);
        if (ret != static_cast<ssize_t>(msgLen)) {
            LOG_ERROR("Failed to receive complete message");
            break;
        }
        
        buffer[msgLen] = '\0';
        
        // 콜백 호출
        if (socketInfo_->call_fun) {
            socketInfo_->call_fun(buffer, msgLen, socketInfo_->data);
        }
    }
    
    socketInfo_->connect = 0;
}

void SocketComm::cCallbackWrapper(char* data, int len, void* arg) {
    SocketComm* self = static_cast<SocketComm*>(arg);
    if (self && self->callback_) {
        self->callback_(std::string(data, len));
    }
}