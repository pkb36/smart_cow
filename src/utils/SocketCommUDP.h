#ifndef SOCKET_COMM_UDP_H
#define SOCKET_COMM_UDP_H

#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <netinet/in.h>

class SocketCommUDP {
public:
    enum class Type {
        SERVER,
        CLIENT
    };
    
    using MessageCallback = std::function<void(const std::string&, const struct sockaddr_in&)>;
    
    SocketCommUDP(Type type, int port);
    ~SocketCommUDP();
    
    // 서버 모드
    bool startServer();
    
    // 클라이언트 모드
    bool connectToServer(const std::string& host = "127.0.0.1");
    
    // 메시지 전송
    bool sendMessage(const std::string& message);
    bool sendMessageTo(const std::string& message, const struct sockaddr_in& addr);
    
    void setMessageCallback(MessageCallback callback);
    void close();
    bool isConnected() const;
private:
    void receiveThread();
    
private:
    Type type_;
    int port_;
    int socket_;
    
    struct sockaddr_in serverAddr_;
    struct sockaddr_in clientAddr_;
    bool hasClient_;
    
    std::thread receiveThread_;
    std::atomic<bool> running_;
    MessageCallback messageCallback_;
    std::mutex mutex_;
};

#endif