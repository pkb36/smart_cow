#ifndef SOCKET_COMM_H
#define SOCKET_COMM_H

#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>

class SocketComm {
public:
    enum class Type {
        SERVER,
        CLIENT
    };
    
    using MessageCallback = std::function<void(const std::string&)>;
    using ConnectionCallback = std::function<void(bool connected)>;
    
    SocketComm(Type type, int port);
    ~SocketComm();
    
    // 서버 모드
    bool startServer();
    
    // 클라이언트 모드
    bool connectToServer(const std::string& host = "127.0.0.1");
    
    // 공통
    bool sendMessage(const std::string& message);
    void setMessageCallback(MessageCallback callback);
    void setConnectionCallback(ConnectionCallback callback);
    void close();
    bool isConnected() const;
    
private:
    void serverThread();
    void clientHandler(int clientSocket);
    void receiveMessages(int socket);
    
private:
    Type type_;
    int port_;
    
    // 서버용
    int listenSocket_;
    std::thread serverThread_;
    
    // 클라이언트용
    int clientSocket_;
    std::thread receiveThread_;
    
    // 공통
    std::atomic<bool> running_;
    std::atomic<bool> connected_;
    MessageCallback messageCallback_;
    ConnectionCallback connectionCallback_;
    std::mutex sendMutex_;
};

#endif // SOCKET_COMM_H