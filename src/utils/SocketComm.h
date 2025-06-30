#ifndef SOCKET_COMM_H
#define SOCKET_COMM_H

#include <string>
#include <functional>
#include <thread>
#include <atomic>

// 기존 C 코드의 SOCKETINFO 구조체를 래핑
typedef struct _SOCKETINFO {
    int socket;
    int port;
    int connect;
    void (*call_fun)(char* data, int len, void* arg);
    void* data;
} SOCKETINFO;

class SocketComm {
public:
    enum class Type {
        SERVER,
        CLIENT
    };
    
    using MessageCallback = std::function<void(const std::string&)>;
    
    SocketComm(Type type, int port);
    ~SocketComm();
    
    // 서버 모드
    bool startServer();
    
    // 클라이언트 모드
    bool connectToServer(const std::string& host = "127.0.0.1");
    
    // 공통
    bool sendMessage(const std::string& message);
    void setMessageCallback(MessageCallback callback);
    void Close();
    bool isConnected() const;
    
private:
    void serverThread();
    void receiveThread();
    static void cCallbackWrapper(char* data, int len, void* arg);
    
private:
    Type type_;
    int port_;
    SOCKETINFO* socketInfo_;
    
    std::thread thread_;
    std::atomic<bool> running_;
    MessageCallback callback_;
};

#endif // SOCKET_COMM_H