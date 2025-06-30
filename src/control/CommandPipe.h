#ifndef COMMAND_PIPE_H
#define COMMAND_PIPE_H

#include <string>
#include <thread>
#include <atomic>
#include <functional>

class CommandPipe {
public:
    using CommandCallback = std::function<void(const std::string&)>;
    
    CommandPipe(const std::string& pipePath);
    ~CommandPipe();
    
    bool create();
    bool open();
    void close();
    bool isOpen() const;
    
    // 명령 콜백 등록
    void setCommandCallback(CommandCallback callback);
    
    // 파이프 쓰기 (다른 프로세스에서 사용)
    static bool sendCommand(const std::string& pipePath, const std::string& command);
    
private:
    void readThread();
    
private:
    std::string pipePath_;
    int pipeFd_;
    
    std::atomic<bool> isOpen_;
    std::atomic<bool> running_;
    std::thread readThread_;
    
    CommandCallback callback_;
};

#endif // COMMAND_PIPE_H