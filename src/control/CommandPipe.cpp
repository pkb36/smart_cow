#include "CommandPipe.h"
#include "../utils/Logger.h"
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>

CommandPipe::CommandPipe(const std::string& pipePath)
    : pipePath_(pipePath)
    , pipeFd_(-1)
    , isOpen_(false)
    , running_(false) {
}

CommandPipe::~CommandPipe() {
    close();
}

bool CommandPipe::create() {
    // 기존 파이프 제거
    unlink(pipePath_.c_str());
    
    // 명명된 파이프(FIFO) 생성
    if (mkfifo(pipePath_.c_str(), 0666) < 0) {
        if (errno != EEXIST) {
            LOG_ERROR("Failed to create pipe %s: %s", pipePath_.c_str(), strerror(errno));
            return false;
        }
    }
    
    LOG_INFO("Command pipe created: %s", pipePath_.c_str());
    return true;
}

bool CommandPipe::open() {
    if (isOpen_) {
        return true;
    }
    
    // 파이프가 없으면 생성
    struct stat st;
    if (stat(pipePath_.c_str(), &st) < 0) {
        if (!create()) {
            return false;
        }
    }
    
    // 논블로킹 모드로 열기 (읽기 전용)
    pipeFd_ = ::open(pipePath_.c_str(), O_RDONLY | O_NONBLOCK);
    if (pipeFd_ < 0) {
        LOG_ERROR("Failed to open pipe %s: %s", pipePath_.c_str(), strerror(errno));
        return false;
    }
    
    isOpen_ = true;
    
    // 읽기 스레드 시작
    running_ = true;
    readThread_ = std::thread(&CommandPipe::readThread, this);
    
    LOG_INFO("Command pipe opened: %s", pipePath_.c_str());
    return true;
}

void CommandPipe::close() {
    running_ = false;
    
    if (readThread_.joinable()) {
        // 파이프에 더미 데이터 쓰기로 스레드 깨우기
        sendCommand(pipePath_, "\n");
        readThread_.join();
    }
    
    if (pipeFd_ >= 0) {
        ::close(pipeFd_);
        pipeFd_ = -1;
    }
    
    isOpen_ = false;
    LOG_INFO("Command pipe closed");
}

bool CommandPipe::isOpen() const {
    return isOpen_;
}

void CommandPipe::setCommandCallback(CommandCallback callback) {
    callback_ = callback;
}

bool CommandPipe::sendCommand(const std::string& pipePath, const std::string& command) {
    // 파이프 열기 (쓰기 전용)
    int fd = ::open(pipePath.c_str(), O_WRONLY | O_NONBLOCK);
    if (fd < 0) {
        // 논블로킹이므로 ENXIO는 읽는 쪽이 없다는 의미
        if (errno == ENXIO) {
            LOG_DEBUG("No reader on pipe %s", pipePath.c_str());
        } else {
            LOG_ERROR("Failed to open pipe %s for writing: %s", 
                      pipePath.c_str(), strerror(errno));
        }
        return false;
    }
    
    // 명령 쓰기 (개행 문자 추가)
    std::string cmdWithNewline = command;
    if (cmdWithNewline.empty() || cmdWithNewline.back() != '\n') {
        cmdWithNewline += '\n';
    }
    
    ssize_t written = write(fd, cmdWithNewline.c_str(), cmdWithNewline.length());
    ::close(fd);
    
    if (written < 0) {
        LOG_ERROR("Failed to write to pipe: %s", strerror(errno));
        return false;
    }
    
    // LOG_DEBUG("Command sent to pipe: %s", command.c_str());
    return true;
}

void CommandPipe::readThread() {
    char buffer[1024];
    std::string lineBuffer;
    
    // 블로킹 모드로 다시 열기
    ::close(pipeFd_);
    pipeFd_ = ::open(pipePath_.c_str(), O_RDONLY);
    if (pipeFd_ < 0) {
        LOG_ERROR("Failed to reopen pipe in blocking mode: %s", strerror(errno));
        return;
    }
    
    while (running_) {
        // 파이프에서 읽기
        ssize_t bytesRead = read(pipeFd_, buffer, sizeof(buffer) - 1);
        
        if (bytesRead > 0) {
            buffer[bytesRead] = '\0';
            lineBuffer += buffer;
            
            // 개행 문자로 분리하여 명령 처리
            size_t pos;
            while ((pos = lineBuffer.find('\n')) != std::string::npos) {
                std::string command = lineBuffer.substr(0, pos);
                lineBuffer.erase(0, pos + 1);
                
                // 공백 제거
                command.erase(0, command.find_first_not_of(" \t\r"));
                command.erase(command.find_last_not_of(" \t\r") + 1);
                
                if (!command.empty() && callback_) {
                    LOG_INFO("Command received: %s", command.c_str());
                    callback_(command);
                }
            }
        } else if (bytesRead == 0) {
            // 파이프의 쓰기 쪽이 닫힘 - 다시 열기
            ::close(pipeFd_);
            pipeFd_ = ::open(pipePath_.c_str(), O_RDONLY);
            if (pipeFd_ < 0) {
                LOG_ERROR("Failed to reopen pipe: %s", strerror(errno));
                break;
            }
        } else {
            if (errno != EINTR && errno != EAGAIN) {
                LOG_ERROR("Pipe read error: %s", strerror(errno));
                break;
            }
        }
    }
}