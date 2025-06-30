// Logger.h
#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <memory>
#include <sstream>
#include <mutex>
#include <cstdarg>
#include <cstdio>

enum class LogLevel {
    TRACE = 0,
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    FATAL
};

class Logger {
public:
    static Logger& getInstance();
    
    void init(const std::string& logPath, LogLevel level = LogLevel::INFO);
    void setLogLevel(LogLevel level);
    
    void log(LogLevel level, const std::string& file, int line, const std::string& message);
    
    // 템플릿 함수 정의를 헤더에 포함
    template<typename... Args>
    void log(LogLevel level, const std::string& file, int line, const std::string& format, Args... args) {
        // 가변 인자 포맷팅
        char buffer[4096];
        snprintf(buffer, sizeof(buffer), format.c_str(), args...);
        log(level, file, line, std::string(buffer));
    }
    
    void flush();
    void close();
    
private:
    Logger();
    ~Logger();
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

// 매크로 정의
#define LOG_TRACE(msg, ...) Logger::getInstance().log(LogLevel::TRACE, __FILE__, __LINE__, msg, ##__VA_ARGS__)
#define LOG_DEBUG(msg, ...) Logger::getInstance().log(LogLevel::DEBUG, __FILE__, __LINE__, msg, ##__VA_ARGS__)
#define LOG_INFO(msg, ...) Logger::getInstance().log(LogLevel::INFO, __FILE__, __LINE__, msg, ##__VA_ARGS__)
#define LOG_WARN(msg, ...) Logger::getInstance().log(LogLevel::WARNING, __FILE__, __LINE__, msg, ##__VA_ARGS__)
#define LOG_ERROR(msg, ...) Logger::getInstance().log(LogLevel::ERROR, __FILE__, __LINE__, msg, ##__VA_ARGS__)
#define LOG_FATAL(msg, ...) Logger::getInstance().log(LogLevel::FATAL, __FILE__, __LINE__, msg, ##__VA_ARGS__)

#endif // LOGGER_H