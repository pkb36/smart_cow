#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <memory>
#include <sstream>
#include <mutex>

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
    
    template<typename... Args>
    void log(LogLevel level, const std::string& file, int line, const std::string& format, Args... args);
    
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