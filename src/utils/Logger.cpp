#include "Logger.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <cstdarg>
#include <sys/stat.h>
#include <sys/types.h>

class Logger::Impl {
public:
    Impl() : logLevel_(LogLevel::INFO), isInitialized_(false) {}
    
    ~Impl() {
        if (fileStream_.is_open()) {
            fileStream_.close();
        }
    }
    
    bool init(const std::string& logPath, LogLevel level) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        logLevel_ = level;
        logPath_ = logPath;
        
        // 로그 디렉토리 생성
        size_t pos = logPath.find_last_of("/\\");
        if (pos != std::string::npos) {
            std::string dir = logPath.substr(0, pos);
            mkdir(dir.c_str(), 0755);
        }
        
        // 파일 열기
        fileStream_.open(logPath, std::ios::app);
        if (!fileStream_.is_open()) {
            std::cerr << "Failed to open log file: " << logPath << std::endl;
            return false;
        }
        
        isInitialized_ = true;
        return true;
    }
    
    void log(LogLevel level, const std::string& file, int line, const std::string& message) {
        if (!isInitialized_ || level < logLevel_) {
            return;
        }
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        // 타임스탬프 생성
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        struct tm timeinfo;
        localtime_r(&time_t, &timeinfo);
        
        // 로그 포맷: [YYYY-MM-DD HH:MM:SS.mmm] [LEVEL] [file:line] message
        std::ostringstream oss;
        oss << "[" << std::put_time(&timeinfo, "%Y-%m-%d %H:%M:%S")
            << "." << std::setfill('0') << std::setw(3) << ms.count() << "] "
            << "[" << levelToString(level) << "] "
            << "[" << getFileName(file) << ":" << line << "] "
            << message;
        
        std::string logLine = oss.str();
        
        // 파일과 콘솔에 출력
        if (fileStream_.is_open()) {
            fileStream_ << logLine << std::endl;
            fileStream_.flush();
        }
        
        // 콘솔 출력 (ERROR 이상만)
        if (level >= LogLevel::ERROR) {
            std::cerr << logLine << std::endl;
        } else if (level >= LogLevel::INFO) {
            std::cout << logLine << std::endl;
        }
    }
    
    void setLogLevel(LogLevel level) {
        std::lock_guard<std::mutex> lock(mutex_);
        logLevel_ = level;
    }
    
    void flush() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (fileStream_.is_open()) {
            fileStream_.flush();
        }
    }
    
    void close() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (fileStream_.is_open()) {
            fileStream_.close();
        }
        isInitialized_ = false;
    }
    
private:
    std::string levelToString(LogLevel level) {
        switch (level) {
            case LogLevel::TRACE: return "TRACE";
            case LogLevel::DEBUG: return "DEBUG";
            case LogLevel::INFO:  return "INFO ";
            case LogLevel::WARNING: return "WARN ";
            case LogLevel::ERROR: return "ERROR";
            case LogLevel::FATAL: return "FATAL";
            default: return "UNKNOWN";
        }
    }
    
    std::string getFileName(const std::string& path) {
        size_t pos = path.find_last_of("/\\");
        return (pos != std::string::npos) ? path.substr(pos + 1) : path;
    }
    
private:
    std::mutex mutex_;
    std::ofstream fileStream_;
    std::string logPath_;
    LogLevel logLevel_;
    bool isInitialized_;
};

// Logger 구현
Logger::Logger() : pImpl(std::make_unique<Impl>()) {}
Logger::~Logger() = default;

Logger& Logger::getInstance() {
    static Logger instance;
    return instance;
}

void Logger::init(const std::string& logPath, LogLevel level) {
    // 날짜별 로그 파일 생성
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    struct tm timeinfo;
    localtime_r(&time_t, &timeinfo);
    
    std::ostringstream oss;
    oss << logPath << "/"
        << std::put_time(&timeinfo, "%Y-%m-%d")
        << ".log";
    
    pImpl->init(oss.str(), level);
}

void Logger::setLogLevel(LogLevel level) {
    pImpl->setLogLevel(level);
}

void Logger::log(LogLevel level, const std::string& file, int line, const std::string& message) {
    pImpl->log(level, file, line, message);
}

template<typename... Args>
void Logger::log(LogLevel level, const std::string& file, int line, 
                const std::string& format, Args... args) {
    // 가변 인자 포맷팅
    char buffer[4096];
    snprintf(buffer, sizeof(buffer), format.c_str(), args...);
    pImpl->log(level, file, line, std::string(buffer));
}

void Logger::flush() {
    pImpl->flush();
}

void Logger::close() {
    pImpl->close();
}

// 템플릿 인스턴스화 (필요한 타입들)
template void Logger::log(LogLevel, const std::string&, int, const std::string&, int);
template void Logger::log(LogLevel, const std::string&, int, const std::string&, const char*);
template void Logger::log(LogLevel, const std::string&, int, const std::string&, double);
template void Logger::log(LogLevel, const std::string&, int, const std::string&, int, int);
template void Logger::log(LogLevel, const std::string&, int, const std::string&, const char*, int);