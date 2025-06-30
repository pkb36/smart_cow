#include "SerialComm.h"
#include "Logger.h"
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/select.h>

SerialComm::SerialComm(const std::string& device, int baudrate)
    : device_(device)
    , baudrate_(baudrate)
    , fd_(-1)
    , isOpen_(false)
    , running_(false) {
}

SerialComm::~SerialComm() {
    close();
}

bool SerialComm::open() {
    if (isOpen_) {
        return true;
    }
    
    // 시리얼 포트 열기
    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ == -1) {
        LOG_ERROR("Failed to open serial port %s: %s", device_.c_str(), strerror(errno));
        return false;
    }
    
    // 현재 설정 백업
    if (tcgetattr(fd_, &oldTermios_) < 0) {
        LOG_ERROR("Failed to get serial attributes: %s", strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    
    // 새 설정 구성
    struct termios newTermios;
    memset(&newTermios, 0, sizeof(newTermios));
    
    // 기본 설정
    newTermios.c_cflag = CS8 | CLOCAL | CREAD;
    newTermios.c_iflag = IGNPAR;
    newTermios.c_oflag = 0;
    newTermios.c_lflag = 0;
    
    // Baudrate 설정
    speed_t speed;
    switch (baudrate_) {
        case 9600:   speed = B9600; break;
        case 19200:  speed = B19200; break;
        case 38400:  speed = B38400; break;
        case 57600:  speed = B57600; break;
        case 115200: speed = B115200; break;
        default:
            LOG_ERROR("Unsupported baudrate: %d", baudrate_);
            ::close(fd_);
            fd_ = -1;
            return false;
    }
    
    cfsetispeed(&newTermios, speed);
    cfsetospeed(&newTermios, speed);
    
    // 타임아웃 설정
    newTermios.c_cc[VTIME] = 0;
    newTermios.c_cc[VMIN] = 1;
    
    // 설정 적용
    tcflush(fd_, TCIFLUSH);
    if (tcsetattr(fd_, TCSANOW, &newTermios) < 0) {
        LOG_ERROR("Failed to set serial attributes: %s", strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    
    isOpen_ = true;
    
    // 읽기 스레드 시작
    if (callback_) {
        running_ = true;
        readThread_ = std::thread(&SerialComm::readThread, this);
    }
    
    LOG_INFO("Serial port %s opened successfully (baudrate: %d)", 
             device_.c_str(), baudrate_);
    return true;
}

void SerialComm::close() {
    if (!isOpen_) {
        return;
    }
    
    running_ = false;
    
    // 읽기 스레드 종료 대기
    if (readThread_.joinable()) {
        readThread_.join();
    }
    
    // 원래 설정 복원
    tcsetattr(fd_, TCSANOW, &oldTermios_);
    
    ::close(fd_);
    fd_ = -1;
    isOpen_ = false;
    
    LOG_INFO("Serial port %s closed", device_.c_str());
}

bool SerialComm::isOpen() const {
    return isOpen_;
}

bool SerialComm::write(const uint8_t* data, size_t length) {
    if (!isOpen_) {
        LOG_ERROR("Serial port not open");
        return false;
    }
    
    size_t written = 0;
    while (written < length) {
        ssize_t ret = ::write(fd_, data + written, length - written);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            LOG_ERROR("Serial write failed: %s", strerror(errno));
            return false;
        }
        written += ret;
    }
    
    // 디버그 로그
    if (length < 64) {  // 작은 데이터만 로그
        std::string hexStr;
        for (size_t i = 0; i < length; i++) {
            char buf[4];
            snprintf(buf, sizeof(buf), "%02X ", data[i]);
            hexStr += buf;
        }
        LOG_DEBUG("Serial TX: %s", hexStr.c_str());
    }
    
    return true;
}

int SerialComm::readWithTimeout(uint8_t* buffer, size_t maxLength, int timeoutMs) {
    if (!isOpen_) {
        LOG_ERROR("Serial port not open");
        return -1;
    }
    
    fd_set readSet;
    struct timeval timeout;
    
    FD_ZERO(&readSet);
    FD_SET(fd_, &readSet);
    
    timeout.tv_sec = timeoutMs / 1000;
    timeout.tv_usec = (timeoutMs % 1000) * 1000;
    
    int ret = select(fd_ + 1, &readSet, nullptr, nullptr, &timeout);
    if (ret < 0) {
        LOG_ERROR("Select failed: %s", strerror(errno));
        return -1;
    } else if (ret == 0) {
        // 타임아웃
        return 0;
    }
    
    // 데이터 읽기
    ssize_t bytesRead = ::read(fd_, buffer, maxLength);
    if (bytesRead < 0) {
        LOG_ERROR("Serial read failed: %s", strerror(errno));
        return -1;
    }
    
    // 디버그 로그
    if (bytesRead > 0 && bytesRead < 64) {
        std::string hexStr;
        for (ssize_t i = 0; i < bytesRead; i++) {
            char buf[4];
            snprintf(buf, sizeof(buf), "%02X ", buffer[i]);
            hexStr += buf;
        }
        LOG_DEBUG("Serial RX: %s", hexStr.c_str());
    }
    
    return bytesRead;
}

void SerialComm::setDataCallback(DataCallback callback) {
    callback_ = callback;
    
    // 이미 열려있고 스레드가 없으면 시작
    if (isOpen_ && callback_ && !running_) {
        running_ = true;
        readThread_ = std::thread(&SerialComm::readThread, this);
    }
}

bool SerialComm::setBaudrate(int baudrate) {
    if (isOpen_) {
        LOG_ERROR("Cannot change baudrate while port is open");
        return false;
    }
    baudrate_ = baudrate;
    return true;
}

void SerialComm::readThread() {
    uint8_t buffer[1024];
    
    while (running_) {
        int bytesRead = readWithTimeout(buffer, sizeof(buffer), 100);
        if (bytesRead > 0 && callback_) {
            callback_(buffer, bytesRead);
        }
    }
}