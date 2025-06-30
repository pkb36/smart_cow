#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include <string>
#include <thread>
#include <atomic>
#include <functional>
#include <termios.h>

class SerialComm {
public:
    using DataCallback = std::function<void(const uint8_t*, size_t)>;
    
    SerialComm(const std::string& device, int baudrate);
    ~SerialComm();
    
    bool open();
    void close();
    bool isOpen() const;
    
    // 데이터 송수신
    bool write(const uint8_t* data, size_t length);
    int readWithTimeout(uint8_t* buffer, size_t maxLength, int timeoutMs);
    
    // 콜백 설정
    void setDataCallback(DataCallback callback);
    
    // 설정
    bool setBaudrate(int baudrate);
    bool setDataBits(int bits);
    bool setParity(char parity);
    bool setStopBits(int bits);
    
private:
    void readThread();
    
private:
    std::string device_;
    int baudrate_;
    int fd_;
    
    std::atomic<bool> isOpen_;
    std::atomic<bool> running_;
    std::thread readThread_;
    
    DataCallback callback_;
    struct termios oldTermios_;
};

#endif // SERIAL_COMM_H