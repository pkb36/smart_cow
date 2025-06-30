#ifndef PTZ_CONTROLLER_H
#define PTZ_CONTROLLER_H

#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>

class SerialComm;

class PTZController {
public:
    enum Direction {
        LEFT = 0,
        RIGHT = 1,
        UP = 2,
        DOWN = 3,
        ZOOM_IN = 4,
        ZOOM_OUT = 5
    };
    
    enum ErrorCode {
        PTZ_NORMAL = 0,
        PTZ_STOP_FAILED = 1
    };
    
    static constexpr int MAX_PTZ_PRESET = 12;
    static constexpr int MAX_RANCH_POS = 32;
    static constexpr int PTZ_POS_SIZE = 11;
    
    struct PTZPosition {
        bool isSet;
        uint8_t data[PTZ_POS_SIZE];
    };
    
    PTZController();
    ~PTZController();
    
    bool init(const std::string& serialDevice, int baudrate);
    void close();
    
    // 기본 PTZ 제어
    bool sendMoveCommand(Direction direction, int speed);
    bool sendStopCommand();
    bool moveAndStop(Direction direction, int speed, int delayMs);
    
    // 프리셋 위치
    bool setPTZPosition(int index, bool isAutoMode = false);
    bool moveToPTZPosition(int index, bool isAutoMode = false);
    bool updatePTZPosition(int index, const uint8_t* posData, bool isAutoMode);
    
    // Ranch 모드 위치
    bool setRanchPosition(int index);
    bool moveToRanchPosition(int index);
    
    // 자동 이동
    bool startAutoMove(const std::string& sequence);
    void stopAutoMove();
    bool isAutoMoveRunning() const;
    
    // 상태
    ErrorCode getLastError() const { return lastError_; }
    int getMoveSpeed() const { return moveSpeed_; }
    
    // 특수 명령
    void sendPipeCommand(const std::string& command);
    
private:
    void autoMoveThread();
    bool parseAutoMoveSequence(const std::string& sequence, std::vector<int>& positions);
    uint8_t calculateChecksum(const uint8_t* data, size_t length);
    
private:
    std::unique_ptr<SerialComm> serial_;
    
    // 프리셋 위치 저장
    PTZPosition ptzPositions_[MAX_PTZ_PRESET];
    PTZPosition ranchPositions_[MAX_RANCH_POS];
    
    // 자동 이동
    std::thread autoMoveThread_;
    std::atomic<bool> autoMoveRunning_;
    std::vector<int> autoMoveSequence_;
    int autoMoveDelay_;
    
    // 상태
    std::atomic<ErrorCode> lastError_;
    std::atomic<int> moveSpeed_;
    std::mutex controlMutex_;
};

#endif // PTZ_CONTROLLER_H