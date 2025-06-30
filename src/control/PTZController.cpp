#include "PTZController.h"
#include "../utils/SerialComm.h"
#include "../utils/Logger.h"
#include <sstream>
#include <cstring>
#include <unistd.h>

PTZController::PTZController()
    : serial_(nullptr)
    , autoMoveRunning_(false)
    , autoMoveDelay_(0)
    , lastError_(PTZ_NORMAL)
    , moveSpeed_(0) {
    
    // 위치 배열 초기화
    memset(ptzPositions_, 0, sizeof(ptzPositions_));
    memset(ranchPositions_, 0, sizeof(ranchPositions_));
}

PTZController::~PTZController() {
    stopAutoMove();
    close();
}

bool PTZController::init(const std::string& serialDevice, int baudrate) {
    std::lock_guard<std::mutex> lock(controlMutex_);
    
    serial_ = std::make_unique<SerialComm>(serialDevice, baudrate);
    
    if (!serial_->open()) {
        LOG_ERROR("Failed to open PTZ serial port: %s", serialDevice.c_str());
        return false;
    }
    
    LOG_INFO("PTZ controller initialized on %s @ %d baud", 
             serialDevice.c_str(), baudrate);
    return true;
}

void PTZController::close() {
    stopAutoMove();
    
    std::lock_guard<std::mutex> lock(controlMutex_);
    if (serial_) {
        serial_->close();
        serial_.reset();
    }
}

bool PTZController::sendMoveCommand(Direction direction, int speed) {
    std::lock_guard<std::mutex> lock(controlMutex_);
    
    if (!serial_ || !serial_->isOpen()) {
        LOG_ERROR("PTZ serial not open");
        lastError_ = PTZ_STOP_FAILED;
        return false;
    }
    
    uint8_t data[11];
    memset(data, 0, sizeof(data));
    
    int len = 0;
    data[len++] = 0x96;  // Sync
    data[len++] = 0x00;  // Addr
    data[len++] = 0x00;  // CmdH
    
    if (speed > 0) {
        data[len++] = 0x41;  // CmdL: Does not require response
    } else {
        data[len++] = 0x01;  // CmdL: Require response
    }
    
    data[len++] = 0x05;  // Data length
    
    // 방향별 명령 설정
    if (speed > 0) {
        switch (direction) {
            case LEFT:
                data[len++] = 0x40;
                break;
            case RIGHT:
                data[len++] = 0x80;
                break;
            case UP:
                data[len++] = 0x10;
                data[len++] = 0x00;
                break;
            case DOWN:
                data[len++] = 0x20;
                data[len++] = 0x00;
                break;
            case ZOOM_IN:
                data[len++] = 0x04;
                data[len++] = 0x00;
                data[len++] = 0x00;
                break;
            case ZOOM_OUT:
                data[len++] = 0x08;
                data[len++] = 0x00;
                data[len++] = 0x00;
                break;
        }
        data[len] = speed;  // PTZ speed
    }
    
    // 체크섬 계산
    data[10] = calculateChecksum(data, 10);
    len = 11;
    
    // 명령 전송
    if (!serial_->write(data, len)) {
        LOG_ERROR("Failed to send PTZ move command");
        lastError_ = PTZ_STOP_FAILED;
        return false;
    }
    
    // 정지 명령인 경우 응답 확인
    if (speed == 0) {
        uint8_t response[7];
        int ret = serial_->readWithTimeout(response, sizeof(response), 1000);
        if (ret <= 0) {
            LOG_ERROR("PTZ stop command timeout");
            lastError_ = PTZ_STOP_FAILED;
            return false;
        }
        
        // 응답 확인
        if (ret >= 6 && response[4] == 0x01 && response[5] == 0x00) {
            lastError_ = PTZ_NORMAL;
        } else {
            lastError_ = PTZ_STOP_FAILED;
            LOG_ERROR("PTZ stop command error: 0x%02X", response[5]);
            return false;
        }
    }
    
    moveSpeed_ = speed;
    LOG_DEBUG("PTZ move command sent: direction=%d, speed=%d", direction, speed);
    return true;
}

bool PTZController::sendStopCommand() {
    return sendMoveCommand(LEFT, 0);  // 방향은 무관
}

bool PTZController::moveAndStop(Direction direction, int speed, int delayMs) {
    if (!sendMoveCommand(direction, speed)) {
        return false;
    }
    
    // 별도 스레드에서 정지 명령 예약
    std::thread([this, delayMs]() {
        usleep(delayMs * 1000);
        sendStopCommand();
    }).detach();
    
    return true;
}

bool PTZController::setPTZPosition(int index, bool isAutoMode) {
    if (index < 0 || index >= MAX_PTZ_PRESET) {
        LOG_ERROR("Invalid PTZ position index: %d", index);
        return false;
    }
    
    std::lock_guard<std::mutex> lock(controlMutex_);
    
    if (!serial_ || !serial_->isOpen()) {
        LOG_ERROR("PTZ serial not open");
        return false;
    }
    
    // 현재 위치 읽기 명령
    uint8_t cmd[7] = {0x96, 0x00, 0x06, 0x01, 0x01, 0x01, 0x9F};
    if (!serial_->write(cmd, sizeof(cmd))) {
        LOG_ERROR("Failed to send get position command");
        return false;
    }
    
    // 응답 읽기 (17바이트)
    uint8_t response[17];
    int ret = serial_->readWithTimeout(response, sizeof(response), 1000);
    if (ret < 17) {
        LOG_ERROR("Failed to read position response");
        return false;
    }
    
    // 위치 저장
    return updatePTZPosition(index, &response[5], isAutoMode);
}

bool PTZController::moveToPTZPosition(int index, bool isAutoMode) {
    if (index < 0 || index >= MAX_PTZ_PRESET) {
        LOG_ERROR("Invalid PTZ position index: %d", index);
        return false;
    }
    
    PTZPosition& pos = isAutoMode ? ptzPositions_[index] : ptzPositions_[index];
    if (!pos.isSet) {
        LOG_ERROR("PTZ position %d not set", index);
        return false;
    }
    
    std::lock_guard<std::mutex> lock(controlMutex_);
    
    if (!serial_ || !serial_->isOpen()) {
        LOG_ERROR("PTZ serial not open");
        return false;
    }
    
    // 이동 명령 구성
    uint8_t cmd[32];
    memset(cmd, 0, sizeof(cmd));
    
    cmd[0] = 0x96;
    cmd[1] = 0x00;
    cmd[2] = 0x01;
    cmd[3] = 0x01;
    cmd[4] = 0x0F;  // 데이터 길이
    
    // 위치 데이터 복사
    memcpy(&cmd[5], pos.data, 10);
    
    // 속도 설정
    cmd[15] = isAutoMode ? 0x20 : 0x40;  // 자동 모드는 느리게
    
    // 체크섬
    cmd[16] = calculateChecksum(cmd, 16);
    
    // 명령 전송
    if (!serial_->write(cmd, 17)) {
        LOG_ERROR("Failed to send move to position command");
        return false;
    }
    
    // 응답 확인
    uint8_t response[7];
    int ret = serial_->readWithTimeout(response, sizeof(response), 1000);
    if (ret > 0 && response[5] == 0x00) {
        LOG_INFO("Moving to PTZ position %d", index);
        return true;
    }
    
    LOG_ERROR("Failed to move to PTZ position %d", index);
    return false;
}

bool PTZController::updatePTZPosition(int index, const uint8_t* posData, bool isAutoMode) {
    if (index < 0 || index >= MAX_PTZ_PRESET || !posData) {
        return false;
    }
    
    PTZPosition& pos = isAutoMode ? ptzPositions_[index] : ptzPositions_[index];
    pos.isSet = true;
    memcpy(pos.data, posData, PTZ_POS_SIZE - 1);  // 11 - 1 for isSet flag
    
    LOG_INFO("PTZ position %d updated", index);
    return true;
}

bool PTZController::setRanchPosition(int index) {
    if (index < 0 || index >= MAX_RANCH_POS) {
        LOG_ERROR("Invalid ranch position index: %d", index);
        return false;
    }
    
    // 자동 이동 중지
    stopAutoMove();
    
    // 일반 PTZ 위치 설정과 동일
    std::lock_guard<std::mutex> lock(controlMutex_);
    
    uint8_t cmd[7] = {0x96, 0x00, 0x06, 0x01, 0x01, 0x01, 0x9F};
    if (!serial_->write(cmd, sizeof(cmd))) {
        return false;
    }
    
    uint8_t response[17];
    int ret = serial_->readWithTimeout(response, sizeof(response), 1000);
    if (ret < 17) {
        return false;
    }
    
    // Ranch 위치 저장
    ranchPositions_[index].isSet = true;
    memcpy(ranchPositions_[index].data, &response[5], 10);
    
    LOG_INFO("Ranch position %d set", index);
    return true;
}

bool PTZController::moveToRanchPosition(int index) {
    if (index < 0 || index >= MAX_RANCH_POS) {
        LOG_ERROR("Invalid ranch position index: %d", index);
        return false;
    }
    
    if (!ranchPositions_[index].isSet) {
        LOG_ERROR("Ranch position %d not set", index);
        return false;
    }
    
    // 자동 이동 중지
    stopAutoMove();
    
    // PTZ 이동과 동일한 방식
    std::lock_guard<std::mutex> lock(controlMutex_);
    
    uint8_t cmd[32];
    memset(cmd, 0, sizeof(cmd));
    
    cmd[0] = 0x96;
    cmd[1] = 0x00;
    cmd[2] = 0x01;
    cmd[3] = 0x01;
    cmd[4] = 0x0F;
    
    memcpy(&cmd[5], ranchPositions_[index].data, 10);
    cmd[15] = 0x40;  // Ranch 속도
    cmd[16] = calculateChecksum(cmd, 16);
    
    return serial_->write(cmd, 17);
}

bool PTZController::startAutoMove(const std::string& sequence) {
    if (autoMoveRunning_) {
        LOG_WARN("Auto move already running");
        return false;
    }
    
    std::vector<int> positions;
    if (!parseAutoMoveSequence(sequence, positions)) {
        return false;
    }
    
    if (positions.size() < 2) {
        LOG_ERROR("Auto move sequence must have at least 2 positions");
        return false;
    }
    
    // 마지막 값은 지연 시간
    autoMoveDelay_ = positions.back();
    positions.pop_back();
    
    autoMoveSequence_ = positions;
    autoMoveRunning_ = true;
    
    // 자동 이동 스레드 시작
    autoMoveThread_ = std::thread(&PTZController::autoMoveThread, this);
    
    LOG_INFO("Auto move started with %zu positions, delay=%d", 
             positions.size(), autoMoveDelay_);
    return true;
}

void PTZController::stopAutoMove() {
    if (!autoMoveRunning_) {
        return;
    }
    
    autoMoveRunning_ = false;
    
    if (autoMoveThread_.joinable()) {
        autoMoveThread_.join();
    }
    
    LOG_INFO("Auto move stopped");
}

bool PTZController::isAutoMoveRunning() const {
    return autoMoveRunning_;
}

void PTZController::sendPipeCommand(const std::string& command) {
    if (command == "up") {
        uint8_t cmd[] = {0x96, 0x0, 0x14, 0x1, 0x6, 0x81, 0x1, 0x4, 0x16, 0x1, 0xFF, 0x4D};
        serial_->write(cmd, sizeof(cmd));
    } else if (command == "down") {
        uint8_t cmd[] = {0x96, 0x0, 0x14, 0x1, 0x6, 0x81, 0x1, 0x4, 0x16, 0x2, 0xFF, 0x4E};
        serial_->write(cmd, sizeof(cmd));
    } else if (command == "left") {
        uint8_t cmd[] = {0x96, 0x0, 0x14, 0x1, 0x6, 0x81, 0x1, 0x4, 0x16, 0x4, 0xFF, 0x50};
        serial_->write(cmd, sizeof(cmd));
    } else if (command == "right") {
        uint8_t cmd[] = {0x96, 0x0, 0x14, 0x1, 0x6, 0x81, 0x1, 0x4, 0x16, 0x8, 0xFF, 0x54};
        serial_->write(cmd, sizeof(cmd));
    } else if (command == "enter") {
        uint8_t cmd[] = {0x96, 0x0, 0x14, 0x1, 0x6, 0x81, 0x1, 0x4, 0x16, 0x10, 0xFF, 0x5C};
        serial_->write(cmd, sizeof(cmd));
    } else if (command == "zoom_init") {
        uint8_t cmd[] = {0x96, 0x0, 0x14, 0x1, 0x6, 0x81, 0x1, 0x4, 0x19, 0x1, 0xFF, 0x50};
        serial_->write(cmd, sizeof(cmd));
        LOG_DEBUG("Zoom init command sent");
    } else if (command == "ir_init") {
        uint8_t cmd1[] = {
            0x96, 0x00, 0x22, 0x05, 0x15, 0x01, 0x01, 0x01, 0x20, 0x30,
            0x40, 0x60, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F,
            0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0xB7
        };
        serial_->write(cmd1, sizeof(cmd1));
        
        usleep(1500000);  // 1.5초 대기
        
        uint8_t cmd2[] = {
            0x96, 0x00, 0x22, 0x05, 0x15, 0x00, 0x7F, 0x7F, 0x7F, 0x7F,
            0x7F, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
            0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x5C
        };
        serial_->write(cmd2, sizeof(cmd2));
        
        LOG_DEBUG("IR init command sent");
    } else {
        LOG_ERROR("Unknown pipe command: %s", command.c_str());
    }
}

void PTZController::autoMoveThread() {
    size_t currentPos = 0;
    
    while (autoMoveRunning_ && !autoMoveSequence_.empty()) {
        int position = autoMoveSequence_[currentPos];
        
        LOG_DEBUG("Auto move to position %d", position);
        
        if (!moveToPTZPosition(position, true)) {
            LOG_ERROR("Failed to move to position %d in auto sequence", position);
        }
        
        // 지연
        for (int i = 0; i < autoMoveDelay_ && autoMoveRunning_; i++) {
            sleep(1);
        }
        
        // 다음 위치
        currentPos = (currentPos + 1) % autoMoveSequence_.size();
    }
}

bool PTZController::parseAutoMoveSequence(const std::string& sequence, 
                                         std::vector<int>& positions) {
    positions.clear();
    
    std::istringstream iss(sequence);
    std::string token;
    
    while (std::getline(iss, token, ',')) {
        try {
            int value = std::stoi(token);
            positions.push_back(value);
        } catch (const std::exception& e) {
            LOG_ERROR("Invalid position value in sequence: %s", token.c_str());
            return false;
        }
    }
    
    return !positions.empty();
}

uint8_t PTZController::calculateChecksum(const uint8_t* data, size_t length) {
    uint8_t sum = 0;
    for (size_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum;
}