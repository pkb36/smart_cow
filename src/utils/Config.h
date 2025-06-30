#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <memory>
#include "../common/Types.h"

class Config {
public:
    Config();
    ~Config();

    bool load(const std::string& filename);
    const SystemConfig& getSystemConfig() const;
    
    // 개별 설정 접근자
    std::string getCameraId() const;
    int getDeviceCount() const;
    int getMaxStreamCount() const;
    int getStreamBasePort() const;
    int getApiPort() const;
    const CameraConfig& getCameraConfig(int index) const;
    
    // 추가 설정 접근자
    std::string getTtyDevice() const;
    int getTtyBaudrate() const;
    std::string getServerUrl() const;
    std::string getRecordPath() const;
    int getRecordDuration() const;
    int getEventBufferTime() const;
    std::string getCodecName() const;
    
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // CONFIG_H