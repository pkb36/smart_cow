#ifndef DEVICE_SETTING_H
#define DEVICE_SETTING_H

#include <string>
#include <memory>
#include <atomic>

class DeviceSetting {
public:
    struct Settings {
        // 녹화 설정
        int recordStatus;
        
        // 분석 설정
        int analysisStatus;
        int nvInterval;
        
        // 탐지 설정
        int optFlowApply;
        int resnet50Apply;
        int enableEventNotify;
        
        // 온도 설정
        int tempCorrection;
        
        // 모드 설정
        std::string ptzStatus;
        int colorPalette;
        
        Settings() : 
            recordStatus(0),
            analysisStatus(0), nvInterval(0),
            optFlowApply(0), resnet50Apply(0), enableEventNotify(1),
            tempCorrection(0), colorPalette(0), ptzStatus("off") {}
    };
    
    static DeviceSetting& getInstance();
    
    bool load(const std::string& filename);
    bool save(const std::string& filename);
    bool save();  // 현재 파일에 저장
    
    // 설정 접근
    const Settings& get() const { return settings_; }
    Settings& getMutable() { return settings_; }
    
    // 개별 설정 업데이트
    void setRecordStatus(bool status);
    void setAnalysisStatus(bool status);
    void setNvInterval(int interval);
    void setOptFlowApply(bool apply);
    void setResnet50Apply(bool apply);
    void setEventNotify(bool enable);
    void setTempCorrection(int correction);
    
    // 변경 감지
    bool hasChanged() const { return changed_; }
    void resetChangeFlag() { changed_ = false; }
    
private:
    DeviceSetting();
    ~DeviceSetting();
    DeviceSetting(const DeviceSetting&) = delete;
    DeviceSetting& operator=(const DeviceSetting&) = delete;
    
private:
    Settings settings_;
    std::string currentFile_;
    std::atomic<bool> changed_;
};

#endif // DEVICE_SETTING_H