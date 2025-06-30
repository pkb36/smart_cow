#include "DeviceSetting.h"
#include "Logger.h"
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

DeviceSetting::DeviceSetting() : changed_(false) {}
DeviceSetting::~DeviceSetting() {}

DeviceSetting& DeviceSetting::getInstance() {
    static DeviceSetting instance;
    return instance;
}

bool DeviceSetting::load(const std::string& filename) {
    try {
        std::ifstream file(filename);
        if (!file.is_open()) {
            LOG_WARN("Device settings file not found: %s, using defaults", filename.c_str());
            currentFile_ = filename;
            return true;  // 파일이 없어도 기본값 사용
        }
        
        json j;
        file >> j;
        
        // 녹화 설정
        settings_.recordStatus = j.value("record_status", 0);
        
        // 분석 설정
        settings_.analysisStatus = j.value("analysis_status", 0);
        settings_.nvInterval = j.value("nv_interval", 0);
        
        // 탐지 설정
        settings_.optFlowApply = j.value("opt_flow_apply", 0);
        settings_.resnet50Apply = j.value("resnet50_apply", 0);
        settings_.enableEventNotify = j.value("enable_event_notify", 1);
        
        // 온도 설정
        settings_.tempCorrection = j.value("temp_correction", 0);
        
        // 모드 설정
        settings_.ptzStatus = j.value("ptz_status", "off");
        settings_.colorPalette = j.value("color_pallet", 0);  // 오타 그대로 유지
        
        currentFile_ = filename;
        changed_ = false;
        
        LOG_INFO("Device settings loaded from %s", filename.c_str());
        LOG_INFO("Record: %s, Analysis: %s, OptFlow: %s, ResNet50: %s, EventNotify: %s",
                 settings_.recordStatus ? "ON" : "OFF",
                 settings_.analysisStatus ? "ON" : "OFF",
                 settings_.optFlowApply ? "ON" : "OFF",
                 settings_.resnet50Apply ? "ON" : "OFF",
                 settings_.enableEventNotify ? "ON" : "OFF");
        
        return true;
        
    } catch (const json::exception& e) {
        LOG_ERROR("JSON parsing error in device settings: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        LOG_ERROR("Device settings loading error: %s", e.what());
        return false;
    }
}

bool DeviceSetting::save(const std::string& filename) {
    try {
        json j;
        
        // 녹화 설정
        j["record_status"] = settings_.recordStatus;
        
        // 분석 설정
        j["analysis_status"] = settings_.analysisStatus;
        j["nv_interval"] = settings_.nvInterval;
        
        // 탐지 설정
        j["opt_flow_apply"] = settings_.optFlowApply;
        j["resnet50_apply"] = settings_.resnet50Apply;
        j["enable_event_notify"] = settings_.enableEventNotify;
        
        // 온도 설정
        j["temp_correction"] = settings_.tempCorrection;
        
        // 모드 설정
        j["ptz_status"] = settings_.ptzStatus;
        j["color_pallet"] = settings_.colorPalette;  // 오타 그대로 유지
        
        // 파일 쓰기
        std::ofstream file(filename);
        if (!file.is_open()) {
            LOG_ERROR("Failed to open device settings file for writing: %s", filename.c_str());
            return false;
        }
        
        file << j.dump(4);  // 들여쓰기 4칸
        file.close();
        
        if (!filename.empty()) {
            currentFile_ = filename;
        }
        changed_ = false;
        
        LOG_INFO("Device settings saved to %s", filename.c_str());
        return true;
        
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to save device settings: %s", e.what());
        return false;
    }
}

bool DeviceSetting::save() {
    if (currentFile_.empty()) {
        // 기본 파일명 사용
        currentFile_ = "device_setting.json";
        LOG_INFO("Using default filename: %s", currentFile_.c_str());
    }
    
    // 파일명이 유효한지 확인
    if (currentFile_.find_first_of("\x00\xff", 0, 2) != std::string::npos) {
        LOG_ERROR("Invalid filename detected, using default");
        currentFile_ = "device_setting.json";
    }
    
    return save(currentFile_);
}

void DeviceSetting::setRecordStatus(bool status) {
    if (settings_.recordStatus != status) {
        settings_.recordStatus = status;
        changed_ = true;
        LOG_INFO("Record status changed to: %s", status ? "ON" : "OFF");
    }
}

void DeviceSetting::setAnalysisStatus(bool status) {
    if (settings_.analysisStatus != status) {
        settings_.analysisStatus = status;
        changed_ = true;
        LOG_INFO("Analysis status changed to: %s", status ? "ON" : "OFF");
    }
}

void DeviceSetting::setNvInterval(int interval) {
    if (settings_.nvInterval != interval) {
        settings_.nvInterval = interval;
        changed_ = true;
        LOG_INFO("NV interval changed to: %d", interval);
    }
}

void DeviceSetting::setOptFlowApply(bool apply) {
    if (settings_.optFlowApply != apply) {
        settings_.optFlowApply = apply;
        changed_ = true;
        LOG_INFO("Optical flow apply changed to: %s", apply ? "ON" : "OFF");
    }
}

void DeviceSetting::setResnet50Apply(bool apply) {
    if (settings_.resnet50Apply != apply) {
        settings_.resnet50Apply = apply;
        changed_ = true;
        LOG_INFO("ResNet50 apply changed to: %s", apply ? "ON" : "OFF");
    }
}

void DeviceSetting::setEventNotify(bool enable) {
    if (settings_.enableEventNotify != enable) {
        settings_.enableEventNotify = enable;
        changed_ = true;
        LOG_INFO("Event notification changed to: %s", enable ? "ON" : "OFF");
    }
}

void DeviceSetting::setTempCorrection(int correction) {
    if (settings_.tempCorrection != correction) {
        settings_.tempCorrection = correction;
        changed_ = true;
        LOG_INFO("Temperature correction changed to: %d", correction);
    }
}