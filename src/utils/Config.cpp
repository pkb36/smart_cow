#include "Config.h"
#include "Logger.h"
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class Config::Impl {
public:
    bool load(const std::string& filename) {
        try {
            std::ifstream file(filename);
            if (!file.is_open()) {
                LOG_ERROR("Failed to open config file: %s", filename.c_str());
                return false;
            }
            
            json j;
            file >> j;
            
            // 기본 설정
            config_.cameraId = j.value("camera_id", "");
            config_.deviceCount = j.value("device_cnt", 0);
            config_.maxStreamCount = j.value("max_stream_cnt", 10);
            config_.streamBasePort = j.value("stream_base_port", 5000);
            config_.snapshotPath = j.value("snapshot_path", "/home/nvidia/webrtc");
            config_.apiPort = j.value("api_port", 8080);  // API 서버 포트
            
            // TTY 설정
            if (j.contains("tty")) {
                auto tty = j["tty"];
                ttyDevice_ = tty.value("name", "/dev/ttyUSB0");
                ttyBaudrate_ = tty.value("baudrate", 38400);
            }
            
            if (j.contains("cameras") && j["cameras"].is_array()) {
                config_.cameras.clear();
                
                for (const auto& cam : j["cameras"]) {
                    CameraConfig camera;
                    
                    camera.name = cam.value("name", "");
                    camera.type = (cam.value("type", "") == "rgb") ? 
                                CameraType::RGB : CameraType::THERMAL;
                    
                    // 소스 설정
                    if (cam.contains("source")) {
                        auto src = cam["source"];
                        camera.source.protocol = src.value("protocol", "udp");
                        camera.source.port = src.value("port", 0);
                        camera.source.encoding = src.value("encoding", "h264");
                        camera.source.width = src.value("width", 1920);
                        camera.source.height = src.value("height", 1080);
                        camera.source.framerate = src.value("framerate", 30);
                    }
                    
                    // 추론 설정
                    if (cam.contains("inference")) {
                        auto inf = cam["inference"];
                        camera.inference.enabled = inf.value("enabled", false);
                        camera.inference.config_file = inf.value("config_file", "");
                        camera.inference.scale_width = inf.value("scale_width", 1280);
                        camera.inference.scale_height = inf.value("scale_height", 720);
                    }
                    
                    // 인코더 설정
                    if (cam.contains("encoder")) {
                        auto enc = cam["encoder"];
                        camera.encoder.codec = enc.value("codec", "h264");
                        camera.encoder.preset = enc.value("preset", "fast");
                        camera.encoder.bitrate = enc.value("bitrate", 2000000);
                        camera.encoder.idr_interval = enc.value("idr_interval", 30);
                    }
                    
                    config_.cameras.push_back(camera);
                }
            }
            
            // 서버 설정
            serverUrl_ = j.value("server_ip", "ws://localhost:8443");
            
            // 녹화 설정
            recordPath_ = j.value("record_path", "/home/nvidia/record");
            recordDuration_ = j.value("record_duration", 300);  // 5분
            eventBufferTime_ = j.value("event_buf_time", 15);   // 15초
            recordEncIndex_ = j.value("record_enc_index", 0);
            eventRecordEncIndex_ = j.value("event_record_enc_index", 0);
            
            // HTTP 서비스
            httpServicePort_ = j.value("http_service_port", "8080");
            
            LOG_INFO("Config loaded successfully from %s", filename.c_str());
            LOG_INFO("Camera ID: %s, Devices: %d, Stream base port: %d",
                     config_.cameraId.c_str(), config_.deviceCount, config_.streamBasePort);
            
            return true;
            
        } catch (const json::exception& e) {
            LOG_ERROR("JSON parsing error: %s", e.what());
            return false;
        } catch (const std::exception& e) {
            LOG_ERROR("Config loading error: %s", e.what());
            return false;
        }
    }
    
public:
    SystemConfig config_;
    
    // 추가 설정들
    std::string ttyDevice_;
    int ttyBaudrate_;
    std::string serverUrl_;
    std::string recordPath_;
    int recordDuration_;
    int eventBufferTime_;
    int recordEncIndex_;
    int eventRecordEncIndex_;
    std::string httpServicePort_;
};

// Config 구현
Config::Config() : pImpl(std::make_unique<Impl>()) {}
Config::Config(const Config& other) : pImpl(std::make_unique<Impl>()) {
    // other의 내용을 복사
    *pImpl = *other.pImpl;
}

Config& Config::operator=(const Config& other) {
    if (this != &other) {
        *pImpl = *other.pImpl;
    }
    return *this;
}

Config::~Config() = default;

bool Config::load(const std::string& filename) {
    return pImpl->load(filename);
}

const SystemConfig& Config::getSystemConfig() const {
    return pImpl->config_;
}

std::string Config::getCameraId() const {
    return pImpl->config_.cameraId;
}

int Config::getDeviceCount() const {
    return pImpl->config_.deviceCount;
}

int Config::getMaxStreamCount() const {
    return pImpl->config_.maxStreamCount;
}

int Config::getStreamBasePort() const {
    return pImpl->config_.streamBasePort;
}

int Config::getApiPort() const {
    return pImpl->config_.apiPort;
}

const CameraConfig& Config::getCameraConfig(int index) const {
    if (index < 0 || index >= static_cast<int>(pImpl->config_.cameras.size())) {
        static CameraConfig empty;
        return empty;
    }
    return pImpl->config_.cameras[index];
}

std::string Config::getTtyDevice() const {
    return pImpl->ttyDevice_;
}

int Config::getTtyBaudrate() const {
    return pImpl->ttyBaudrate_;
}

std::string Config::getServerUrl() const {
    return pImpl->serverUrl_;
}

std::string Config::getRecordPath() const {
    return pImpl->recordPath_;
}

int Config::getRecordDuration() const {
    return pImpl->recordDuration_;
}

int Config::getEventBufferTime() const {
    return pImpl->eventBufferTime_;
}

std::string Config::getCodecName() const {
    // 첫 번째 비디오 인코더에서 코덱 추출
    if (!pImpl->config_.cameras.empty()) {
        const std::string& encoder = pImpl->config_.cameras[0].encoder.codec;
        if (encoder.find("vp8") != std::string::npos) return "VP8";
        if (encoder.find("vp9") != std::string::npos) return "VP9";
        if (encoder.find("264") != std::string::npos) return "H264";
    }
    return "VP8";  // 기본값
}