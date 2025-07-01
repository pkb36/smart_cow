#include <gst/gst.h>
#include <glib.h>
#include <signal.h>
#include <iostream>
#include <memory>
#include <thread>
#include <atomic>
#include <fstream>
#include <sys/statvfs.h>
#include <chrono>
// Pipeline
#include "pipeline/Pipeline.h"
#include "pipeline/CameraSource.h"

// Utils
#include "utils/Config.h"
#include "utils/Logger.h"
#include "utils/DeviceSetting.h"
#include "utils/ProcessManager.h"
#include "utils/SerialComm.h"

// Control
#include "control/PTZController.h"
#include "control/CommandPipe.h"

// API
#include "api/ApiServer.h"

// Signaling & WebRTC
#include "signaling/SignalingClient.h"
#include "webrtc/PeerManager.h"

// 전역 변수
static GMainLoop* g_mainLoop = nullptr;
static std::atomic<bool> g_running(true);
static std::unique_ptr<Pipeline> g_pipeline;
static std::unique_ptr<Config> g_config;
static std::unique_ptr<PTZController> g_ptzController;
static std::unique_ptr<ApiServer> g_apiServer;
static std::unique_ptr<SignalingClient> g_signalingClient;
static std::unique_ptr<PeerManager> g_peerManager;
static std::unique_ptr<CommandPipe> g_commandPipe;

class StatusReporter {
public:
    StatusReporter(SignalingClient* client) 
        : client_(client)
        , running_(true) {
        
        thread_ = std::thread([this]() {
            while (running_) {
                updateStatus();
                std::this_thread::sleep_for(std::chrono::seconds(5));
            }
        });
    }
    
    ~StatusReporter() {
        stop();
    }
    
    void stop() {
        running_ = false;
        if (thread_.joinable()) {
            thread_.join();
        }
    }
    
private:
    void updateStatus() {
        try {
            double cpuTemp = getCpuTemperature();
            double gpuTemp = getGpuTemperature();
            int diskUsage = getDiskUsage();
            
            client_->updateCameraStatus("Off", cpuTemp, gpuTemp, diskUsage);
        } catch (const std::exception& e) {
            LOG_ERROR("Status update error: %s", e.what());
        }
    }
    
    double getCpuTemperature() {
        std::ifstream file("/sys/devices/virtual/thermal/thermal_zone0/temp");
        if (file.is_open()) {
            int temp;
            file >> temp;
            file.close();
            return temp / 1000.0;
        }
        return 0.0;
    }
    
    double getGpuTemperature() {
        std::ifstream file("/sys/devices/virtual/thermal/thermal_zone1/temp");
        if (file.is_open()) {
            int temp;
            file >> temp;
            file.close();
            return temp / 1000.0;
        }
        return 0.0;
    }
    
    int getDiskUsage() {
        struct statvfs stat;
        if (statvfs("/", &stat) == 0) {
            unsigned long total = stat.f_blocks * stat.f_frsize;
            unsigned long available = stat.f_bavail * stat.f_frsize;
            unsigned long used = total - available;
            return (int)((used * 100) / total);
        }
        return 0;
    }
    
private:
    SignalingClient* client_;
    std::atomic<bool> running_;
    std::thread thread_;
};

static std::unique_ptr<StatusReporter> g_statusReporter;

// 시그널 핸들러
static void signalHandler(int sig) {
    LOG_INFO("Received signal %d", sig);
    g_running = false;
    
    if (g_mainLoop) {
        g_main_loop_quit(g_mainLoop);
    }
}

// 프로그램 종료 시 정리
static void cleanup() {
    LOG_INFO("Cleaning up resources...");
    
    // 파이프라인 정지
    if (g_pipeline) {
        g_pipeline->stop();
    }
    
    // API 서버 정지
    if (g_apiServer) {
        g_apiServer->stop();
    }
    
    // 시그널링 연결 해제
    if (g_signalingClient) {
        g_signalingClient->disconnect();
    }
    
    // 모든 WebRTC 프로세스 정지
    if (g_peerManager) {
        g_peerManager.reset();
    }
    
    // 프로세스 정리
    ProcessManager::getInstance().stopAllProcesses();
    
    // 설정 저장
    DeviceSetting::getInstance().save();
    
    LOG_INFO("Cleanup completed");
}

// 커맨드 파이프 핸들러
static void handlePipeCommand(const std::string& command) {
    LOG_INFO("Received pipe command: %s", command.c_str());
    
    // PTZ 명령 처리
    if (g_ptzController && (command == "up" || command == "down" || 
        command == "left" || command == "right" || command == "enter" ||
        command == "zoom_init" || command == "ir_init")) {
        g_ptzController->sendPipeCommand(command);
        return;
    }
    
    // 녹화 명령
    if (command == "record_start") {
        auto& settings = DeviceSetting::getInstance().getMutable();
        settings.recordStatus = 1;
        
        ProcessManager& pm = ProcessManager::getInstance();
        pm.startRecording(g_config->getDeviceCount(), 
                         g_config->getStreamBasePort(),
                         g_config->getCodecName(),
                         g_config->getRecordPath(),
                         g_config->getRecordDuration());
    } else if (command == "record_stop") {
        auto& settings = DeviceSetting::getInstance().getMutable();
        settings.recordStatus = 0;
        
        ProcessManager::getInstance().stopRecording();
    }
    
    // 분석 on/off
    else if (command == "analysis_on") {
        auto& settings = DeviceSetting::getInstance().getMutable();
        settings.analysisStatus = 1;
        settings.nvInterval = 0;
    } else if (command == "analysis_off") {
        auto& settings = DeviceSetting::getInstance().getMutable();
        settings.analysisStatus = 0;
        settings.nvInterval = INT32_MAX;
    }
}

// 시그널링 메시지 핸들러
static void handleSignalingMessage(const SignalingClient::Message& message) {
    LOG_DEBUG("Signaling message: type=%s, peer=%s", 
              message.type.c_str(), message.peerId.c_str());
    
    if (g_peerManager) {
        g_peerManager->handleSignalingMessage(message);
    }
}

// 시그널링 상태 변경 핸들러
static void handleSignalingStateChange(SignalingClient::ConnectionState state) {
    const char* stateStr = "UNKNOWN";
    switch (state) {
        case SignalingClient::ConnectionState::DISCONNECTED:
            stateStr = "DISCONNECTED";
            break;
        case SignalingClient::ConnectionState::CONNECTING:
            stateStr = "CONNECTING";
            break;
        case SignalingClient::ConnectionState::CONNECTED:
            stateStr = "CONNECTED";
            break;
        case SignalingClient::ConnectionState::RECONNECTING:
            stateStr = "RECONNECTING";
            break;
    }
    
    LOG_INFO("Signaling connection state: %s", stateStr);
}

// 타이머 콜백들
static gboolean statusTimerCallback(gpointer data) {
    // 카메라 상태 정보 전송
    if (g_signalingClient && g_signalingClient->isConnected()) {
        // TODO: 상태 정보 수집 및 전송
        LOG_TRACE("Status timer tick");
    }
    
    return G_SOURCE_CONTINUE;
}

static gboolean checkTimerCallback(gpointer data) {
    // 시스템 체크
    ProcessManager::getInstance().checkProcesses();
    
    // 설정 변경 확인 및 저장
    if (DeviceSetting::getInstance().hasChanged()) {
        DeviceSetting::getInstance().save();
        DeviceSetting::getInstance().resetChangeFlag();
    }
    
    return G_SOURCE_CONTINUE;
}

static void printUsage(const char* progName) {
    std::cout << "Usage: " << progName << " [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --config=FILE    Configuration file (default: config.json)" << std::endl;
    std::cout << "  --pipe=PATH      Command pipe path (default: /home/nvidia/webrtc/webrtc_pipe)" << std::endl;
    std::cout << "  --help           Show this help message" << std::endl;
}

int main(int argc, char* argv[]) {
    // 명령줄 옵션 파싱
    gchar* configFile = nullptr;
    gchar* pipePath = nullptr;
    
    GOptionEntry entries[] = {
        {"config", 'c', 0, G_OPTION_ARG_STRING, &configFile, "Configuration file", "FILE"},
        {"pipe", 'p', 0, G_OPTION_ARG_STRING, &pipePath, "Command pipe path", "PATH"},
        {nullptr}
    };

    if (getenv("DISPLAY"))
    {
        printf("DISPLAY 환경변수 제거 중...\n");
        unsetenv("DISPLAY");
    }
    
    GError* error = nullptr;
    GOptionContext* context = g_option_context_new("- WebRTC camera streaming system");
    g_option_context_add_main_entries(context, entries, nullptr);
    g_option_context_add_group(context, gst_init_get_option_group());
    
    if (!g_option_context_parse(context, &argc, &argv, &error)) {
        std::cerr << "Option parsing failed: " << error->message << std::endl;
        g_error_free(error);
        g_option_context_free(context);
        return 1;
    }
    
    g_option_context_free(context);
    
    // 기본값 설정
    std::string configPath = configFile ? configFile : "config.json";
    std::string pipePathStr = pipePath ? pipePath : "/home/nvidia/webrtc/webrtc_pipe";
    
    g_free(configFile);
    g_free(pipePath);
    
    // GStreamer 초기화
    gst_init(&argc, &argv);
    
    // 로거 초기화

    Logger::getInstance().init("./logs", LogLevel::DEBUG);
    
    LOG_INFO("========================================");
    LOG_INFO("WebRTC Camera System Starting...");
    LOG_INFO("Version: 1.0.0");
    LOG_INFO("Config: %s", configPath.c_str());
    LOG_INFO("========================================");
    
    // 시그널 핸들러 설정
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // 종료 시 정리 함수 등록
    atexit(cleanup);
    
    try {
        // 설정 파일 로드
        g_config = std::make_unique<Config>();
        if (!g_config->load(configPath)) {
            LOG_FATAL("Failed to load configuration file: %s", configPath.c_str());
            return 1;
        }
        
        // 디바이스 설정 로드
        std::string deviceSettingPath = "device_setting.json";
        DeviceSetting::getInstance().load(deviceSettingPath);
        
        // 커맨드 파이프 생성
        g_commandPipe = std::make_unique<CommandPipe>(pipePathStr);
        g_commandPipe->setCommandCallback(handlePipeCommand);
        if (!g_commandPipe->open()) {
            LOG_ERROR("Failed to open command pipe");
        }
        
        // PTZ 컨트롤러 초기화 (시리얼 포트가 설정된 경우)
        std::string serialDevice = g_config->getTtyDevice();
        int serialBaudrate = g_config->getTtyBaudrate();
        
        if (!serialDevice.empty()) {
            g_ptzController = std::make_unique<PTZController>();
            if (!g_ptzController->init(serialDevice, serialBaudrate)) {
                LOG_WARN("PTZ controller initialization failed");
                g_ptzController.reset();
            }
        }
        
        // 파이프라인 생성 및 초기화
        g_pipeline = std::make_unique<Pipeline>();
        if (!g_pipeline->init(*g_config)) {
            LOG_FATAL("Failed to initialize pipeline");
            return 1;
        }
        
        // API 서버 시작
        g_apiServer = std::make_unique<ApiServer>(g_config->getApiPort());
        
        // 카메라별 검출 버퍼 등록
        for (int i = 0; i < g_config->getDeviceCount(); i++) {
            const CameraConfig& camConfig = g_config->getCameraConfig(i);
            auto* cameraSource = g_pipeline->getCamera(i);
            if (cameraSource) {
                g_apiServer->registerDetectionBuffer(camConfig.type, 
                                                   cameraSource->getDetectionBuffer());
            }
        }
        
        if (!g_apiServer->start()) {
            LOG_ERROR("Failed to start API server");
        }
        
        // 피어 매니저 생성
        g_peerManager = std::make_unique<PeerManager>(g_pipeline.get(), 
                                                     g_config->getMaxStreamCount());
        g_peerManager->init(g_config->getStreamBasePort(), 6000, g_config->getCodecName());
        
        // 시그널링 클라이언트 생성
        std::string serverUrl = g_config->getServerUrl();
        g_signalingClient = std::make_unique<SignalingClient>(serverUrl, 
                                                             g_config->getCameraId());

        g_peerManager->setSignalingClient(g_signalingClient.get());                                                     
        g_signalingClient->setMessageCallback(handleSignalingMessage);
        g_signalingClient->setStateCallback(handleSignalingStateChange);
        
        // 파이프라인 시작
        if (!g_pipeline->start()) {
            LOG_FATAL("Failed to start pipeline");
            return 1;
        }
        
        // 시그널링 서버 연결
        if (g_signalingClient->connect()) {
            // StatusReporter가 자동으로 스레드 관리
            g_statusReporter = std::make_unique<StatusReporter>(g_signalingClient.get());
            
            // 30초마다 서버로 전송
            g_signalingClient->startStatusReporting(30);
        }
        // 메인 루프 생성
        g_mainLoop = g_main_loop_new(nullptr, FALSE);
        
        // 타이머 설정
        g_timeout_add_seconds(5, statusTimerCallback, nullptr);  // 5초마다 상태 전송
        g_timeout_add_seconds(1, checkTimerCallback, nullptr);   // 1초마다 체크
        
        LOG_INFO("System started successfully. Entering main loop...");
        
        // 메인 루프 실행
        g_main_loop_run(g_mainLoop);
        
        // 정리
        g_main_loop_unref(g_mainLoop);
        g_mainLoop = nullptr;
        
    } catch (const std::exception& e) {
        LOG_FATAL("Exception caught: %s", e.what());
        return 1;
    }
    
    LOG_INFO("Program terminated normally");
    return 0;
}