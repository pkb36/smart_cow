#include "Pipeline.h"
#include "CameraSource.h"
#include "StreamOutput.h"
#include "../utils/Config.h"
#include "../utils/Logger.h"
#include "../utils/DeviceSetting.h"
#include <gst/gst.h>
#include <sstream>

Pipeline::Pipeline()
    : pipeline_(nullptr)
    , bus_(nullptr)
    , busWatchId_(0)
    , isRunning_(false) {
    
    LOG_INFO("Pipeline created");
}

Pipeline::~Pipeline() {
    stop();
    
    if (busWatchId_) {
        g_source_remove(busWatchId_);
        busWatchId_ = 0;
    }
    
    if (bus_) {
        gst_object_unref(bus_);
        bus_ = nullptr;
    }
    
    if (pipeline_) {
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
    }
}

bool Pipeline::init(const Config& config) {
    // 설정 저장
    config_ = std::make_unique<Config>();
    *config_ = config;
    
    // GStreamer 파이프라인 생성
    if (!createPipeline()) {
        return false;
    }
    
    // 카메라 소스 설정
    if (!setupCameras(config)) {
        return false;
    }
    
    // 요소 연결
    if (!linkElements()) {
        return false;
    }

    // 스트림 출력 설정
    if (!setupOutputs(config)) {
        return false;
    }
    
    LOG_INFO("Pipeline initialized successfully");
    return true;
}

bool Pipeline::start() {
    LOG_INFO("Starting pipeline...");
    
    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        LOG_ERROR("Failed to set pipeline to PLAYING state");
        
        // 오류 메시지 확인
        GstMessage* msg = gst_bus_timed_pop_filtered(
            gst_element_get_bus(pipeline_), 0,
            static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_WARNING));
        
        if (msg) {
            handleBusMessage(msg);
            gst_message_unref(msg);
        }
        
        return false;
    }

    printPipelineElements();

    LOG_INFO("Pipeline started");
    return true;
}

void Pipeline::stop() {
    if (!isRunning_) {
        return;
    }
    
    LOG_INFO("Stopping pipeline...");
    
    // EOS 이벤트 전송
    gst_element_send_event(pipeline_, gst_event_new_eos());
    
    // 상태 변경 대기
    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_NULL);
    if (ret == GST_STATE_CHANGE_ASYNC) {
        // 상태 변경 완료 대기 (최대 5초)
        GstState state, pending;
        ret = gst_element_get_state(pipeline_, &state, &pending, 5 * GST_SECOND);
    }
    
    isRunning_ = false;
    LOG_INFO("Pipeline stopped");
}

bool Pipeline::isRunning() const {
    return isRunning_;
}

GstState Pipeline::getState() const {
    if (!pipeline_) {
        return GST_STATE_NULL;
    }
    
    GstState state, pending;
    gst_element_get_state(pipeline_, &state, &pending, 0);
    return state;
}

void Pipeline::handleBusMessage(GstMessage* message) {
    switch (GST_MESSAGE_TYPE(message)) {
        case GST_MESSAGE_ERROR: {
            GError* err;
            gchar* debug_info;
            
            gst_message_parse_error(message, &err, &debug_info);
            LOG_ERROR("Error from element %s: %s",
                     GST_OBJECT_NAME(message->src), err->message);
            LOG_ERROR("Debug info: %s", debug_info ? debug_info : "none");
            
            g_clear_error(&err);
            g_free(debug_info);
            
            // 에러 시 파이프라인 정지
            stop();
            break;
        }
        
        case GST_MESSAGE_WARNING: {
            GError* err;
            gchar* debug_info;
            
            gst_message_parse_warning(message, &err, &debug_info);
            LOG_WARN("Warning from element %s: %s",
                    GST_OBJECT_NAME(message->src), err->message);
            
            g_clear_error(&err);
            g_free(debug_info);
            break;
        }
        
        case GST_MESSAGE_INFO: {
            GError* err;
            gchar* debug_info;
            
            gst_message_parse_info(message, &err, &debug_info);
            LOG_INFO("Info from element %s: %s",
                    GST_OBJECT_NAME(message->src), err->message);
            
            g_clear_error(&err);
            g_free(debug_info);
            break;
        }
        
        case GST_MESSAGE_STATE_CHANGED: {
            if (GST_MESSAGE_SRC(message) == GST_OBJECT(pipeline_)) {
                GstState old_state, new_state, pending_state;
                gst_message_parse_state_changed(message, &old_state, &new_state, &pending_state);
                
                LOG_DEBUG("Pipeline state changed from %s to %s",
                         gst_element_state_get_name(old_state),
                         gst_element_state_get_name(new_state));
            }
            break;
        }
        
        case GST_MESSAGE_EOS:
            LOG_INFO("End of stream reached");
            stop();
            break;
            
        default:
            break;
    }
}

bool Pipeline::createPipeline() {
    // 파이프라인 생성
    pipeline_ = gst_pipeline_new("main-pipeline");
    if (!pipeline_) {
        LOG_ERROR("Failed to create pipeline");
        return false;
    }
    
    // 버스 설정
    bus_ = gst_element_get_bus(pipeline_);
    busWatchId_ = gst_bus_add_watch(bus_, Pipeline::busCallback, this);
    
    return true;
}

bool Pipeline::setupCameras(const Config& config) {
    int deviceCount = config.getDeviceCount();
    
    cameras_.clear();
    cameras_.reserve(deviceCount);
    
    for (int i = 0; i < deviceCount; i++) {
        const CameraConfig& camConfig = config.getCameraConfig(i);

        LOG_INFO("=== Camera Setup ===");
        LOG_INFO("Index: %d", i);
        LOG_INFO("Type: %s", (camConfig.type == CameraType::RGB) ? "RGB" : "THERMAL");
        LOG_INFO("Source Port: %d", camConfig.source.port);
        LOG_INFO("Output Port: %d", 5000 + i);
        LOG_INFO("Bitrate: %d", camConfig.encoder.bitrate);
        LOG_INFO("==================");
        
        auto camera = std::make_unique<CameraSource>(camConfig.type, i);
        
        // 새로운 init 메서드 사용 (pipeline 전달)
        if (!camera->init(camConfig, pipeline_)) {
            LOG_ERROR("Failed to initialize camera %d", i);
            return false;
        }
        
        cameras_.push_back(std::move(camera));
    }
    
    LOG_INFO("Set up %d cameras", deviceCount);
    return true;
}

bool Pipeline::setupOutputs(const Config& config) {
    outputs_.clear();
    
    int deviceCount = config.getDeviceCount();
    
    // 각 카메라의 main_tee 찾기 (Inter Plugin 방식)
    for (int cam = 0; cam < deviceCount; cam++) {
        // CameraSource에서 main_tee 가져오기
        if (cam < cameras_.size()) {
            GstElement* mainTee = cameras_[cam]->getMainTee();  // 새 메서드 필요
            
            if (mainTee) {
                LOG_INFO("Found main_tee for camera %d", cam);
                
                // WebRTC용 출력은 이미 CameraSource에서 생성됨
                // 추가 출력이 필요한 경우에만 여기서 생성
                
            } else {
                LOG_WARN("Main tee not found for camera %d", cam);
            }
        }
    }
    
    LOG_INFO("Setup completed for Inter Plugin outputs");
    return true;
}

bool Pipeline::linkElements() {
    // 새로운 구조에서는 각 카메라가 이미 내부적으로 완전히 연결되어 있음
    // 여기서는 추가적인 연결이나 설정만 처리
    
    LOG_INFO("All camera pipelines are self-contained, no additional linking needed");
    
    // 필요한 경우 카메라 간 연결이나 추가 처리
    // 예: 모든 카메라의 출력을 하나의 Muxer로 합치는 경우
    // 예: 공통 녹화 싱크를 추가하는 경우
    
    return true;
}

gboolean Pipeline::busCallback(GstBus* bus, GstMessage* message, gpointer data) {
    Pipeline* self = static_cast<Pipeline*>(data);
    self->handleBusMessage(message);
    return TRUE;
}

void Pipeline::printPipelineElements() {
    if (!pipeline_) {
        printf("Pipeline is NULL\n");
        return;
    }
    
    printf("\n=== PIPELINE STRUCTURE ===\n");
    printf("Pipeline: %s\n", GST_ELEMENT_NAME(pipeline_));
    
    printBinElements(GST_BIN(pipeline_), 1);
    
    printf("========================\n\n");
}

void Pipeline::printBinElements(GstBin* bin, int level) {
    GstIterator* it = gst_bin_iterate_elements(bin);
    GValue item = G_VALUE_INIT;
    
    while (gst_iterator_next(it, &item) == GST_ITERATOR_OK) {
        GstElement* element = GST_ELEMENT(g_value_get_object(&item));
        
        // 들여쓰기
        for (int i = 0; i < level; i++) {
            printf("  ");
        }
        
        gchar* name = gst_element_get_name(element);
        gchar* factory_name = gst_plugin_feature_get_name(
            GST_PLUGIN_FEATURE(gst_element_get_factory(element)));
        
        printf("├─ %s (%s)\n", name, factory_name);
        
        // Bin인 경우 재귀적으로 출력
        if (GST_IS_BIN(element)) {
            printBinElements(GST_BIN(element), level + 1);
        }
        
        g_free(name);
        g_value_reset(&item);
    }
    
    g_value_unset(&item);
    gst_iterator_free(it);
}

std::string Pipeline::getPipelineString() const {
    if (!pipeline_) return "";
    
    std::ostringstream oss;
    
    // 각 카메라별로 파이프라인 문자열 생성
    for (int i = 0; i < cameras_.size(); i++) {
        oss << "\n=== Camera " << i << " Pipeline ===\n";
        
        // 소스 체인
        oss << "shmsrc socket-path=/tmp/RGB_Camera.sock" 
            << " ! ";
        
        oss << "nvvideoconvert ! ";
        oss << "clockoverlay time-format=\"%D %H:%M:%S\" font-desc=\"Arial, 18\" ! ";
        oss << "videorate ! ";
        
        // caps 필터
        oss << "video/x-raw,width=1920,height=1080,framerate=10/1 ! ";
        
        oss << "queue max-size-buffers=5 leaky=downstream ! ";
        oss << "tee name=tee" << i << " ";
        
        // 추론이 활성화된 경우
        if (config_->getCameraConfig(i).inference.enabled) {
            oss << "\n\n# Inference branch\n";
            oss << "tee" << i << ". ! queue ! ";
            oss << "videoscale ! ";
            oss << "video/x-raw,width=" << config_->getCameraConfig(i).inference.scale_width 
                << ",height=" << config_->getCameraConfig(i).inference.scale_height << " ! ";
            oss << "nvvideoconvert ! ";
            oss << "nvstreammux batch-size=1 width=" << config_->getCameraConfig(i).inference.scale_width
                << " height=" << config_->getCameraConfig(i).inference.scale_height 
                << " live-source=1 ! ";
            oss << "nvinfer config-file-path=" << config_->getCameraConfig(i).inference.config_file << " ! ";
            oss << "nvof ! ";
            oss << "nvvideoconvert ! ";
            oss << "dspostproc ! ";
            oss << "nvdsosd ! ";
            oss << "nvvideoconvert ! ";
            oss << "tee name=main_tee" << i << " ";
            
            // WebRTC 출력
            oss << "\n\n# WebRTC output\n";
            oss << "main_tee" << i << ". ! queue ! ";
            oss << "nvvideoconvert ! ";
            oss << "capsfilter caps=\"video/x-raw,format=I420";
            oss << ",width=1920,height=1080";
            oss << ",framerate=10/1\" ! ";
            oss << "intervideosink channel=";
            oss << "Webrtc_RGB_Camera";
            
            // Fakesink
            oss << "\n\n# Fakesink (for pipeline stability)\n";
            oss << "main_tee" << i << ". ! queue max-size-buffers=1 leaky=downstream ! ";
            oss << "fakesink sync=false";
        } else {
            // 추론 비활성화
            oss << "\n\n# Direct WebRTC output (no inference)\n";
            oss << "tee" << i << ". ! queue ! ";
            oss << "nvvideoconvert ! ";
            oss << "intervideosink channel=";
            oss << "Webrtc_RGB_Camera";
        }
        
        oss << "\n";
    }
    
    return oss.str();
}