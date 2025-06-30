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
    
    // 스트림 출력 설정
    if (!setupOutputs(config)) {
        return false;
    }
    
    // 요소 연결
    if (!linkElements()) {
        return false;
    }
    
    LOG_INFO("Pipeline initialized successfully");
    return true;
}

bool Pipeline::start() {
    if (isRunning_) {
        LOG_WARN("Pipeline already running");
        return true;
    }
    
    LOG_INFO("Starting pipeline...");
    
    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        LOG_ERROR("Failed to set pipeline to PLAYING state");
        
        // 디버깅을 위한 에러 메시지 출력
        GstMessage* msg;
        GstBus* bus = gst_element_get_bus(pipeline_);
        while ((msg = gst_bus_pop(bus)) != nullptr) {
            if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
                GError* err;
                gchar* debug_info;
                
                gst_message_parse_error(msg, &err, &debug_info);
                LOG_ERROR("Error from element %s: %s",
                         GST_OBJECT_NAME(msg->src), err->message);
                LOG_ERROR("Debug info: %s", debug_info ? debug_info : "none");
                
                g_clear_error(&err);
                g_free(debug_info);
            }
            gst_message_unref(msg);
        }
        gst_object_unref(bus);
        
        return false;
    }
    
    isRunning_ = true;
    LOG_INFO("Pipeline started successfully");
    
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

CameraSource* Pipeline::getCamera(int index) const {
    if (index >= 0 && index < static_cast<int>(cameras_.size())) {
        return cameras_[index].get();
    }
    return nullptr;
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
        
        auto camera = std::make_unique<CameraSource>(camConfig.type, i);
        
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
    int maxStreamCount = config.getMaxStreamCount();
    int basePort = config.getStreamBasePort();
    
    // 각 카메라에 대해 스트림 출력 생성
    for (int cam = 0; cam < deviceCount; cam++) {
        GstElement* mainTee = nullptr;
        GstElement* subTee = nullptr;
        
        // Tee 요소 찾기
        gchar teeName[64];
        g_snprintf(teeName, sizeof(teeName), "video_enc_tee1_%d", cam);
        mainTee = gst_bin_get_by_name(GST_BIN(pipeline_), teeName);
        
        g_snprintf(teeName, sizeof(teeName), "video_enc_tee2_%d", cam);
        subTee = gst_bin_get_by_name(GST_BIN(pipeline_), teeName);
        
        if (!mainTee || !subTee) {
            LOG_ERROR("Failed to find tee elements for camera %d", cam);
            continue;
        }
        
        // 메인 스트림 출력 (Sender + Recorder + Event Recorder)
        for (int stream = 0; stream < maxStreamCount + 2; stream++) {
            auto output = std::make_unique<StreamOutput>(cam, stream, StreamOutput::MAIN_STREAM);
            if (output->init(pipeline_, mainTee, basePort)) {
                outputs_.push_back(std::move(output));
            }
        }
        
        // 서브 스트림 출력 (Sender + Recorder + Event Recorder)
        for (int stream = 0; stream < maxStreamCount + 2; stream++) {
            auto output = std::make_unique<StreamOutput>(cam, stream, StreamOutput::SUB_STREAM);
            if (output->init(pipeline_, subTee, basePort)) {
                outputs_.push_back(std::move(output));
            }
        }
        
        gst_object_unref(mainTee);
        gst_object_unref(subTee);
    }
    
    LOG_INFO("Set up %zu stream outputs", outputs_.size());
    return true;
}

bool Pipeline::linkElements() {
    // 카메라별 인코더 생성 및 연결
    for (size_t i = 0; i < cameras_.size(); i++) {
        const CameraConfig& config = config_->getCameraConfig(i);
        
        // 메인 인코더 체인 생성
        GError* error = nullptr;
        GstElement* encBin1 = gst_parse_bin_from_description(
            config.encoder.c_str(), TRUE, &error);
        
        if (error) {
            LOG_ERROR("Failed to create encoder 1 for camera %zu: %s", i, error->message);
            g_error_free(error);
            return false;
        }
        
        gchar binName[64];
        g_snprintf(binName, sizeof(binName), "encoder1_%zu", i);
        gst_element_set_name(encBin1, binName);
        
        // 서브 인코더 체인 생성 (있는 경우)
        // TODO: config에 encoder2 추가 필요
        
        // 파이프라인에 추가
        gst_bin_add(GST_BIN(pipeline_), encBin1);
        
        // 카메라 출력과 인코더 연결
        GstElement* cameraOutput = cameras_[i]->getOutputElement();
        if (!gst_element_link(cameraOutput, encBin1)) {
            LOG_ERROR("Failed to link camera %zu to encoder", i);
            return false;
        }
        
        // Tee 요소 생성 및 연결
        g_snprintf(binName, sizeof(binName), "video_enc_tee1_%zu", i);
        GstElement* tee1 = gst_element_factory_make("tee", binName);
        gst_bin_add(GST_BIN(pipeline_), tee1);
        
        if (!gst_element_link(encBin1, tee1)) {
            LOG_ERROR("Failed to link encoder to tee for camera %zu", i);
            return false;
        }
    }
    
    LOG_INFO("All elements linked successfully");
    return true;
}

gboolean Pipeline::busCallback(GstBus* bus, GstMessage* message, gpointer data) {
    Pipeline* self = static_cast<Pipeline*>(data);
    self->handleBusMessage(message);
    return TRUE;
}