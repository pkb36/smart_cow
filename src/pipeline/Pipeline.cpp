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
    
    // 3초 후 모든 요소 상태 확인
    g_timeout_add(3000, [](gpointer data) -> gboolean {
        Pipeline* self = static_cast<Pipeline*>(data);
        
        // 파이프라인의 모든 요소 순회
        GstIterator* it = gst_bin_iterate_elements(GST_BIN(self->pipeline_));
        GValue item = G_VALUE_INIT;
        gboolean done = FALSE;
        
        LOG_INFO("=== 파이프라인 요소 상태 ===");
        while (!done) {
            switch (gst_iterator_next(it, &item)) {
                case GST_ITERATOR_OK: {
                    GstElement* element = GST_ELEMENT(g_value_get_object(&item));
                    GstState state, pending;
                    gst_element_get_state(element, &state, &pending, 0);
                    
                    const gchar* name = gst_element_get_name(element);
                    LOG_INFO("요소: %s, 상태: %s, 대기: %s",
                            name,
                            gst_element_state_get_name(state),
                            gst_element_state_get_name(pending));
                    
                    g_value_reset(&item);
                    break;
                }
                case GST_ITERATOR_DONE:
                    done = TRUE;
                    break;
                default:
                    break;
            }
        }
        g_value_unset(&item);
        gst_iterator_free(it);

        const char* infer_elements[] = {
            "infer_queue_0", "infer_scale_0", "infer_conv_0", 
            "RGB", "nvinfer_1", "nvof", "dspostproc_1", "nvosd_1",
            "infer_queue_1", "infer_scale_1", "infer_conv_1",
            "thermal", "nvinfer_2", "dspostproc_2", "nvosd_2"
        };

        for (const char* name : infer_elements) {
            GstElement* elem = gst_bin_get_by_name(GST_BIN(self->pipeline_), name);
            if (elem) {
                GstState state, pending;
                gst_element_get_state(elem, &state, &pending, 0);
                if (state != GST_STATE_PLAYING) {
                    LOG_WARN("요소 %s 상태: %s (대기: %s)", 
                            name, 
                            gst_element_state_get_name(state),
                            gst_element_state_get_name(pending));
                }
                gst_object_unref(elem);
            }
        }
        
        return FALSE; // 한 번만 실행
    }, this);

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
    int maxStreamCount = config.getMaxStreamCount();
    int basePort = config.getStreamBasePort();
    
    // 각 카메라에 대해 스트림 출력 생성
    for (int cam = 0; cam < deviceCount; cam++) {
        GstElement* mainTee = nullptr;
        GstElement* subTee = nullptr;
        
        // Tee 요소 찾기 - 인코더 체인 내부에서 찾기
        gchar teeName[64];
        g_snprintf(teeName, sizeof(teeName), "video_enc_tee1_%d", cam);
        
        // 먼저 파이프라인 레벨에서 찾기
        mainTee = gst_bin_get_by_name(GST_BIN(pipeline_), teeName);
        
        if (!mainTee) {
            // 인코더 bin 내부에서 찾기
            gchar binName[64];
            g_snprintf(binName, sizeof(binName), "encoder1_%d", cam);
            GstElement* encBin = gst_bin_get_by_name(GST_BIN(pipeline_), binName);
            if (encBin) {
                mainTee = gst_bin_get_by_name(GST_BIN(encBin), teeName);
                if (!mainTee) {
                    // bin 내부의 모든 요소를 순회하며 tee 찾기
                    GstIterator* it = gst_bin_iterate_elements(GST_BIN(encBin));
                    GValue item = G_VALUE_INIT;
                    gboolean done = FALSE;
                    
                    while (!done) {
                        switch (gst_iterator_next(it, &item)) {
                            case GST_ITERATOR_OK: {
                                GstElement* element = GST_ELEMENT(g_value_get_object(&item));
                                gchar* elemName = gst_element_get_name(element);
                                if (g_str_has_prefix(elemName, "video_enc_tee1_")) {
                                    mainTee = GST_ELEMENT(gst_object_ref(element));
                                    done = TRUE;
                                }
                                g_free(elemName);
                                g_value_reset(&item);
                                break;
                            }
                            case GST_ITERATOR_DONE:
                                done = TRUE;
                                break;
                            default:
                                break;
                        }
                    }
                    g_value_unset(&item);
                    gst_iterator_free(it);
                }
                gst_object_unref(encBin);
            }
        }
        
        if (!mainTee) {
            LOG_WARN("Main tee not found for camera %d", cam);
            continue;
        }
        
        // 서브 tee 찾기
        g_snprintf(teeName, sizeof(teeName), "video_enc_tee2_%d", cam);
        subTee = gst_bin_get_by_name(GST_BIN(pipeline_), teeName);
        
        if (!subTee) {
            // 인코더2 bin 내부에서 찾기
            gchar binName[64];
            g_snprintf(binName, sizeof(binName), "encoder2_%d", cam);
            GstElement* encBin = gst_bin_get_by_name(GST_BIN(pipeline_), binName);
            if (encBin) {
                subTee = gst_bin_get_by_name(GST_BIN(encBin), teeName);
                if (!subTee) {
                    // bin 내부의 모든 요소를 순회하며 tee 찾기
                    GstIterator* it = gst_bin_iterate_elements(GST_BIN(encBin));
                    GValue item = G_VALUE_INIT;
                    gboolean done = FALSE;
                    
                    while (!done) {
                        switch (gst_iterator_next(it, &item)) {
                            case GST_ITERATOR_OK: {
                                GstElement* element = GST_ELEMENT(g_value_get_object(&item));
                                gchar* elemName = gst_element_get_name(element);
                                if (g_str_has_prefix(elemName, "video_enc_tee2_")) {
                                    subTee = GST_ELEMENT(gst_object_ref(element));
                                    done = TRUE;
                                }
                                g_free(elemName);
                                g_value_reset(&item);
                                break;
                            }
                            case GST_ITERATOR_DONE:
                                done = TRUE;
                                break;
                            default:
                                break;
                        }
                    }
                    g_value_unset(&item);
                    gst_iterator_free(it);
                }
                gst_object_unref(encBin);
            }
        }
        
        LOG_INFO("Found tees for camera %d: main=%p, sub=%p", cam, mainTee, subTee);
        
        // 메인 스트림 출력 (Sender + Recorder + Event Recorder)
        for (int stream = 0; stream < maxStreamCount + 2; stream++) {
            auto output = std::make_unique<StreamOutput>(cam, stream, StreamOutput::MAIN_STREAM);
            if (output->init(pipeline_, mainTee, basePort)) {
                outputs_.push_back(std::move(output));
            }
        }
        
        // 서브 스트림 출력 (Sender + Recorder + Event Recorder)
        if (subTee) {
            for (int stream = 0; stream < maxStreamCount + 2; stream++) {
                auto output = std::make_unique<StreamOutput>(cam, stream, StreamOutput::SUB_STREAM);
                if (output->init(pipeline_, subTee, basePort)) {
                    outputs_.push_back(std::move(output));
                }
            }
            gst_object_unref(subTee);
        }
        
        if (mainTee) {
            gst_object_unref(mainTee);
        }
    }
    
    LOG_INFO("Set up %zu stream outputs", outputs_.size());
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