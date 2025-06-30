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
    // pad link 결과를 문자열로 변환하는 헬퍼 함수
    auto padLinkReturnToString = [](GstPadLinkReturn ret) -> const char* {
        switch(ret) {
            case GST_PAD_LINK_OK: return "OK";
            case GST_PAD_LINK_WRONG_HIERARCHY: return "WRONG_HIERARCHY";
            case GST_PAD_LINK_WAS_LINKED: return "WAS_LINKED";
            case GST_PAD_LINK_WRONG_DIRECTION: return "WRONG_DIRECTION";
            case GST_PAD_LINK_NOFORMAT: return "NOFORMAT";
            case GST_PAD_LINK_NOSCHED: return "NOSCHED";
            case GST_PAD_LINK_REFUSED: return "REFUSED";
            default: return "UNKNOWN";
        }
    };
    
    // 카메라별 인코더 생성 및 연결
    for (size_t i = 0; i < cameras_.size(); i++) {
        const CameraConfig& config = config_->getCameraConfig(i);
        
        // 인코더 체인이 config에 있는지 확인
        if (config.encoder.empty()) {
            LOG_WARN("No encoder config for camera %zu", i);
            continue;
        }
        
        // 메인 인코더 체인 생성 (enc)
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
        
        // 파이프라인에 추가
        gst_bin_add(GST_BIN(pipeline_), encBin1);
        
        // 카메라 출력(추론 체인의 출력)과 인코더 연결
        GstElement* cameraOutput = cameras_[i]->getOutputElement();
        
        // Ghost pad를 통한 연결
        GstPad* srcPad = gst_element_get_static_pad(cameraOutput, "src");
        if (!srcPad) {
            LOG_ERROR("Failed to get src pad from camera output %zu", i);
            return false;
        }
        
        GstPad* sinkPad = gst_element_get_static_pad(encBin1, "sink");
        if (!sinkPad) {
            // Ghost pad가 없으면 첫 번째 요소의 sink pad 찾기
            GstIterator* it = gst_bin_iterate_sinks(GST_BIN(encBin1));
            GValue item = G_VALUE_INIT;
            gboolean done = FALSE;
            
            while (!done) {
                switch (gst_iterator_next(it, &item)) {
                    case GST_ITERATOR_OK: {
                        GstElement* element = GST_ELEMENT(g_value_get_object(&item));
                        sinkPad = gst_element_get_static_pad(element, "sink");
                        if (sinkPad) {
                            done = TRUE;
                        }
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
        
        if (!srcPad || !sinkPad) {
            LOG_ERROR("Failed to get pads for linking camera %zu to encoder", i);
            if (srcPad) gst_object_unref(srcPad);
            if (sinkPad) gst_object_unref(sinkPad);
            return false;
        }
        
        GstPadLinkReturn linkRet = gst_pad_link(srcPad, sinkPad);
        if (linkRet != GST_PAD_LINK_OK) {
            LOG_ERROR("Failed to link camera %zu to encoder: %d", i, linkRet);
            gst_object_unref(srcPad);
            gst_object_unref(sinkPad);
            return false;
        }
        
        gst_object_unref(srcPad);
        gst_object_unref(sinkPad);
        
        LOG_INFO("Encoder1 linked for camera %zu", i);
        
        // 서브 인코더 처리 (enc2)
        if (!config.encoder2.empty()) {
            // "video_src_tee0. !" 부분 제거하고 나머지만 파싱
            std::string enc2Config = config.encoder2;
            size_t teePos = enc2Config.find("video_src_tee");
            if (teePos != std::string::npos) {
                size_t exclamPos = enc2Config.find("!", teePos);
                if (exclamPos != std::string::npos) {
                    enc2Config = enc2Config.substr(exclamPos + 1);
                    // 앞의 공백 제거
                    enc2Config.erase(0, enc2Config.find_first_not_of(" "));
                }
            }
            
            GstElement* encBin2 = gst_parse_bin_from_description(
                enc2Config.c_str(), TRUE, &error);
            
            if (error) {
                LOG_ERROR("Failed to create encoder 2 for camera %zu: %s", i, error->message);
                g_error_free(error);
                error = nullptr;
            } else {
                g_snprintf(binName, sizeof(binName), "encoder2_%zu", i);
                gst_element_set_name(encBin2, binName);
                
                // 파이프라인에 추가
                gst_bin_add(GST_BIN(pipeline_), encBin2);
                
                // video_src_tee 찾기
                g_snprintf(binName, sizeof(binName), "video_src_tee%zu", i);
                GstElement* srcTee = nullptr;
                
                // 카메라 소스 element에서 tee 찾기
                GstElement* sourceElement = cameras_[i]->getSourceElement();
                if (sourceElement) {
                    if (GST_IS_BIN(sourceElement)) {
                        srcTee = gst_bin_get_by_name(GST_BIN(sourceElement), binName);
                    } else {
                        // 소스가 bin이 아니면 파이프라인에서 찾기
                        srcTee = gst_bin_get_by_name(GST_BIN(pipeline_), binName);
                    }
                }
                
                if (srcTee) {
                    // Tee의 새 src pad 요청
                    GstPadTemplate* padTemplate = gst_element_class_get_pad_template(
                        GST_ELEMENT_GET_CLASS(srcTee), "src_%u");
                    GstPad* teeSrcPad = gst_element_request_pad(srcTee, padTemplate, nullptr, nullptr);
                    
                    if (teeSrcPad) {
                        // 소스 bin에 ghost pad 추가
                        gchar ghostPadName[64];
                        g_snprintf(ghostPadName, sizeof(ghostPadName), "src_enc2_%zu", i);
                        
                        GstElement* parentBin = GST_ELEMENT(gst_element_get_parent(srcTee));
                        if (parentBin) {
                            GstPad* ghostPad = gst_ghost_pad_new(ghostPadName, teeSrcPad);
                            gst_element_add_pad(parentBin, ghostPad);
                            
                            // enc2의 sink pad 찾기
                            GstPad* enc2SinkPad = gst_element_get_static_pad(encBin2, "sink");
                            if (!enc2SinkPad) {
                                // Ghost pad가 없으면 첫 번째 요소 찾기
                                GstIterator* it = gst_bin_iterate_sinks(GST_BIN(encBin2));
                                GValue item = G_VALUE_INIT;
                                gboolean done = FALSE;
                                
                                while (!done) {
                                    switch (gst_iterator_next(it, &item)) {
                                        case GST_ITERATOR_OK: {
                                            GstElement* element = GST_ELEMENT(g_value_get_object(&item));
                                            enc2SinkPad = gst_element_get_static_pad(element, "sink");
                                            if (enc2SinkPad) {
                                                done = TRUE;
                                            }
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
                            
                            if (ghostPad && enc2SinkPad) {
                                GstPadLinkReturn ret = gst_pad_link(ghostPad, enc2SinkPad);
                                if (ret == GST_PAD_LINK_OK) {
                                    LOG_INFO("Encoder2 linked for camera %zu", i);
                                } else {
                                    LOG_ERROR("Failed to link ghost pad to encoder2 for camera %zu: %s", i, 
                                             padLinkReturnToString(ret));
                                }
                                gst_object_unref(enc2SinkPad);
                            }
                            
                            gst_object_unref(parentBin);
                        }
                        
                        gst_element_release_request_pad(srcTee, teeSrcPad);
                        gst_object_unref(teeSrcPad);
                    }
                    gst_object_unref(srcTee);
                } else {
                    LOG_ERROR("Source tee not found for encoder2 of camera %zu", i);
                }
            }
        }
        
        // 스냅샷 처리
        if (!config.snapshot.empty()) {
            // "video_src_tee0. !" 부분 제거하고 나머지만 파싱
            std::string snapConfig = config.snapshot;
            size_t teePos = snapConfig.find("video_src_tee");
            if (teePos != std::string::npos) {
                size_t exclamPos = snapConfig.find("!", teePos);
                if (exclamPos != std::string::npos) {
                    snapConfig = snapConfig.substr(exclamPos + 1);
                    // 앞의 공백 제거
                    snapConfig.erase(0, snapConfig.find_first_not_of(" "));
                }
            }
            
            GstElement* snapBin = gst_parse_bin_from_description(
                snapConfig.c_str(), TRUE, &error);
            
            if (error) {
                LOG_ERROR("Failed to create snapshot for camera %zu: %s", i, error->message);
                g_error_free(error);
                error = nullptr;
            } else {
                g_snprintf(binName, sizeof(binName), "snapshot_%zu", i);
                gst_element_set_name(snapBin, binName);
                
                // 파이프라인에 추가
                gst_bin_add(GST_BIN(pipeline_), snapBin);
                
                // multifilesink location 설정
                GstElement* filesink = gst_bin_get_by_name(GST_BIN(snapBin), "multifilesink0");
                if (filesink) {
                    std::string location = config_->getSystemConfig().snapshotPath + 
                                         "/snapshot_cam" + std::to_string(i) + "_%05d.jpg";
                    g_object_set(filesink, "location", location.c_str(), nullptr);
                    gst_object_unref(filesink);
                }
                
                // video_src_tee 찾기
                g_snprintf(binName, sizeof(binName), "video_src_tee%zu", i);
                GstElement* srcTee = nullptr;
                
                // 카메라 소스 element에서 tee 찾기
                GstElement* sourceElement = cameras_[i]->getSourceElement();
                if (sourceElement) {
                    if (GST_IS_BIN(sourceElement)) {
                        srcTee = gst_bin_get_by_name(GST_BIN(sourceElement), binName);
                    } else {
                        // 소스가 bin이 아니면 파이프라인에서 찾기
                        srcTee = gst_bin_get_by_name(GST_BIN(pipeline_), binName);
                    }
                }
                
                if (srcTee) {
                    // Tee의 새 src pad 요청
                    GstPadTemplate* padTemplate = gst_element_class_get_pad_template(
                        GST_ELEMENT_GET_CLASS(srcTee), "src_%u");
                    GstPad* teeSrcPad = gst_element_request_pad(srcTee, padTemplate, nullptr, nullptr);
                    
                    if (teeSrcPad) {
                        // 소스 bin에 ghost pad 추가
                        gchar ghostPadName[64];
                        g_snprintf(ghostPadName, sizeof(ghostPadName), "src_snap_%zu", i);
                        
                        GstElement* parentBin = GST_ELEMENT(gst_element_get_parent(srcTee));
                        if (parentBin) {
                            GstPad* ghostPad = gst_ghost_pad_new(ghostPadName, teeSrcPad);
                            gst_element_add_pad(parentBin, ghostPad);
                            
                            // snapshot의 sink pad 찾기
                            GstPad* snapSinkPad = gst_element_get_static_pad(snapBin, "sink");
                            if (!snapSinkPad) {
                                // Ghost pad가 없으면 첫 번째 요소 찾기
                                GstIterator* it = gst_bin_iterate_sinks(GST_BIN(snapBin));
                                GValue item = G_VALUE_INIT;
                                gboolean done = FALSE;
                                
                                while (!done) {
                                    switch (gst_iterator_next(it, &item)) {
                                        case GST_ITERATOR_OK: {
                                            GstElement* element = GST_ELEMENT(g_value_get_object(&item));
                                            snapSinkPad = gst_element_get_static_pad(element, "sink");
                                            if (snapSinkPad) {
                                                done = TRUE;
                                            }
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
                            
                            if (ghostPad && snapSinkPad) {
                                GstPadLinkReturn ret = gst_pad_link(ghostPad, snapSinkPad);
                                if (ret == GST_PAD_LINK_OK) {
                                    LOG_INFO("Snapshot linked for camera %zu", i);
                                } else {
                                    LOG_ERROR("Failed to link ghost pad to snapshot for camera %zu: %s", i,
                                             padLinkReturnToString(ret));
                                }
                                gst_object_unref(snapSinkPad);
                            }
                            
                            gst_object_unref(parentBin);
                        }
                        
                        gst_element_release_request_pad(srcTee, teeSrcPad);
                        gst_object_unref(teeSrcPad);
                    }
                    gst_object_unref(srcTee);
                } else {
                    LOG_ERROR("Source tee not found for snapshot of camera %zu", i);
                }
            }
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