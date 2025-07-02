#include "CameraSource.h"
#include "../detection/DetectionBuffer.h"
#include "../utils/Logger.h"
#include "../utils/DeviceSetting.h"
#include <gst/gst.h>
#include <gst/rtp/gstrtpbuffer.h>
#include <gst/rtp/rtp.h>
#include <nvdsmeta.h>
#include <cmath>
#include <fstream>

CameraSource::CameraSource(CameraType type, int index)
    : type_(type)
    , index_(index)
    , pipeline_(nullptr) {
    
    // 구조체 초기화
    memset(&elements_, 0, sizeof(elements_));
    
    // 검출 버퍼 생성
    detectionBuffer_ = std::make_unique<DetectionBuffer>(type);
    
    LOG_INFO("CameraSource created: %s camera (index=%d)",
             (type == CameraType::RGB) ? "RGB" : "THERMAL", index);
}

CameraSource::~CameraSource() {
    // 동적으로 추가된 피어 출력들 정리
    {
        std::lock_guard<std::mutex> lock(peerOutputsMutex_);
        for (auto& pair : peerOutputs_) {
            // removePeerOutput이 호출되지 않은 경우를 위한 안전장치
            PeerOutput* peerOutput = pair.second.get();
            if (peerOutput) {
                // 패드 연결 해제
                if (peerOutput->teeSrcPad && peerOutput->queueSinkPad) {
                    gst_pad_unlink(peerOutput->teeSrcPad, peerOutput->queueSinkPad);
                }
                
                // Tee request pad 해제
                if (peerOutput->teeSrcPad && elements_.main_tee) {
                    gst_element_release_request_pad(elements_.main_tee, peerOutput->teeSrcPad);
                    gst_object_unref(peerOutput->teeSrcPad);
                }
                
                // Queue sink pad 해제
                if (peerOutput->queueSinkPad) {
                    gst_object_unref(peerOutput->queueSinkPad);
                }
            }
        }
        peerOutputs_.clear();
    }
    
    // 요소들은 파이프라인이 관리하므로 여기서는 unref하지 않음
    // GStreamer는 파이프라인이 NULL 상태가 될 때 자동으로 정리
    
    LOG_INFO("CameraSource destroyed: %s camera (index=%d)",
             (type_ == CameraType::RGB) ? "RGB" : "THERMAL", index_);
}

bool CameraSource::init(const CameraConfig& config, GstElement* pipeline) {
    if (!pipeline) {
        LOG_ERROR("Invalid pipeline");
        return false;
    }
    
    pipeline_ = pipeline;
    config_ = config;
    
    // 1. 소스 체인 생성
    if (!createSourceChain(config)) {
        LOG_ERROR("Failed to create source chain");
        return false;
    }
    
    // 2. 추론 체인 생성 (옵션)
    if (config.inference.enabled) {
        if (!createInferenceChain(config)) {
            LOG_ERROR("Failed to create inference chain");
            return false;
        }
    }
    
    // 3. 요소들 연결 (파이프라인에 추가 및 링크)
    if (!linkElements(config)) {
        LOG_ERROR("Failed to link elements");
        return false;
    }
    
    // 4. 프로브 추가
    if (!addProbes()) {
        LOG_ERROR("Failed to add probes");
        return false;
    }
    
    // 5. 검출기 초기화 (추론이 활성화된 경우)
    if (config.inference.enabled) {
        detector_ = std::make_unique<Detector>(type_);
        
        // 설정 파일 초기화
        if (!detector_->init(config.inference.config_file)) {
            LOG_ERROR("Failed to initialize detector");
            return false;
        }
        
        // 검출 콜백 설정 - DetectionBuffer에 저장
        detector_->setDetectionCallback([this](const DetectionData& detection) {
            detectionBuffer_->addDetection(detection);
            
            // 이벤트 발생 시 추가 처리
            handleDetectionEvent(detection);
        });
        
        // 설정 적용
        auto& settings = DeviceSetting::getInstance().get();
        detector_->setEnabled(settings.analysisStatus);
        detector_->setInterval(settings.nvInterval);
    }

    LOG_INFO("CameraSource initialized: %s camera (inference=%s)",
             (type_ == CameraType::RGB) ? "RGB" : "THERMAL",
             config.inference.enabled ? "enabled" : "disabled");
    
    return true;
}

GstPadProbeReturn CameraSource::osdSinkPadProbe(GstPad* pad, GstPadProbeInfo* info, 
                                                gpointer userData) {
    CameraSource* self = static_cast<CameraSource*>(userData);
        
    if (!self->detector_) {
        return GST_PAD_PROBE_OK;
    }
    
    GstBuffer* buf = GST_PAD_PROBE_INFO_BUFFER(info);
    NvDsBatchMeta* batchMeta = gst_buffer_get_nvds_batch_meta(buf);
    
    if (batchMeta) {
        // 프레임 번호 추출
        uint32_t frameNumber = 0;
        if (batchMeta->frame_meta_list) {
            NvDsFrameMeta* frameMeta = static_cast<NvDsFrameMeta*>(
                batchMeta->frame_meta_list->data);
            frameNumber = frameMeta->frame_num;
        }
        
        // Detector에 위임
        self->detector_->processBatchMeta(batchMeta, frameNumber);
    }
    
    return GST_PAD_PROBE_OK;
}

bool CameraSource::createSourceChain(const CameraConfig& config) {
    gchar elementName[64];
    
    g_snprintf(elementName, sizeof(elementName), "intervideosrc_%d", index_);
    elements_.intervideosrc = gst_element_factory_make("intervideosrc", elementName);

    // 카메라별 채널 설정
    const char* channel_name = (type_ == CameraType::RGB) ? "RGB_Camera" : "Thermal_Camera";
    g_object_set(elements_.intervideosrc, "channel", channel_name, nullptr);
    
    // 컨버터
    elements_.converter1 = gst_element_factory_make("nvvideoconvert", nullptr);
    
    // 클럭 오버레이
    elements_.clockoverlay = gst_element_factory_make("clockoverlay", nullptr);
    g_object_set(elements_.clockoverlay,
                 "time-format", "%D %H:%M:%S",
                 "font-desc", "Arial, 18",
                 nullptr);
    
    // 비디오 레이트
    elements_.videorate = gst_element_factory_make("videorate", nullptr);
    
    // Caps 필터
    elements_.capsfilter = gst_element_factory_make("capsfilter", nullptr);
    GstCaps* caps = gst_caps_new_simple("video/x-raw",
        "width", G_TYPE_INT, config.source.width,
        "height", G_TYPE_INT, config.source.height,
        "framerate", GST_TYPE_FRACTION, config.source.framerate, 1,
        nullptr);
    g_object_set(elements_.capsfilter, "caps", caps, nullptr);
    gst_caps_unref(caps);
    
    // 큐와 Tee
    elements_.queue1 = gst_element_factory_make("queue", nullptr);
    g_object_set(elements_.queue1,
                 "max-size-buffers", 5,
                 "leaky", 2, // downstream
                 nullptr);
    
    elements_.tee = gst_element_factory_make("tee", nullptr);
    g_object_set(elements_.tee, "allow-not-linked", TRUE, nullptr);
    
    return true;
}

bool CameraSource::createInferenceChain(const CameraConfig& config) {
    // g_object_set(elements_.infer,
    //          "config-file-path", config.inference.config_file.c_str(),
    //          "unique-id", index_ + 1,
    //          nullptr);

    // // 파일 존재 확인
    // if (!g_file_test(config.inference.config_file.c_str(), G_FILE_TEST_EXISTS)) {
    //     LOG_ERROR("추론 설정 파일이 없습니다: %s", config.inference.config_file.c_str());
    //     return false;
    // }
    // LOG_INFO("추론 설정 파일 확인됨: %s", config.inference.config_file.c_str());

    // 추론을 위한 체인
    elements_.queue2 = gst_element_factory_make("queue", nullptr);
    if (elements_.queue2) {
        // Queue 속성 설정 (버퍼링 문제일 수 있음)
        g_object_set(elements_.queue2,
                    "max-size-buffers", 30,
                    "max-size-time", 1 * GST_SECOND,
                    "leaky", 2,
                    nullptr);
        LOG_INFO("Queue2 생성 및 설정 완료");
    }
    elements_.videoscale = gst_element_factory_make("videoscale", nullptr);
    elements_.converter2 = gst_element_factory_make("nvvideoconvert", nullptr);
    
    // Mux
    elements_.mux = gst_element_factory_make("nvstreammux", nullptr);
    g_object_set(elements_.mux,
                    "batch-size", 1,
                    "width", config.inference.scale_width,
                    "height", config.inference.scale_height,
                    "live-source", 1,
                    "batched-push-timeout", 33000,  // 40000 -> 33000 (30fps 기준)
                    "enable-padding", 0,
                    nullptr);
    
    // 추론
    elements_.infer = gst_element_factory_make("nvinfer", nullptr);

    g_object_set(elements_.infer,
                "config-file-path", config.inference.config_file.c_str(),
                "unique-id", index_ + 1,
                nullptr);
    
    // 후처리
    elements_.nvof = gst_element_factory_make("nvof", nullptr);
    elements_.converter3 = gst_element_factory_make("nvvideoconvert", nullptr);
    elements_.postproc = gst_element_factory_make("dspostproc", nullptr);
    elements_.osd = gst_element_factory_make("nvdsosd", nullptr);
    elements_.converter4 = gst_element_factory_make("nvvideoconvert", nullptr);
    
    return true;
}

bool CameraSource::linkElements(const CameraConfig& config) {
    if (!pipeline_) {
        LOG_ERROR("Pipeline not set");
        return false;
    }
    
    // ========== 1. 기본 소스 체인 생성 ==========
    gst_bin_add_many(GST_BIN(pipeline_),
        elements_.intervideosrc, elements_.converter1, elements_.clockoverlay,
        elements_.videorate, elements_.capsfilter, elements_.queue1,
        elements_.tee, nullptr);
    
    if (!gst_element_link_many(
            elements_.intervideosrc, elements_.converter1, 
            elements_.clockoverlay, elements_.videorate, 
            elements_.capsfilter, elements_.queue1,
            elements_.tee, nullptr)) {
        LOG_ERROR("Failed to link source chain");
        return false;
    }
    
    // ========== 3. 추론이 활성화된 경우 ==========
    if (config.inference.enabled) {
        LOG_INFO("추론 체인 연결 중... (Camera %d)", index_);
        
        // 3-1. 추론 체인 요소들 파이프라인에 추가
        gst_bin_add_many(GST_BIN(pipeline_),
            elements_.queue2, elements_.videoscale, elements_.converter2,
            elements_.mux, elements_.infer, elements_.nvof,
            elements_.converter3, elements_.postproc, elements_.osd,
            elements_.converter4, nullptr);
        
        // 3-2. Tee → 추론 체인 연결
        GstPadTemplate* tee_template = gst_element_class_get_pad_template(
            GST_ELEMENT_GET_CLASS(elements_.tee), "src_%u");
            
        GstPad* tee_pad = gst_element_request_pad(elements_.tee, tee_template, nullptr, nullptr);
        GstPad* queue_pad = gst_element_get_static_pad(elements_.queue2, "sink");
        
        if (gst_pad_link(tee_pad, queue_pad) != GST_PAD_LINK_OK) {
            LOG_ERROR("Failed to link tee to inference queue");
            gst_object_unref(queue_pad);
            return false;
        }
        gst_object_unref(queue_pad);
        
        // 3-3. 추론 체인 내부 연결
        if (!gst_element_link(elements_.queue2, elements_.videoscale)) {
            LOG_ERROR("Failed to link queue2 to videoscale");
            return false;
        }
        
        GstCaps* scale_caps = gst_caps_new_simple("video/x-raw",
            "width", G_TYPE_INT, config.inference.scale_width,
            "height", G_TYPE_INT, config.inference.scale_height, nullptr);
        
        if (!gst_element_link_filtered(elements_.videoscale, elements_.converter2, scale_caps)) {
            LOG_ERROR("Failed to link videoscale to converter2");
            gst_caps_unref(scale_caps);
            return false;
        }
        gst_caps_unref(scale_caps);
        
        GstPad* conv_pad = gst_element_get_static_pad(elements_.converter2, "src");
        GstPad* mux_pad = gst_element_get_request_pad(elements_.mux, "sink_0");
        
        if (gst_pad_link(conv_pad, mux_pad) != GST_PAD_LINK_OK) {
            LOG_ERROR("Failed to link converter to mux");
            gst_object_unref(conv_pad);
            return false;
        }
        gst_object_unref(conv_pad);
        
        if (!gst_element_link_many(elements_.mux, elements_.infer,
                                  elements_.converter3, elements_.postproc,
                                  elements_.osd, elements_.converter4, nullptr)) {
            LOG_ERROR("Failed to link inference chain");
            return false;
        }
        
        // 3-4. Main Tee 생성 및 연결
        char main_tee_name[32];
        snprintf(main_tee_name, sizeof(main_tee_name), "main_tee_%d", index_);
        
        GstElement* main_tee = gst_element_factory_make("tee", main_tee_name);
        g_object_set(main_tee, "allow-not-linked", TRUE, NULL);
        gst_bin_add(GST_BIN(pipeline_), main_tee);
        
        gst_element_link(elements_.converter4, main_tee);
        
        // // 3-5. WebRTC Inter 출력 (Raw Video)
        // char webrtc_queue_name[32], webrtc_conv_name[32], webrtc_sink_name[32];
        // snprintf(webrtc_queue_name, sizeof(webrtc_queue_name), "webrtc_queue_%d", index_);
        // snprintf(webrtc_conv_name, sizeof(webrtc_conv_name), "webrtc_conv_%d", index_);
        // snprintf(webrtc_sink_name, sizeof(webrtc_sink_name), "webrtc_sink_%d", index_);

        // GstElement* webrtc_queue = gst_element_factory_make("queue", webrtc_queue_name);
        // GstElement* webrtc_conv = gst_element_factory_make("nvvideoconvert", webrtc_conv_name);
        // GstElement* webrtc_sink = gst_element_factory_make("intervideosink", webrtc_sink_name);

        // const char* webrtc_channel = (type_ == CameraType::RGB) ? "Webrtc_RGB_Camera" : "Webrtc_Thermal_Camera";
        // g_object_set(webrtc_sink, "channel", webrtc_channel, nullptr);
        // g_object_set(webrtc_queue, "max-size-buffers", 5, "leaky", 2, nullptr);

        // gst_bin_add_many(GST_BIN(pipeline_), webrtc_queue, webrtc_conv, webrtc_sink, nullptr);

        // GstPad* tee_webrtc_pad = gst_element_get_request_pad(main_tee, "src_%u");
        // GstPad* webrtc_queue_sink = gst_element_get_static_pad(webrtc_queue, "sink");
        // gst_pad_link(tee_webrtc_pad, webrtc_queue_sink);
        // gst_object_unref(tee_webrtc_pad);
        // gst_object_unref(webrtc_queue_sink);

        // gst_element_link_many(webrtc_queue, webrtc_conv, webrtc_sink, nullptr);
        // LOG_INFO("WebRTC Inter 출력 추가 완료: channel=%s", webrtc_channel);
        
        // 3-6. Fakesink (파이프라인 안정성용)
        char fakesink_queue_name[32], fakesink_name[32];
        snprintf(fakesink_queue_name, sizeof(fakesink_queue_name), "fakesink_queue_%d", index_);
        snprintf(fakesink_name, sizeof(fakesink_name), "fakesink_%d", index_);
        
        GstElement* fakesink_queue = gst_element_factory_make("queue", fakesink_queue_name);
        GstElement* fakesink = gst_element_factory_make("fakesink", fakesink_name);
        
        g_object_set(fakesink_queue, "max-size-buffers", 1, "leaky", 2, NULL);
        g_object_set(fakesink, "sync", FALSE, NULL);
        
        gst_bin_add_many(GST_BIN(pipeline_), fakesink_queue, fakesink, nullptr);
        
        GstPad* tee_fake_pad = gst_element_get_request_pad(main_tee, "src_%u");
        GstPad* fake_queue_sink = gst_element_get_static_pad(fakesink_queue, "sink");
        gst_pad_link(tee_fake_pad, fake_queue_sink);
        gst_object_unref(tee_fake_pad);
        gst_object_unref(fake_queue_sink);
        
        gst_element_link(fakesink_queue, fakesink);

        elements_.main_tee = main_tee;
        LOG_INFO("추론 체인 연결 완료");
    }
    
    LOG_INFO("All elements linked successfully for %s camera",
             (type_ == CameraType::RGB) ? "RGB" : "THERMAL");
    
    return true;
}

bool CameraSource::addProbes() {
    // 1. 추론 직후 프레임 로깅 (nvinfer src pad)
    // if (elements_.infer) {
    //     GstPad* inferSrcPad = gst_element_get_static_pad(elements_.infer, "src");
    //     if (inferSrcPad) {
    //         gst_pad_add_probe(inferSrcPad, GST_PAD_PROBE_TYPE_BUFFER,
    //             [](GstPad* pad, GstPadProbeInfo* info, gpointer data) -> GstPadProbeReturn {
    //                 CameraSource* self = static_cast<CameraSource*>(data);
    //                 static guint64 frameCount[2] = {0, 0};
                    
    //                 GstBuffer* buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    //                 frameCount[self->index_]++;
                    
    //                 // 버퍼 정보
    //                 GstClockTime pts = GST_BUFFER_PTS(buffer);
    //                 GstClockTime dts = GST_BUFFER_DTS(buffer);
    //                 GstClockTime duration = GST_BUFFER_DURATION(buffer);
                    
    //                 // 메타데이터 확인
    //                 NvDsBatchMeta* batchMeta = gst_buffer_get_nvds_batch_meta(buffer);
    //                 int objectCount = 0;
    //                 uint32_t frameNum = 0;
                    
    //                 if (batchMeta && batchMeta->frame_meta_list) {
    //                     NvDsFrameMeta* frameMeta = (NvDsFrameMeta*)batchMeta->frame_meta_list->data;
    //                     frameNum = frameMeta->frame_num;
                        
    //                     // 객체 수 카운트
    //                     for (NvDsMetaList* l = frameMeta->obj_meta_list; l != nullptr; l = l->next) {
    //                         objectCount++;
    //                     }
    //                 }
                    
    //                 LOG_INFO("[%s] INFER_OUT Frame #%llu (frame_num=%u): PTS=%.3f, Objects=%d",
    //                          (self->type_ == CameraType::RGB) ? "RGB" : "THERMAL",
    //                          frameCount[self->index_],
    //                          frameNum,
    //                          pts / 1000000000.0,  // 나노초를 초로 변환
    //                          objectCount);
                    
    //                 return GST_PAD_PROBE_OK;
    //             }, this, nullptr);
    //         gst_object_unref(inferSrcPad);
    //     }
    // }

    if (elements_.main_tee) {
        GstPad* pad = gst_element_get_static_pad(elements_.main_tee, "src_0");
        if (!pad) {
            GstIterator* it = gst_element_iterate_src_pads(elements_.main_tee);
            GValue item = G_VALUE_INIT;
            if (gst_iterator_next(it, &item) == GST_ITERATOR_OK) {
                pad = GST_PAD(g_value_get_object(&item));
                gst_object_ref(pad);
                g_value_reset(&item);
            }
            gst_iterator_free(it);
        }
        
        if (pad) {
            gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER,
                [](GstPad* pad, GstPadProbeInfo* info, gpointer data) -> GstPadProbeReturn {
                    CameraSource* self = static_cast<CameraSource*>(data);
                    static int counter[2] = {0, 0};
                    static bool caps_printed[2] = {false, false};
                    
                    counter[self->index_]++;
                    
                    // 처음 한 번만 Caps 정보 출력
                    if (!caps_printed[self->index_]) {
                        GstCaps* caps = gst_pad_get_current_caps(pad);
                        if (caps) {
                            gchar* caps_str = gst_caps_to_string(caps);
                            LOG_INFO("=== Camera %d (%s) Caps ===", 
                                    self->index_,
                                    (self->type_ == CameraType::RGB) ? "RGB" : "THERMAL");
                            LOG_INFO("%s", caps_str);
                            
                            // 구조체에서 상세 정보 추출
                            GstStructure* structure = gst_caps_get_structure(caps, 0);
                            if (structure) {
                                const gchar* format = gst_structure_get_string(structure, "format");
                                gint width, height;
                                gint fps_n, fps_d;
                                
                                if (gst_structure_get_int(structure, "width", &width) &&
                                    gst_structure_get_int(structure, "height", &height)) {
                                    LOG_INFO("Resolution: %dx%d", width, height);
                                }
                                
                                if (gst_structure_get_fraction(structure, "framerate", &fps_n, &fps_d)) {
                                    LOG_INFO("Framerate: %d/%d = %.2f fps", fps_n, fps_d, 
                                            (float)fps_n / fps_d);
                                }
                                
                                if (format) {
                                    LOG_INFO("Format: %s", format);
                                }
                                
                                // 이미지 크기가 예상과 맞는지 확인
                                if (self->type_ == CameraType::RGB) {
                                    if (width != 1920 || height != 1080) {
                                        LOG_WARN("RGB camera unexpected resolution: %dx%d (expected 1920x1080)", 
                                                width, height);
                                    }
                                } else {  // THERMAL
                                    if (width != 384 || height != 288) {
                                        LOG_WARN("Thermal camera unexpected resolution: %dx%d (expected 384x288)", 
                                                width, height);
                                    }
                                }
                            }
                            LOG_INFO("========================");
                            
                            g_free(caps_str);
                            gst_caps_unref(caps);
                            caps_printed[self->index_] = true;
                        }
                    }
                    
                    
                    if(self->index_ == 0) {
                        // RGB 카메라의 경우
                        // LOG_INFO("main_tee_%d output: %d frames", 
                        //         self->index_, counter[self->index_]);
                    } else {
                        // // Thermal 카메라의 경우
                        // LOG_INFO("main_tee_%d output: %d frames", 
                        //         self->index_, counter[self->index_]);
                    }
                    
                    return GST_PAD_PROBE_OK;
                }, this, nullptr);
            gst_object_unref(pad);
        }
    }
    
    return true;
}

bool CameraSource::addPeerOutput(const std::string& peerId) {
    std::lock_guard<std::mutex> lock(peerOutputsMutex_);
    
    if (peerOutputs_.find(peerId) != peerOutputs_.end()) {
        LOG_WARN("WebRTC output already exists for peer: %s", peerId.c_str());
        return false;
    }
    
    if (!elements_.main_tee) {
        LOG_ERROR("Main tee not available");
        return false;
    }
    
    // 1. 파이프라인 현재 상태 확인
    GstState current_state;
    GstStateChangeReturn state_ret = gst_element_get_state(pipeline_, &current_state, nullptr, 0);
    LOG_INFO("Pipeline current state: %s", gst_element_state_get_name(current_state));
    
    // 2. 요소 생성
    auto peerOutput = std::make_unique<PeerOutput>();
    peerOutput->peerId = peerId;
    
    char queueName[64], convName[64], sinkName[64];
    snprintf(queueName, sizeof(queueName), "webrtc_queue_%s_%d", peerId.c_str(), index_);
    snprintf(convName, sizeof(convName), "webrtc_conv_%s_%d", peerId.c_str(), index_);
    snprintf(sinkName, sizeof(sinkName), "webrtc_sink_%s_%d", peerId.c_str(), index_);
    
    peerOutput->queue = gst_element_factory_make("queue", queueName);
    peerOutput->converter = gst_element_factory_make("nvvideoconvert", convName);
    peerOutput->intervideosink = gst_element_factory_make("intervideosink", sinkName);
    
    if (!peerOutput->queue || !peerOutput->converter || !peerOutput->intervideosink) {
        LOG_ERROR("Failed to create elements for peer %s", peerId.c_str());
        if (peerOutput->queue) gst_object_unref(peerOutput->queue);
        if (peerOutput->converter) gst_object_unref(peerOutput->converter);
        if (peerOutput->intervideosink) gst_object_unref(peerOutput->intervideosink);
        return false;
    }
    
    // 3. 속성 설정
    g_object_set(peerOutput->queue,
                 "max-size-buffers", 5,
                 "max-size-time", 1 * GST_SECOND,
                 "leaky", 2,  // downstream
                 nullptr);
    
    std::string webrtc_channel = "Webrtc_" + 
        std::string(type_ == CameraType::RGB ? "RGB" : "Thermal") + 
        "_" + peerId;
    
    g_object_set(peerOutput->intervideosink,
                 "channel", webrtc_channel.c_str(),
                 nullptr);
    
    LOG_INFO("Creating dynamic output with channel: %s", webrtc_channel.c_str());
    
    // 4. 파이프라인에 요소 추가 (PAUSED 상태로)
    gst_element_set_state(peerOutput->queue, GST_STATE_READY);
    gst_element_set_state(peerOutput->converter, GST_STATE_READY);
    gst_element_set_state(peerOutput->intervideosink, GST_STATE_READY);
    
    if (!gst_bin_add(GST_BIN(pipeline_), peerOutput->queue)) {
        LOG_ERROR("Failed to add queue to pipeline");
        return false;
    }
    if (!gst_bin_add(GST_BIN(pipeline_), peerOutput->converter)) {
        LOG_ERROR("Failed to add converter to pipeline");
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->queue);
        return false;
    }
    if (!gst_bin_add(GST_BIN(pipeline_), peerOutput->intervideosink)) {
        LOG_ERROR("Failed to add intervideosink to pipeline");
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->queue);
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->converter);
        return false;
    }
    
    // 5. 요소들 연결
    if (!gst_element_link(peerOutput->queue, peerOutput->converter)) {
        LOG_ERROR("Failed to link queue to converter");
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->queue);
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->converter);
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->intervideosink);
        return false;
    }
    
    if (!gst_element_link(peerOutput->converter, peerOutput->intervideosink)) {
        LOG_ERROR("Failed to link converter to intervideosink");
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->queue);
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->converter);
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->intervideosink);
        return false;
    }
    
    // 6. Tee와 연결
    GstPadTemplate* padTemplate = gst_element_class_get_pad_template(
        GST_ELEMENT_GET_CLASS(elements_.main_tee), "src_%u");
    
    if (!padTemplate) {
        LOG_ERROR("Failed to get tee pad template");
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->queue);
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->converter);
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->intervideosink);
        return false;
    }
    
    peerOutput->teeSrcPad = gst_element_request_pad(elements_.main_tee, padTemplate, nullptr, nullptr);
    if (!peerOutput->teeSrcPad) {
        LOG_ERROR("Failed to request tee src pad");
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->queue);
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->converter);
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->intervideosink);
        return false;
    }
    
    peerOutput->queueSinkPad = gst_element_get_static_pad(peerOutput->queue, "sink");
    if (!peerOutput->queueSinkPad) {
        LOG_ERROR("Failed to get queue sink pad");
        gst_element_release_request_pad(elements_.main_tee, peerOutput->teeSrcPad);
        gst_object_unref(peerOutput->teeSrcPad);
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->queue);
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->converter);
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->intervideosink);
        return false;
    }
    
    // 7. 패드 연결
    GstPadLinkReturn linkResult = gst_pad_link(peerOutput->teeSrcPad, peerOutput->queueSinkPad);
    if (linkResult != GST_PAD_LINK_OK) {
        LOG_ERROR("Failed to link tee to queue: %s (%d)", 
                  gst_pad_link_get_name(linkResult), linkResult);
        
        // 디버깅 정보
        GstCaps* tee_caps = gst_pad_query_caps(peerOutput->teeSrcPad, nullptr);
        GstCaps* queue_caps = gst_pad_query_caps(peerOutput->queueSinkPad, nullptr);
        
        gchar* tee_caps_str = gst_caps_to_string(tee_caps);
        gchar* queue_caps_str = gst_caps_to_string(queue_caps);
        
        LOG_ERROR("Tee src caps: %s", tee_caps_str);
        LOG_ERROR("Queue sink caps: %s", queue_caps_str);
        
        g_free(tee_caps_str);
        g_free(queue_caps_str);
        gst_caps_unref(tee_caps);
        gst_caps_unref(queue_caps);
        
        gst_element_release_request_pad(elements_.main_tee, peerOutput->teeSrcPad);
        gst_object_unref(peerOutput->teeSrcPad);
        gst_object_unref(peerOutput->queueSinkPad);
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->queue);
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->converter);
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->intervideosink);
        return false;
    }
    
    // 8. 상태 동기화
    if (current_state >= GST_STATE_PAUSED) {
        LOG_INFO("Syncing state with parent (state: %s)", gst_element_state_get_name(current_state));
        
        // 순차적으로 상태 변경
        gst_element_set_state(peerOutput->queue, GST_STATE_PAUSED);
        gst_element_set_state(peerOutput->converter, GST_STATE_PAUSED);
        gst_element_set_state(peerOutput->intervideosink, GST_STATE_PAUSED);
        
        if (current_state == GST_STATE_PLAYING) {
            gst_element_set_state(peerOutput->queue, GST_STATE_PLAYING);
            gst_element_set_state(peerOutput->converter, GST_STATE_PLAYING);
            gst_element_set_state(peerOutput->intervideosink, GST_STATE_PLAYING);
        }
    }

    // 10. 연결 확인
    GstStateChangeReturn ret = gst_element_get_state(peerOutput->intervideosink, 
                                                     nullptr, nullptr, GST_SECOND);
    if (ret == GST_STATE_CHANGE_SUCCESS) {
        LOG_INFO("InterVideoSink is in correct state");
    } else {
        LOG_WARN("InterVideoSink state change result: %d", ret);
    }
    
    // 9. 저장
    peerOutputs_[peerId] = std::move(peerOutput);
    
    LOG_INFO("Successfully added WebRTC output for camera %d, peer %s: channel=%s", 
             index_, peerId.c_str(), webrtc_channel.c_str());
    
    
    
    return true;
}

// 제거도 패드 블로킹 사용
bool CameraSource::removePeerOutput(const std::string& peerId) {
    std::lock_guard<std::mutex> lock(peerOutputsMutex_);
    
    auto it = peerOutputs_.find(peerId);
    if (it == peerOutputs_.end()) {
        LOG_WARN("Peer output not found: %s", peerId.c_str());
        return false;
    }
    
    PeerOutput* peerOutput = it->second.get();
    
    LOG_INFO("Removing peer output for camera %d: peer=%s", index_, peerId.c_str());
    
    // 1. Queue의 sink 패드를 블록
    if (peerOutput->queueSinkPad) {
        struct RemoveData {
            PeerOutput* output;
            GstElement* pipeline;
            GstElement* tee;
            bool completed;
        };
        
        RemoveData removeData = {peerOutput, pipeline_, elements_.main_tee, false};
        
        gulong probeId = gst_pad_add_probe(peerOutput->queueSinkPad,
            GST_PAD_PROBE_TYPE_BLOCK_DOWNSTREAM,
            [](GstPad* pad, GstPadProbeInfo* info, gpointer userData) -> GstPadProbeReturn {
                RemoveData* data = static_cast<RemoveData*>(userData);
                
                LOG_DEBUG("Pad blocked for removal");
                
                // 패드 연결 해제
                if (data->output->teeSrcPad && data->output->queueSinkPad) {
                    gst_pad_unlink(data->output->teeSrcPad, data->output->queueSinkPad);
                }
                
                // 요소들을 NULL 상태로
                gst_element_set_state(data->output->queue, GST_STATE_NULL);
                gst_element_set_state(data->output->converter, GST_STATE_NULL);
                gst_element_set_state(data->output->intervideosink, GST_STATE_NULL);
                
                // 파이프라인에서 제거
                gst_bin_remove(GST_BIN(data->pipeline), data->output->queue);
                gst_bin_remove(GST_BIN(data->pipeline), data->output->converter);
                gst_bin_remove(GST_BIN(data->pipeline), data->output->intervideosink);
                
                // Tee request pad 해제
                if (data->output->teeSrcPad && data->tee) {
                    gst_element_release_request_pad(data->tee, data->output->teeSrcPad);
                    gst_object_unref(data->output->teeSrcPad);
                }
                
                data->completed = true;
                return GST_PAD_PROBE_REMOVE;
            },
            &removeData,
            nullptr);
        
        // 블록 처리 대기
        g_usleep(50000);  // 50ms
        
        if (peerOutput->queueSinkPad) {
            gst_object_unref(peerOutput->queueSinkPad);
        }
    }
    
    // 2. 맵에서 제거
    peerOutputs_.erase(it);
    
    LOG_INFO("Successfully removed peer output for camera %d: peer=%s", index_, peerId.c_str());
    return true;
}

void CameraSource::handleDetectionEvent(const DetectionData& detection) {
    // 중요 이벤트 처리
    for (const auto& obj : detection.objects) {
        switch (obj.classId) {
            case CLASS_LABOR_SIGN_COW:
                LOG_WARN("분만 징후 감지! Camera: %s, Frame: %u",
                        (type_ == CameraType::RGB) ? "RGB" : "THERMAL",
                        detection.frameNumber);
                // TODO: 알림 전송
                break;
                
            case CLASS_FLIP_COW:
                if (obj.color == BboxColor::RED) {
                    LOG_WARN("전도 소 확정! Camera: %s, Frame: %u",
                            (type_ == CameraType::RGB) ? "RGB" : "THERMAL",
                            detection.frameNumber);
                    // TODO: 긴급 알림
                }
                break;
                
            case CLASS_HEAT_COW:
                if (obj.color == BboxColor::RED) {
                    LOG_INFO("발정 소 확정! Camera: %s, Frame: %u",
                            (type_ == CameraType::RGB) ? "RGB" : "THERMAL",
                            detection.frameNumber);
                    // TODO: 기록 및 알림
                }
                break;
        }
    }
}