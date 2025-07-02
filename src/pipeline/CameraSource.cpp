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
    , source_(nullptr)
    , decoder_(nullptr)
    , converter_(nullptr)
    , clockOverlay_(nullptr)
    , tee_(nullptr)
    , queue_(nullptr)
    , scale_(nullptr)
    , mux_(nullptr)
    , infer_(nullptr)
    , tracker_(nullptr)
    , postproc_(nullptr)
    , osd_(nullptr)
    , outputConverter_(nullptr) {
    
    // 검출 버퍼 생성
    detectionBuffer_ = std::make_unique<DetectionBuffer>(type);
    
    LOG_INFO("CameraSource created: %s camera (index=%d)",
             (type == CameraType::RGB) ? "RGB" : "THERMAL", index);
}

CameraSource::~CameraSource() {
    // 요소들은 파이프라인이 관리하므로 여기서는 unref하지 않음
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
    
    // 3. 인코더 체인 생성
    if (!createEncoderChain(config)) {
        LOG_ERROR("Failed to create encoder chain");
        return false;
    }
    
    // 4. 요소들 연결 (파이프라인에 추가 및 링크)
    if (!linkElements(config)) {
        LOG_ERROR("Failed to link elements");
        return false;
    }
    
    // 5. 프로브 추가
    if (!addProbes()) {
        LOG_ERROR("Failed to add probes");
        return false;
    }
    
    // 6. 외부 연결을 위한 참조 설정
    tee_ = elements_.tee;  // Pipeline에서 접근할 수 있도록
    outputConverter_ = elements_.converter4 ? elements_.converter4 : elements_.sink;
    
    detector_ = std::make_unique<Detector>(type_);
    if (config.inference.enabled) {
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

    LOG_INFO("CameraSource initialized: %s camera",
             (type_ == CameraType::RGB) ? "RGB" : "THERMAL");
    
    return true;
}

bool CameraSource::link() {
    // 이미 내부 연결은 완료됨
    // 외부 연결은 Pipeline에서 처리
    return true;
}

GstElement* CameraSource::getSourceElement() const {
    return source_;
}

GstElement* CameraSource::getTeeElement() const {
    return tee_;
}

GstElement* CameraSource::getOutputElement() const {
    return outputConverter_;
}

DetectionBuffer* CameraSource::getDetectionBuffer() const {
    return detectionBuffer_.get();
}

bool CameraSource::parseAndCreateSource(const std::string& sourceStr) {
    // 전체 소스 체인을 파싱
    GError* error = nullptr;
    GstElement* bin = gst_parse_bin_from_description(sourceStr.c_str(), TRUE, &error);
    
    if (error) {
        LOG_ERROR("Failed to parse source: %s", error->message);
        g_error_free(error);
        return false;
    }
    
    source_ = bin;
    return true;
}

bool CameraSource::addElementsToPipeline(GstElement* pipeline) {
    // 소스와 추론 체인 추가
    gst_bin_add_many(GST_BIN(pipeline), source_, outputConverter_, nullptr);
    
    return true;
}

bool CameraSource::linkInternalElements() {
    // 소스 체인과 추론 체인은 이미 내부적으로 연결되어 있음
    // Tee와 추론 체인만 연결하면 됨
    
    // Tee의 src pad와 추론 체인의 sink pad를 연결
    // 이미 config에서 "video_src_tee0. ! queue ! ..." 형태로 연결되어 있음
    
    LOG_DEBUG("Internal elements already linked through parse");
    
    return true;
}

bool CameraSource::setupProbes() {
    // 2. Tee 이후 프로브
    if (tee_) {
        GstPad* srcPad = gst_element_get_static_pad(tee_, "src_0");
        if (!srcPad) {
            // Request pad 시도
            GstPadTemplate* padTemplate = gst_element_class_get_pad_template(
                GST_ELEMENT_GET_CLASS(tee_), "src_%u");
            if (padTemplate) {
                srcPad = gst_element_request_pad(tee_, padTemplate, nullptr, nullptr);
            }
        }
        
        if (srcPad) {
            gst_pad_add_probe(srcPad, GST_PAD_PROBE_TYPE_BUFFER,
                [](GstPad* pad, GstPadProbeInfo* info, gpointer data) -> GstPadProbeReturn {
                    CameraSource* self = static_cast<CameraSource*>(data);
                    static guint64 count[2] = {0, 0};
                    count[self->index_]++;
                    if (count[self->index_] % 30 == 0) {
                        LOG_INFO("[%s] Tee 이후: %llu 프레임", 
                                (self->type_ == CameraType::RGB) ? "RGB" : "THERMAL",
                                count[self->index_]);
                    }
                    return GST_PAD_PROBE_OK;
                }, this, nullptr);
        }
    }
    
    // 3. Mux 이후 프로브
    if (mux_) {
        GstPad* srcPad = gst_element_get_static_pad(mux_, "src");
        if (srcPad) {
            gst_pad_add_probe(srcPad, GST_PAD_PROBE_TYPE_BUFFER,
                [](GstPad* pad, GstPadProbeInfo* info, gpointer data) -> GstPadProbeReturn {
                    CameraSource* self = static_cast<CameraSource*>(data);
                    static guint64 count[2] = {0, 0};
                    count[self->index_]++;
                    if (count[self->index_] % 30 == 0) {
                        LOG_INFO("[%s] Mux 이후: %llu 프레임", 
                                (self->type_ == CameraType::RGB) ? "RGB" : "THERMAL",
                                count[self->index_]);
                    }
                    return GST_PAD_PROBE_OK;
                }, this, nullptr);
            gst_object_unref(srcPad);
        }
    }
    
    // 4. 추론 이후 프로브
    if (infer_) {
        GstPad* srcPad = gst_element_get_static_pad(infer_, "src");
        if (srcPad) {
            gst_pad_add_probe(srcPad, GST_PAD_PROBE_TYPE_BUFFER,
                [](GstPad* pad, GstPadProbeInfo* info, gpointer data) -> GstPadProbeReturn {
                    CameraSource* self = static_cast<CameraSource*>(data);
                    static guint64 count[2] = {0, 0};
                    count[self->index_]++;
                    if (count[self->index_] % 30 == 0) {
                        LOG_INFO("[%s] 추론 이후: %llu 프레임", 
                                (self->type_ == CameraType::RGB) ? "RGB" : "THERMAL",
                                count[self->index_]);
                    }
                    return GST_PAD_PROBE_OK;
                }, this, nullptr);
            gst_object_unref(srcPad);
        }
    }
    
    // 5. 기존 OSD 프로브
    GstPad* osdSinkPad = gst_element_get_static_pad(osd_, "sink");
    if (!osdSinkPad) {
        LOG_ERROR("Failed to get OSD sink pad");
        return false;
    }
    
    gst_pad_add_probe(osdSinkPad, GST_PAD_PROBE_TYPE_BUFFER,
                      CameraSource::osdSinkPadProbe, this, nullptr);
    
    gst_object_unref(osdSinkPad);
    
    LOG_DEBUG("All probes set up");
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

BboxColor CameraSource::determineObjectColor(NvDsObjectMeta* objMeta) {
    // 대각선 길이 계산
    float diagonal = std::sqrt(objMeta->rect_params.width * objMeta->rect_params.width + 
                              objMeta->rect_params.height * objMeta->rect_params.height);
    
    // 크기 필터링
    float minDiag = 40.0f;
    float maxDiag = 1000.0f;
    
    if (diagonal < minDiag || diagonal > maxDiag) {
        return BboxColor::NONE;
    }
    
    // 클래스별 색상
    auto& settings = DeviceSetting::getInstance().get();
    
    switch (objMeta->class_id) {
        case CLASS_NORMAL_COW:
        case CLASS_NORMAL_COW_SITTING:
            return BboxColor::GREEN;
            
        case CLASS_HEAT_COW:
            if (settings.resnet50Apply) {
                // 추가 조건 확인 필요 시
                return BboxColor::RED;
            }
            return BboxColor::YELLOW;
            
        case CLASS_FLIP_COW:
            if (settings.optFlowApply) {
                // 추가 조건 확인 필요 시
                return BboxColor::RED;
            }
            return BboxColor::YELLOW;
            
        case CLASS_LABOR_SIGN_COW:
            return BboxColor::RED;
            
        case CLASS_OVER_TEMP:
            return BboxColor::BLUE;
            
        default:
            return BboxColor::GREEN;
    }
}

bool CameraSource::createPipeline(const CameraConfig& config) {
    // 1. 소스 체인 생성
    if (!createSourceChain(config)) {
        return false;
    }
    
    // 2. 추론 체인 생성 (옵션)
    if (config.inference.enabled) {
        if (!createInferenceChain(config)) {
            return false;
        }
    }
    
    // 3. 인코더 체인 생성
    if (!createEncoderChain(config)) {
        return false;
    }
    
    // 4. 요소들 연결
    if (!linkElements(config)) {
        return false;
    }
    
    return true;
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

bool CameraSource::createEncoderChain(const CameraConfig& config) {
    // 추론이 활성화되지 않은 경우에만 원본 영상 인코딩
    if (!config.inference.enabled) {
        gchar elementName[64];
        
        // Queue 생성
        g_snprintf(elementName, sizeof(elementName), "encoder_queue_%d", index_);
        elements_.queue3 = gst_element_factory_make("queue", elementName);
        if (!elements_.queue3) {
            LOG_ERROR("Failed to create encoder queue");
            return false;
        }
        
        // nvvideoconvert for memory conversion
        g_snprintf(elementName, sizeof(elementName), "encoder_convert_%d", index_);
        elements_.encoder_convert = gst_element_factory_make("nvvideoconvert", elementName);
        if (!elements_.encoder_convert) {
            LOG_ERROR("Failed to create encoder converter");
            return false;
        }
        
        // 인코더 생성
        const char* encoder_factory = nullptr;
        if (config.encoder.codec == "h264") {
            encoder_factory = "nvv4l2h264enc";
        } else if (config.encoder.codec == "h265") {
            encoder_factory = "nvv4l2h265enc";
        } else {
            LOG_ERROR("Unknown codec: %s", config.encoder.codec.c_str());
            return false;
        }
        
        g_snprintf(elementName, sizeof(elementName), "encoder_%d", index_);
        elements_.encoder = gst_element_factory_make(encoder_factory, elementName);
        if (!elements_.encoder) {
            LOG_ERROR("Failed to create encoder: %s", encoder_factory);
            return false;
        }
        
        // 인코더 설정
        g_object_set(elements_.encoder,
                     "preset-level", 1, // FastPreset
                     "idrinterval", config.encoder.idr_interval,
                     "bitrate", config.encoder.bitrate,
                     "maxperf-enable", TRUE,
                     nullptr);
        
        // RTP Payloader 생성
        const char* payloader_factory = nullptr;
        if (config.encoder.codec == "h264") {
            payloader_factory = "rtph264pay";
        } else if (config.encoder.codec == "h265") {
            payloader_factory = "rtph265pay";
        }
        
        g_snprintf(elementName, sizeof(elementName), "payloader_%d", index_);
        elements_.payloader = gst_element_factory_make(payloader_factory, elementName);
        if (!elements_.payloader) {
            LOG_ERROR("Failed to create payloader: %s", payloader_factory);
            return false;
        }
        
        g_object_set(elements_.payloader,
                     "pt", 96,
                     "config-interval", 1,
                     nullptr);
        
        // 출력 큐
        g_snprintf(elementName, sizeof(elementName), "output_queue_%d", index_);
        elements_.queue4 = gst_element_factory_make("queue", elementName);
        if (!elements_.queue4) {
            LOG_ERROR("Failed to create output queue");
            return false;
        }
        
        // 출력 싱크
        g_snprintf(elementName, sizeof(elementName), "udp_output_%d", index_);
        elements_.sink = gst_element_factory_make("udpsink", elementName);
        if (!elements_.sink) {
            LOG_ERROR("Failed to create udpsink");
            return false;
        }
        
        g_object_set(elements_.sink,
                     "host", "127.0.0.1",
                    "port", 5000 + index_,
                    "sync", FALSE,
                    "async", FALSE,
                    "buffer-size", 2097152,  // 2MB 버퍼 추가
                     nullptr);
        
        LOG_INFO("Encoder chain created for camera %d (no inference): codec=%s, bitrate=%d, port=%d",
                 index_, config.encoder.codec.c_str(), config.encoder.bitrate, 5000 + index_);
    }
    
    // 추론이 활성화된 경우 - 추론 후 영상을 인코딩할 엘리먼트들은 나중에 추가
    // (main_tee 이후에 연결될 예정)
    
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
    
    // ========== 2. 추론이 비활성화된 경우 ==========
    if (!config.inference.enabled && elements_.encoder) {
        gst_bin_add_many(GST_BIN(pipeline_),
            elements_.queue3, elements_.encoder_convert, elements_.encoder, 
            elements_.payloader, elements_.queue4, elements_.sink, nullptr);
        
        GstPadTemplate* tee_template = gst_element_class_get_pad_template(
            GST_ELEMENT_GET_CLASS(elements_.tee), "src_%u");
        
        GstPad* tee_pad = gst_element_request_pad(elements_.tee, tee_template, nullptr, nullptr);
        GstPad* queue_pad = gst_element_get_static_pad(elements_.queue3, "sink");
        
        if (gst_pad_link(tee_pad, queue_pad) != GST_PAD_LINK_OK) {
            LOG_ERROR("Failed to link tee to encoder queue");
            gst_object_unref(queue_pad);
            return false;
        }
        gst_object_unref(queue_pad);
        
        if (!gst_element_link_many(elements_.queue3, elements_.encoder_convert,
                                  elements_.encoder, elements_.payloader,
                                  elements_.queue4, elements_.sink, nullptr)) {
            LOG_ERROR("Failed to link encoder chain");
            return false;
        }
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
        LOG_WARN("WebRTC output already exists: %s", peerId.c_str());
        return false;
    }
    
    if (!elements_.main_tee) {
        LOG_ERROR("Main tee not available");
        return false;
    }
    
    // WebRTC 출력 구조체 생성 (간소화된 구조)
    auto peerOutput = std::make_unique<PeerOutput>();
    peerOutput->peerId = peerId;
    
    // WebRTC Inter 요소 이름 생성
    char queueName[64], convName[64], sinkName[64];
    snprintf(queueName, sizeof(queueName), "webrtc_queue_%d", index_);
    snprintf(convName, sizeof(convName), "webrtc_conv_%d", index_);
    snprintf(sinkName, sizeof(sinkName), "webrtc_sink_%d", index_);
    
    // 1. Queue 생성
    peerOutput->queue = gst_element_factory_make("queue", queueName);
    if (!peerOutput->queue) {
        LOG_ERROR("Failed to create webrtc queue");
        return false;
    }
    g_object_set(peerOutput->queue,
                 "max-size-buffers", 10,
                 "leaky", 2,  // downstream
                 nullptr);
    
    // 2. Converter 생성
    peerOutput->encoder_convert = gst_element_factory_make("nvvideoconvert", convName);
    if (!peerOutput->encoder_convert) {
        LOG_ERROR("Failed to create webrtc converter");
        gst_object_unref(peerOutput->queue);
        return false;
    }
    
    // 3. InterVideo Sink 생성
    peerOutput->udpsink = gst_element_factory_make("intervideosink", sinkName);
    if (!peerOutput->udpsink) {
        LOG_ERROR("Failed to create intervideosink");
        gst_object_unref(peerOutput->queue);
        gst_object_unref(peerOutput->encoder_convert);
        return false;
    }
    
    // Inter 채널 설정
    const char* webrtc_channel = (type_ == CameraType::RGB) ? "Webrtc_RGB_Camera" : "Webrtc_Thermal_Camera";
    g_object_set(peerOutput->udpsink,
                 "channel", webrtc_channel,
                 nullptr);
    
    // encoder/payloader 멤버는 nullptr로 설정 (사용하지 않음)
    peerOutput->encoder = nullptr;
    peerOutput->payloader = nullptr;
    
    // 4. 파이프라인에 추가
    gst_bin_add_many(GST_BIN(pipeline_), 
                     peerOutput->queue, 
                     peerOutput->encoder_convert, 
                     peerOutput->udpsink, 
                     nullptr);
    
    // 5. 요소들 연결 (Queue → Converter → InterSink)
    if (!gst_element_link_many(peerOutput->queue, 
                              peerOutput->encoder_convert,
                              peerOutput->udpsink, nullptr)) {
        LOG_ERROR("Failed to link webrtc elements");
        // 실패시 정리
        gst_bin_remove_many(GST_BIN(pipeline_), 
                           peerOutput->queue, 
                           peerOutput->encoder_convert, 
                           peerOutput->udpsink, 
                           nullptr);
        return false;
    }
    
    // 6. Tee 패드 연결 (디버그 강화)
    GstPadTemplate* padTemplate = gst_element_class_get_pad_template(
        GST_ELEMENT_GET_CLASS(elements_.main_tee), "src_%u");
    
    if (!padTemplate) {
        LOG_ERROR("Failed to get tee pad template");
        return false;
    }
    
    peerOutput->teeSrcPad = gst_element_request_pad(elements_.main_tee, 
                                                    padTemplate, nullptr, nullptr);
    if (!peerOutput->teeSrcPad) {
        LOG_ERROR("Failed to request tee src pad");
        return false;
    }
    
    peerOutput->queueSinkPad = gst_element_get_static_pad(peerOutput->queue, "sink");
    if (!peerOutput->queueSinkPad) {
        LOG_ERROR("Failed to get queue sink pad");
        gst_element_release_request_pad(elements_.main_tee, peerOutput->teeSrcPad);
        gst_object_unref(peerOutput->teeSrcPad);
        return false;
    }
    
    // 패드 링크 시도 (디버그 정보 추가)
    GstPadLinkReturn linkResult = gst_pad_link(peerOutput->teeSrcPad, peerOutput->queueSinkPad);
    if (linkResult != GST_PAD_LINK_OK) {
        LOG_ERROR("Failed to link tee to webrtc queue: %d", linkResult);
        
        // Caps 정보 출력 (디버깅용)
        GstCaps* teeCaps = gst_pad_query_caps(peerOutput->teeSrcPad, nullptr);
        GstCaps* queueCaps = gst_pad_query_caps(peerOutput->queueSinkPad, nullptr);
        
        if (teeCaps) {
            gchar* teeStr = gst_caps_to_string(teeCaps);
            LOG_ERROR("Tee caps: %s", teeStr);
            g_free(teeStr);
            gst_caps_unref(teeCaps);
        }
        
        if (queueCaps) {
            gchar* queueStr = gst_caps_to_string(queueCaps);
            LOG_ERROR("Queue caps: %s", queueStr);
            g_free(queueStr);
            gst_caps_unref(queueCaps);
        }
        
        // 정리
        gst_element_release_request_pad(elements_.main_tee, peerOutput->teeSrcPad);
        gst_object_unref(peerOutput->teeSrcPad);
        gst_object_unref(peerOutput->queueSinkPad);
        return false;
    }
    
    // 7. 상태 동기화
    GstState state;
    gst_element_get_state(pipeline_, &state, nullptr, 0);
    
    if (state == GST_STATE_PLAYING) {
        gst_element_sync_state_with_parent(peerOutput->queue);
        gst_element_sync_state_with_parent(peerOutput->encoder_convert);
        gst_element_sync_state_with_parent(peerOutput->udpsink);
    }
    
    // 8. 저장
    peerOutputs_[peerId] = std::move(peerOutput);
    
    LOG_INFO("Added WebRTC Inter output for camera %d: channel=%s", index_, webrtc_channel);
    return true;
}

bool CameraSource::removePeerOutput(const std::string& peerId) {
    std::lock_guard<std::mutex> lock(peerOutputsMutex_);
    
    auto it = peerOutputs_.find(peerId);
    if (it == peerOutputs_.end()) {
        LOG_WARN("Peer output not found: %s", peerId.c_str());
        return false;
    }
    
    PeerOutput* peerOutput = it->second.get();
    
    // 1. 패드 연결 해제
    if (peerOutput->teeSrcPad && peerOutput->queueSinkPad) {
        gst_pad_unlink(peerOutput->teeSrcPad, peerOutput->queueSinkPad);
    }
    
    // 2. 요소들을 NULL 상태로 변경 (WebRTC는 인코더/페이로더 없음)
    if (peerOutput->queue) {
        gst_element_set_state(peerOutput->queue, GST_STATE_NULL);
    }
    if (peerOutput->encoder_convert) {
        gst_element_set_state(peerOutput->encoder_convert, GST_STATE_NULL);
    }
    if (peerOutput->udpsink) {  // 실제로는 intervideosink
        gst_element_set_state(peerOutput->udpsink, GST_STATE_NULL);
    }
    
    // WebRTC에서는 사용하지 않는 요소들 (NULL 체크 후 처리)
    if (peerOutput->encoder) {
        gst_element_set_state(peerOutput->encoder, GST_STATE_NULL);
    }
    if (peerOutput->payloader) {
        gst_element_set_state(peerOutput->payloader, GST_STATE_NULL);
    }
    
    // 3. 파이프라인에서 제거 (NULL 체크 후 제거)
    if (peerOutput->queue) {
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->queue);
    }
    if (peerOutput->encoder_convert) {
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->encoder_convert);
    }
    if (peerOutput->udpsink) {
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->udpsink);
    }
    
    // WebRTC에서는 사용하지 않는 요소들 (NULL 체크 후 제거)
    if (peerOutput->encoder) {
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->encoder);
    }
    if (peerOutput->payloader) {
        gst_bin_remove(GST_BIN(pipeline_), peerOutput->payloader);
    }
    
    // 4. Tee에서 request pad 해제
    if (peerOutput->teeSrcPad) {
        gst_element_release_request_pad(elements_.main_tee, peerOutput->teeSrcPad);
        gst_object_unref(peerOutput->teeSrcPad);
    }
    
    // 5. Queue sink pad 해제
    if (peerOutput->queueSinkPad) {
        gst_object_unref(peerOutput->queueSinkPad);
    }
    
    // 6. 맵에서 제거
    peerOutputs_.erase(it);
    
    LOG_INFO("Removed peer output for camera %d: peer=%s", index_, peerId.c_str());
    return true;
}

void CameraSource::saveBufferAsImage(GstBuffer* buffer, GstCaps* caps, int frameNumber) {
    // 파일명 생성
    char filename[256];
    snprintf(filename, sizeof(filename),
             "/home/nvidia/camera_%d_%s_frame_%06d.png",
             index_,
             (type_ == CameraType::RGB) ? "rgb" : "thermal",
             frameNumber);
    
    LOG_INFO("Attempting to save image: %s", filename);
    
    // GstSample 생성
    GstSample* sample = gst_sample_new(buffer, caps, nullptr, nullptr);
    if (!sample) {
        LOG_ERROR("Failed to create sample");
        return;
    }
    
    // 변환 파이프라인
    GError* error = nullptr;
    GstElement* pipeline = gst_parse_launch(
        "appsrc name=src ! "
        "videoconvert ! "
        "pngenc ! "
        "filesink name=sink",
        &error);
    
    if (error) {
        LOG_ERROR("Failed to create pipeline: %s", error->message);
        g_error_free(error);
        gst_sample_unref(sample);
        return;
    }
    
    // 요소 가져오기
    GstElement* appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "src");
    GstElement* filesink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
    
    // 설정
    g_object_set(appsrc, "caps", caps, nullptr);
    g_object_set(filesink, "location", filename, nullptr);
    
    // 파이프라인 시작
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    
    // 샘플 푸시
    g_signal_emit_by_name(appsrc, "push-sample", sample);
    g_signal_emit_by_name(appsrc, "end-of-stream");
    
    // 완료 대기
    GstBus* bus = gst_element_get_bus(pipeline);
    GstMessage* msg = gst_bus_timed_pop_filtered(bus, 5 * GST_SECOND,
                                                 (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    
    if (msg) {
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
            LOG_INFO("Successfully saved image: %s", filename);
        } else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
            gchar* debug;
            GError* err;
            gst_message_parse_error(msg, &err, &debug);
            LOG_ERROR("Error saving image: %s", err->message);
            g_error_free(err);
            g_free(debug);
        }
        gst_message_unref(msg);
    }
    
    // 정리
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(bus);
    gst_object_unref(appsrc);
    gst_object_unref(filesink);
    gst_object_unref(pipeline);
    gst_sample_unref(sample);
}

void CameraSource::convertToJpeg(const char* rawFile, int width, int height, 
                                GstVideoFormat format, int frameNumber) {
    // GStreamer를 사용한 JPEG 변환
    char pipeline_str[1024];
    char jpeg_filename[256];
    
    snprintf(jpeg_filename, sizeof(jpeg_filename),
             "/home/nvidia/camera_%d_%s_frame_%06d.jpg",
             index_,
             (type_ == CameraType::RGB) ? "rgb" : "thermal",
             frameNumber);
    
    const char* format_str = (format == GST_VIDEO_FORMAT_I420) ? "I420" : "NV12";
    
    snprintf(pipeline_str, sizeof(pipeline_str),
             "filesrc location=%s ! "
             "videoparse width=%d height=%d format=%s ! "
             "videoconvert ! "
             "jpegenc ! "
             "filesink location=%s",
             rawFile, width, height, format_str, jpeg_filename);
    
    GstElement* pipeline = gst_parse_launch(pipeline_str, nullptr);
    if (pipeline) {
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
        
        // 변환 완료 대기
        GstBus* bus = gst_element_get_bus(pipeline);
        GstMessage* msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE,
                                                     (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
        
        if (msg) {
            if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS) {
                LOG_INFO("Converted to JPEG: %s", jpeg_filename);
            }
            gst_message_unref(msg);
        }
        
        gst_object_unref(bus);
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
    }
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