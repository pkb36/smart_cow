#include "CameraSource.h"
#include "../detection/DetectionBuffer.h"
#include "../detection/Detector.h"
#include "../utils/Logger.h"
#include "../utils/DeviceSetting.h"
#include <gst/gst.h>
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
    // 1. UDP 소스 직후 프로브
    GstElement* udpsrc = gst_bin_get_by_name(GST_BIN(source_), "udpsrc");
    if (udpsrc) {
        GstPad* srcPad = gst_element_get_static_pad(udpsrc, "src");
        if (srcPad) {
            gst_pad_add_probe(srcPad, GST_PAD_PROBE_TYPE_BUFFER,
                [](GstPad* pad, GstPadProbeInfo* info, gpointer data) -> GstPadProbeReturn {
                    CameraSource* self = static_cast<CameraSource*>(data);
                    static guint64 count[2] = {0, 0};
                    count[self->index_]++;
                    if (count[self->index_] % 30 == 0) {
                        LOG_INFO("[%s] UDP 수신: %llu 패킷", 
                                (self->type_ == CameraType::RGB) ? "RGB" : "THERMAL",
                                count[self->index_]);
                    }
                    return GST_PAD_PROBE_OK;
                }, this, nullptr);
            gst_object_unref(srcPad);
        }
    }
    
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
    
    GstBuffer* buf = GST_PAD_PROBE_INFO_BUFFER(info);
    NvDsBatchMeta* batchMeta = gst_buffer_get_nvds_batch_meta(buf);
    
    if (!batchMeta) {
        return GST_PAD_PROBE_OK;
    }
    
    // 프레임 카운트
    static guint frameCount[2] = {0, 0};
    frameCount[self->index_]++;
    
    // DeviceSetting 확인
    auto& settings = DeviceSetting::getInstance().get();
    
    // 분석이 비활성화되면 건너뛰기
    if (!settings.analysisStatus) {
        return GST_PAD_PROBE_OK;
    }
    
    // 프레임별 처리
    for (NvDsMetaList* l_frame = batchMeta->frame_meta_list; 
         l_frame != nullptr; l_frame = l_frame->next) {
        
        NvDsFrameMeta* frameMeta = static_cast<NvDsFrameMeta*>(l_frame->data);
        
        // 검출 데이터 생성
        DetectionData detection;
        detection.frameNumber = frameMeta->frame_num;
        detection.cameraType = self->type_;
        
        // 타임스탬프
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        detection.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
        
        // 객체 처리
        for (NvDsMetaList* l_obj = frameMeta->obj_meta_list;
             l_obj != nullptr; l_obj = l_obj->next) {
            
            NvDsObjectMeta* objMeta = static_cast<NvDsObjectMeta*>(l_obj->data);
            
            // DetectedObject 생성
            DetectedObject obj;
            obj.classId = objMeta->class_id;
            obj.confidence = objMeta->confidence;
            obj.bbox.x = static_cast<int>(objMeta->rect_params.left);
            obj.bbox.y = static_cast<int>(objMeta->rect_params.top);
            obj.bbox.width = static_cast<int>(objMeta->rect_params.width);
            obj.bbox.height = static_cast<int>(objMeta->rect_params.height);
            
            // 색상 결정
            obj.color = self->determineObjectColor(objMeta);
            obj.hasBbox = (obj.color != BboxColor::NONE);
            
            // OSD 메타데이터 업데이트
            if (obj.hasBbox) {
                switch (obj.color) {
                    case BboxColor::GREEN:
                        objMeta->rect_params.border_color.red = 0.0;
                        objMeta->rect_params.border_color.green = 1.0;
                        objMeta->rect_params.border_color.blue = 0.0;
                        break;
                    case BboxColor::YELLOW:
                        objMeta->rect_params.border_color.red = 1.0;
                        objMeta->rect_params.border_color.green = 1.0;
                        objMeta->rect_params.border_color.blue = 0.0;
                        break;
                    case BboxColor::RED:
                        objMeta->rect_params.border_color.red = 1.0;
                        objMeta->rect_params.border_color.green = 0.0;
                        objMeta->rect_params.border_color.blue = 0.0;
                        break;
                    case BboxColor::BLUE:
                        objMeta->rect_params.border_color.red = 0.0;
                        objMeta->rect_params.border_color.green = 0.0;
                        objMeta->rect_params.border_color.blue = 1.0;
                        break;
                    default:
                        objMeta->rect_params.has_bg_color = 0;
                        break;
                }
                
                if (obj.color != BboxColor::NONE) {
                    objMeta->rect_params.border_color.alpha = 1.0;
                    objMeta->rect_params.border_width = 3;
                }
            }
            
            detection.objects.push_back(obj);
        }
        
        // 검출 버퍼에 추가
        if (!detection.objects.empty()) {
            self->detectionBuffer_->addDetection(detection);
        }
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
    
    // UDP 소스
    g_snprintf(elementName, sizeof(elementName), "udpsrc_%d", index_);

    // UDP 소스
    elements_.udpsrc = gst_element_factory_make("udpsrc", elementName);
    g_object_set(elements_.udpsrc,
                 "port", config.source.port,
                 "buffer-size", 524288,
                 nullptr);
    
    g_snprintf(elementName, sizeof(elementName), "rtpdepay_%d", index_);             
    // RTP Depayloader
    if (config.source.encoding == "h264") {
        elements_.rtpdepay = gst_element_factory_make("rtph264depay", elementName);
        elements_.parser = gst_element_factory_make("h264parse", nullptr);
    } else if (config.source.encoding == "h265") {
        elements_.rtpdepay = gst_element_factory_make("rtph265depay", nullptr);
        elements_.parser = gst_element_factory_make("h265parse", nullptr);
    }
    
    // 디코더
    elements_.decoder = gst_element_factory_make("nvv4l2decoder", nullptr);
    
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
                    "max-size-buffers", 5,
                    "max-size-bytes", 0,
                    "max-size-time", 0,
                    "leaky", 2,  // downstream
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
                 nullptr);
    
    // 추론
    elements_.infer = gst_element_factory_make("nvinfer", nullptr);
    g_object_set(elements_.infer,
                 "config-file-path", config.inference.config_file.c_str(),
                 "unique-id", index_ + 1,
                 nullptr);
    
    // 후처리
    elements_.nvof = nullptr;
    elements_.converter3 = gst_element_factory_make("nvvideoconvert", nullptr);
    elements_.postproc = gst_element_factory_make("dspostproc", nullptr);
    elements_.osd = gst_element_factory_make("nvdsosd", nullptr);
    elements_.converter4 = gst_element_factory_make("nvvideoconvert", nullptr);
    
    return true;
}

bool CameraSource::createEncoderChain(const CameraConfig& config) {
    gchar elementName[64];  // 요소 이름을 위한 버퍼
    
    // Queue 생성 - 고유한 이름 사용
    g_snprintf(elementName, sizeof(elementName), "encoder_queue_%d", index_);
    elements_.queue3 = gst_element_factory_make("queue", elementName);
    if (!elements_.queue3) {
        LOG_ERROR("Failed to create encoder queue");
        return false;
    }
    
    // nvvideoconvert for memory conversion - 고유한 이름 사용
    g_snprintf(elementName, sizeof(elementName), "encoder_convert_%d", index_);
    elements_.encoder_convert = gst_element_factory_make("nvvideoconvert", elementName);
    if (!elements_.encoder_convert) {
        LOG_ERROR("Failed to create encoder converter");
        return false;
    }
    
    // 인코더 생성 - 고유한 이름 사용
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
    
    // RTP Payloader 생성 - 고유한 이름 사용
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
    
    // 출력 큐 - 고유한 이름 사용
    g_snprintf(elementName, sizeof(elementName), "output_queue_%d", index_);
    elements_.queue4 = gst_element_factory_make("queue", elementName);
    if (!elements_.queue4) {
        LOG_ERROR("Failed to create output queue");
        return false;
    }
    
    // 출력 싱크 - 고유한 이름 사용
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
                 nullptr);
    
    LOG_INFO("Encoder chain created for camera %d: codec=%s, bitrate=%d, port=%d",
             index_, config.encoder.codec.c_str(), config.encoder.bitrate, 5000 + index_);
    
    return true;
}

bool CameraSource::linkElements(const CameraConfig& config) {
    // 먼저 pipeline_ 멤버 변수 추가 필요
    if (!pipeline_) {
        LOG_ERROR("Pipeline not set");
        return false;
    }
    
    // 1. 파이프라인에 모든 요소 추가
    gst_bin_add_many(GST_BIN(pipeline_),
        // 소스 체인
        elements_.udpsrc, elements_.rtpdepay, elements_.parser,
        elements_.decoder, elements_.converter1, elements_.clockoverlay,
        elements_.videorate, elements_.capsfilter, elements_.queue1,
        elements_.tee,
        // 인코더 체인 (encoder_convert 추가!)
        elements_.queue3, elements_.encoder_convert, elements_.encoder, 
        elements_.payloader, elements_.queue4, elements_.sink,
        nullptr);
    
    // 추론이 활성화된 경우 추론 체인도 추가
    if (config.inference.enabled) {
        gst_bin_add_many(GST_BIN(pipeline_),
            elements_.queue2, elements_.videoscale, elements_.converter2,
            elements_.mux, elements_.infer,
            elements_.converter3, elements_.postproc, elements_.osd,
            elements_.converter4,
            nullptr);
    }
    
    // 2. RTP caps 설정
    GstCaps* rtp_caps = nullptr;
    if (config.source.encoding == "h264") {
        rtp_caps = gst_caps_from_string(
            "application/x-rtp,media=video,clock-rate=90000,"
            "encoding-name=H264,payload=96");
    } else if (config.source.encoding == "h265") {
        rtp_caps = gst_caps_from_string(
            "application/x-rtp,media=video,clock-rate=90000,"
            "encoding-name=H265,payload=96");
    }
    
    // 3. 소스 체인 연결
    if (!gst_element_link_filtered(elements_.udpsrc, elements_.rtpdepay, rtp_caps)) {
        LOG_ERROR("Failed to link udpsrc to rtpdepay");
        gst_caps_unref(rtp_caps);
        return false;
    }
    gst_caps_unref(rtp_caps);
    
    if (!gst_element_link_many(
            elements_.rtpdepay, elements_.parser,
            elements_.decoder, elements_.converter1, 
            elements_.clockoverlay, elements_.videorate, 
            elements_.capsfilter, elements_.queue1,
            elements_.tee, nullptr)) {
        LOG_ERROR("Failed to link source chain");
        return false;
    }
    
    // 4. Tee에서 인코더 체인으로 연결
    GstPadTemplate* tee_src_pad_template = gst_element_class_get_pad_template(
        GST_ELEMENT_GET_CLASS(elements_.tee), "src_%u");
    
    GstPad* tee_encoder_pad = gst_element_request_pad(
        elements_.tee, tee_src_pad_template, nullptr, nullptr);
    GstPad* encoder_queue_pad = gst_element_get_static_pad(elements_.queue3, "sink");
    
    if (gst_pad_link(tee_encoder_pad, encoder_queue_pad) != GST_PAD_LINK_OK) {
        LOG_ERROR("Failed to link tee to encoder queue");
        gst_object_unref(encoder_queue_pad);
        return false;
    }
    gst_object_unref(encoder_queue_pad);
    
    LOG_DEBUG("Linking encoder chain...");

    if (!elements_.queue3) LOG_ERROR("queue3 is null");
    if (!elements_.encoder) LOG_ERROR("encoder is null");
    if (!elements_.payloader) LOG_ERROR("payloader is null");
    if (!elements_.queue4) LOG_ERROR("queue4 is null");
    if (!elements_.sink) LOG_ERROR("sink is null");

    // 5. 인코더 체인 연결
    if (!gst_element_link(elements_.queue3, elements_.encoder_convert)) {
        LOG_ERROR("Failed to link queue3 to encoder_convert");
        return false;
    }

    if (!gst_element_link(elements_.encoder_convert, elements_.encoder)) {
        LOG_ERROR("Failed to link encoder_convert to encoder");
        return false;
    }
    
    if (!gst_element_link(elements_.encoder, elements_.payloader)) {
        LOG_ERROR("Failed to link encoder to payloader");
        return false;
    }
    
    if (!gst_element_link(elements_.payloader, elements_.queue4)) {
        LOG_ERROR("Failed to link payloader to queue4");
        return false;
    }
    
    if (!gst_element_link(elements_.queue4, elements_.sink)) {
        LOG_ERROR("Failed to link queue4 to sink");
        return false;
    }
    
    LOG_DEBUG("Encoder chain linked successfully");
    
    // 6. 추론이 활성화된 경우
    if (config.inference.enabled) {
        LOG_INFO("추론 체인 연결 중... (Camera %d)", index_);
        
        // Tee에서 추론 체인으로 연결
        GstPad* tee_infer_pad = gst_element_request_pad(
            elements_.tee, tee_src_pad_template, nullptr, nullptr);
        GstPad* infer_queue_pad = gst_element_get_static_pad(elements_.queue2, "sink");
        
        if (gst_pad_link(tee_infer_pad, infer_queue_pad) != GST_PAD_LINK_OK) {
            LOG_ERROR("Failed to link tee to inference queue");
            gst_object_unref(infer_queue_pad);
            return false;
        }
        gst_object_unref(infer_queue_pad);
        LOG_INFO("Tee -> queue2 연결 성공");
        
        // ⭐ 추론 체인 내부 연결 (누락된 부분!)
        if (!gst_element_link(elements_.queue2, elements_.videoscale)) {
            LOG_ERROR("Failed to link queue2 to videoscale");
            return false;
        }
        LOG_INFO("queue2 -> videoscale 연결 성공");
        
        // Scale caps 설정
        GstCaps* scale_caps = gst_caps_new_simple("video/x-raw",
            "width", G_TYPE_INT, config.inference.scale_width,
            "height", G_TYPE_INT, config.inference.scale_height,
            nullptr);
        
        if (!gst_element_link_filtered(elements_.videoscale, elements_.converter2, scale_caps)) {
            LOG_ERROR("Failed to link videoscale to converter2");
            gst_caps_unref(scale_caps);
            return false;
        }
        gst_caps_unref(scale_caps);
        LOG_INFO("videoscale -> converter2 연결 성공");
        
        // Converter를 mux에 연결
        GstPad* conv_src_pad = gst_element_get_static_pad(elements_.converter2, "src");
        GstPad* mux_sink_pad = gst_element_get_request_pad(elements_.mux, "sink_0");
        
        if (!conv_src_pad || !mux_sink_pad) {
            LOG_ERROR("Failed to get pads for mux connection");
            if (conv_src_pad) gst_object_unref(conv_src_pad);
            if (mux_sink_pad) {
                gst_element_release_request_pad(elements_.mux, mux_sink_pad);
                gst_object_unref(mux_sink_pad);
            }
            return false;
        }
        
        if (gst_pad_link(conv_src_pad, mux_sink_pad) != GST_PAD_LINK_OK) {
            LOG_ERROR("Failed to link converter to mux");
            gst_object_unref(conv_src_pad);
            gst_element_release_request_pad(elements_.mux, mux_sink_pad);
            gst_object_unref(mux_sink_pad);
            return false;
        }
        gst_object_unref(conv_src_pad);
        LOG_INFO("converter2 -> mux 연결 성공");
        
        if (!gst_element_link(elements_.mux, elements_.infer)) {
            LOG_ERROR("Failed to link mux to infer");
            return false;
        }
        LOG_INFO("연결: mux -> infer ✓");

        if (!gst_element_link(elements_.infer, elements_.converter3)) {
            LOG_ERROR("Failed to link infer to converter3");
            
            // 디버깅: 패드 상태 확인
            GstPad* infer_src = gst_element_get_static_pad(elements_.infer, "src");
            GstPad* conv3_sink = gst_element_get_static_pad(elements_.converter3, "sink");
            
            LOG_ERROR("infer src pad: %p", infer_src);
            LOG_ERROR("converter3 sink pad: %p", conv3_sink);
            
            if (infer_src) gst_object_unref(infer_src);
            if (conv3_sink) gst_object_unref(conv3_sink);
            
            return false;
        }
        LOG_INFO("연결: infer -> converter3 ✓");

        // 3. converter3 -> postproc
        if (!gst_element_link(elements_.converter3, elements_.postproc)) {
            LOG_ERROR("Failed to link converter3 to postproc");
            return false;
        }
        LOG_INFO("연결: converter3 -> postproc ✓");
        
        // 4. postproc -> osd
        if (!gst_element_link(elements_.postproc, elements_.osd)) {
            LOG_ERROR("Failed to link postproc to osd");
            return false;
        }
        LOG_INFO("연결: postproc -> osd ✓");
        
        // 5. osd -> converter4
        if (!gst_element_link(elements_.osd, elements_.converter4)) {
            LOG_ERROR("Failed to link osd to converter4");
            return false;
        }
        LOG_INFO("연결: osd -> converter4 ✓");

        GstElement* fakesink = gst_element_factory_make("fakesink", nullptr);
        gst_bin_add(GST_BIN(pipeline_), fakesink);
        
        if (!gst_element_link(elements_.converter4, fakesink)) {
            LOG_ERROR("Failed to link converter4 to fakesink");
            return false;
        }
        LOG_INFO("연결: converter4 -> fakesink ✓ (테스트용)");
        LOG_INFO("추론 체인 연결 완료");
    }

    LOG_INFO("All elements linked successfully for %s camera",
             (type_ == CameraType::RGB) ? "RGB" : "THERMAL");
    
    return true;
}

bool CameraSource::addProbes() {
    // UDP 수신 모니터링
    GstPad* udp_src_pad = gst_element_get_static_pad(elements_.udpsrc, "src");
    gst_pad_add_probe(udp_src_pad, GST_PAD_PROBE_TYPE_BUFFER,
        [](GstPad* pad, GstPadProbeInfo* info, gpointer data) -> GstPadProbeReturn {
            static guint64 count = 0;
            static time_t last_time = 0;
            
            count++;
            time_t now = time(nullptr);
            if (now != last_time) {
                LOG_INFO("UDP 수신: %llu 패킷/초", count);
                count = 0;
                last_time = now;
            }
            return GST_PAD_PROBE_OK;
        }, this, nullptr);
    
    // 2. 디코더 이후 프로브 추가
    if (elements_.decoder) {
        GstPad* decoder_src_pad = gst_element_get_static_pad(elements_.decoder, "src");
        if (decoder_src_pad) {
            gst_pad_add_probe(decoder_src_pad, GST_PAD_PROBE_TYPE_BUFFER,
                [](GstPad* pad, GstPadProbeInfo* info, gpointer data) -> GstPadProbeReturn {
                    CameraSource* self = static_cast<CameraSource*>(data);
                    static guint64 count[2] = {0, 0};
                    count[self->index_]++;
                    if (count[self->index_] % 30 == 0) {
                        LOG_INFO("[Camera %d] 디코더 출력: %llu 프레임", 
                                self->index_, count[self->index_]);
                    }
                    return GST_PAD_PROBE_OK;
                }, this, nullptr);
            gst_object_unref(decoder_src_pad);
        }
    }

    if (elements_.queue2) {
        // src pad 프로브도 추가
        GstPad* queue2_src = gst_element_get_static_pad(elements_.queue2, "src");
        if (queue2_src) {
            gst_pad_add_probe(queue2_src, GST_PAD_PROBE_TYPE_BUFFER,
                [](GstPad* pad, GstPadProbeInfo* info, gpointer data) -> GstPadProbeReturn {
                    CameraSource* self = static_cast<CameraSource*>(data);
                    static guint64 count[2] = {0, 0};
                    count[self->index_]++;
                    if (count[self->index_] % 30 == 0) {
                        LOG_INFO("[Camera %d] Queue2 출력: %llu 프레임", 
                                self->index_, count[self->index_]);
                    }
                    return GST_PAD_PROBE_OK;
                }, this, nullptr);
            gst_object_unref(queue2_src);
        }
    }

    // 3. Tee 직전 프로브 추가
    if (elements_.tee) {
        GstPad* tee_sink_pad = gst_element_get_static_pad(elements_.tee, "sink");
        if (tee_sink_pad) {
            gst_pad_add_probe(tee_sink_pad, GST_PAD_PROBE_TYPE_BUFFER,
                [](GstPad* pad, GstPadProbeInfo* info, gpointer data) -> GstPadProbeReturn {
                    CameraSource* self = static_cast<CameraSource*>(data);
                    static guint64 count[2] = {0, 0};
                    count[self->index_]++;
                    if (count[self->index_] % 30 == 0) {
                        LOG_INFO("[Camera %d] Tee 입력: %llu 프레임", 
                                self->index_, count[self->index_]);
                    }
                    return GST_PAD_PROBE_OK;
                }, this, nullptr);
            gst_object_unref(tee_sink_pad);
        }
    }
    
// 4. 추론 체인 입구 확인
    if (config_.inference.enabled && elements_.queue2) {
        GstPad* infer_queue_sink = gst_element_get_static_pad(elements_.queue2, "sink");
        if (infer_queue_sink) {
            gst_pad_add_probe(infer_queue_sink, GST_PAD_PROBE_TYPE_BUFFER,
                [](GstPad* pad, GstPadProbeInfo* info, gpointer data) -> GstPadProbeReturn {
                    CameraSource* self = static_cast<CameraSource*>(data);
                    static guint64 count[2] = {0, 0};
                    count[self->index_]++;
                    if (count[self->index_] % 30 == 0) {
                        LOG_INFO("[Camera %d] 추론 큐 입력: %llu 프레임", 
                                self->index_, count[self->index_]);
                    }
                    return GST_PAD_PROBE_OK;
                }, this, nullptr);
            gst_object_unref(infer_queue_sink);
        }
    }
    
    // 5. Mux 출력 확인
    if (config_.inference.enabled && elements_.mux) {
        GstPad* mux_src = gst_element_get_static_pad(elements_.mux, "src");
        if (mux_src) {
            gst_pad_add_probe(mux_src, GST_PAD_PROBE_TYPE_BUFFER,
                [](GstPad* pad, GstPadProbeInfo* info, gpointer data) -> GstPadProbeReturn {
                    CameraSource* self = static_cast<CameraSource*>(data);
                    static guint64 count[2] = {0, 0};
                    count[self->index_]++;
                    if (count[self->index_] % 30 == 0) {
                        LOG_INFO("[Camera %d] Mux 출력: %llu 프레임", 
                                self->index_, count[self->index_]);
                    }
                    return GST_PAD_PROBE_OK;
                }, this, nullptr);
            gst_object_unref(mux_src);
        }
    }
    
    // 6. 기존 OSD 프로브
    if (elements_.osd) {
        GstPad* osd_sink_pad = gst_element_get_static_pad(elements_.osd, "sink");
        if (osd_sink_pad) {
            gst_pad_add_probe(osd_sink_pad, GST_PAD_PROBE_TYPE_BUFFER,
                CameraSource::osdSinkPadProbe, this, nullptr);
            gst_object_unref(osd_sink_pad);
        }
    }

    // 7. videoscale 출력 확인
    if (elements_.videoscale) {
        GstPad* scale_src = gst_element_get_static_pad(elements_.videoscale, "src");
        if (scale_src) {
            gst_pad_add_probe(scale_src, GST_PAD_PROBE_TYPE_BUFFER,
                [](GstPad* pad, GstPadProbeInfo* info, gpointer data) -> GstPadProbeReturn {
                    CameraSource* self = static_cast<CameraSource*>(data);
                    static guint64 count[2] = {0, 0};
                    count[self->index_]++;
                    if (count[self->index_] % 30 == 0) {
                        // Caps 정보도 출력
                        GstCaps* caps = gst_pad_get_current_caps(pad);
                        if (caps) {
                            gchar* caps_str = gst_caps_to_string(caps);
                            LOG_INFO("[Camera %d] Videoscale 출력: %llu 프레임, caps: %s", 
                                    self->index_, count[self->index_], caps_str);
                            g_free(caps_str);
                            gst_caps_unref(caps);
                        }
                    }
                    return GST_PAD_PROBE_OK;
                }, this, nullptr);
            gst_object_unref(scale_src);
        }
    }

    if (elements_.converter2) {
        GstPad* conv2_src = gst_element_get_static_pad(elements_.converter2, "src");
        if (conv2_src) {
            gst_pad_add_probe(conv2_src, GST_PAD_PROBE_TYPE_BUFFER,
                [](GstPad* pad, GstPadProbeInfo* info, gpointer data) -> GstPadProbeReturn {
                    CameraSource* self = static_cast<CameraSource*>(data);
                    static guint64 count[2] = {0, 0};
                    count[self->index_]++;
                    if (count[self->index_] % 30 == 0) {
                        LOG_INFO("[Camera %d] Converter2 출력: %llu 프레임", 
                                self->index_, count[self->index_]);
                    }
                    return GST_PAD_PROBE_OK;
                }, this, nullptr);
            gst_object_unref(conv2_src);
        }
    }

    // 9. Infer 출력 확인
    if (elements_.infer) {
        GstPad* infer_src = gst_element_get_static_pad(elements_.infer, "src");
        if (infer_src) {
            gst_pad_add_probe(infer_src, GST_PAD_PROBE_TYPE_BUFFER,
                [](GstPad* pad, GstPadProbeInfo* info, gpointer data) -> GstPadProbeReturn {
                    CameraSource* self = static_cast<CameraSource*>(data);
                    static guint64 count[2] = {0, 0};
                    count[self->index_]++;
                    if (count[self->index_] % 30 == 0) {
                        LOG_INFO("[Camera %d] Infer 출력: %llu 프레임", 
                                self->index_, count[self->index_]);
                    }
                    return GST_PAD_PROBE_OK;
                }, this, nullptr);
            gst_object_unref(infer_src);
        }
    }
    
    // 10. nvof 출력 확인
    if (elements_.nvof) {
        GstPad* nvof_src = gst_element_get_static_pad(elements_.nvof, "src");
        if (nvof_src) {
            gst_pad_add_probe(nvof_src, GST_PAD_PROBE_TYPE_BUFFER,
                [](GstPad* pad, GstPadProbeInfo* info, gpointer data) -> GstPadProbeReturn {
                    CameraSource* self = static_cast<CameraSource*>(data);
                    static guint64 count[2] = {0, 0};
                    count[self->index_]++;
                    if (count[self->index_] % 30 == 0) {
                        LOG_INFO("[Camera %d] NVOF 출력: %llu 프레임", 
                                self->index_, count[self->index_]);
                    }
                    return GST_PAD_PROBE_OK;
                }, this, nullptr);
            gst_object_unref(nvof_src);
        }
    }
    
    return true;
}