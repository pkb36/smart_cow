#include "CameraSource.h"
#include "../detection/DetectionBuffer.h"
#include "../detection/Detector.h"
#include "../utils/Logger.h"
#include "../utils/DeviceSetting.h"
#include <gst/gst.h>
#include <nvdsmeta.h>
#include <cmath>

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
    
    config_ = config;
    
    // GStreamer 요소 생성
    if (!createElements(config)) {
        return false;
    }
    
    // 파이프라인에 추가
    if (!addElementsToPipeline(pipeline)) {
        return false;
    }
    
    // 내부 요소 연결
    if (!linkInternalElements()) {
        return false;
    }
    
    // 프로브 설정
    if (!setupProbes()) {
        return false;
    }
    
    LOG_INFO("CameraSource initialized: %s camera with %s",
             (type_ == CameraType::RGB) ? "RGB" : "THERMAL",
             config.inferConfig.c_str());
    
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

bool CameraSource::createElements(const CameraConfig& config) {
    gchar elementName[64];
    
    // 소스 파싱 및 생성
    if (!parseAndCreateSource(config.source)) {
        return false;
    }
    
    // Clock Overlay
    g_snprintf(elementName, sizeof(elementName), "clock_overlay_%d", index_);
    clockOverlay_ = gst_element_factory_make("clockoverlay", elementName);
    if (!clockOverlay_) {
        LOG_ERROR("Failed to create clockoverlay");
        return false;
    }
    g_object_set(clockOverlay_,
                 "time-format", "%D %H:%M:%S",
                 "font-desc", "Arial, 18",
                 nullptr);
    
    // Tee (분기점)
    g_snprintf(elementName, sizeof(elementName), "video_src_tee%d", index_);
    tee_ = gst_element_factory_make("tee", elementName);
    if (!tee_) {
        LOG_ERROR("Failed to create tee");
        return false;
    }
    
    // 추론 파이프라인을 위한 요소들
    g_snprintf(elementName, sizeof(elementName), "infer_queue_%d", index_);
    queue_ = gst_element_factory_make("queue", elementName);
    
    g_snprintf(elementName, sizeof(elementName), "infer_scale_%d", index_);
    scale_ = gst_element_factory_make("videoscale", elementName);
    
    // 컨버터 (추론용)
    GstElement* inferConverter = gst_element_factory_make("nvvideoconvert", nullptr);
    
    // StreamMux
    const char* muxName = (type_ == CameraType::RGB) ? "RGB" : "thermal";
    mux_ = gst_element_factory_make("nvstreammux", muxName);
    if (!mux_) {
        LOG_ERROR("Failed to create nvstreammux");
        return false;
    }
    
    // StreamMux 설정
    int muxWidth = (type_ == CameraType::RGB) ? 1280 : 640;
    int muxHeight = (type_ == CameraType::RGB) ? 720 : 480;
    
    g_object_set(mux_,
                 "batch-size", 1,
                 "width", muxWidth,
                 "height", muxHeight,
                 "live-source", 1,
                 nullptr);
    
    // 추론 엔진 (nvinfer)
    g_snprintf(elementName, sizeof(elementName), "nvinfer_%d", index_ + 1);
    infer_ = gst_element_factory_make("nvinfer", elementName);
    if (!infer_) {
        LOG_ERROR("Failed to create nvinfer");
        return false;
    }
    g_object_set(infer_,
                 "config-file-path", config.inferConfig.c_str(),
                 "unique-id", index_ + 1,
                 nullptr);
    
    // 트래커는 RGB 카메라에만
    if (type_ == CameraType::RGB) {
        tracker_ = gst_element_factory_make("nvtracker", "tracker_1");
        if (tracker_) {
            g_object_set(tracker_,
                         "ll-lib-file", "/opt/nvidia/deepstream/deepstream/lib/libnvds_nvmultiobjecttracker.so",
                         "ll-config-file", "config_tracker_NvDCF_perf.yml",
                         "tracker-width", 640,
                         "tracker-height", 384,
                         nullptr);
        }
    }
    
    // Optical Flow (nvof)
    GstElement* opticalFlow = gst_element_factory_make("nvof", nullptr);
    
    // 후처리
    g_snprintf(elementName, sizeof(elementName), "dspostproc_%d", index_ + 1);
    postproc_ = gst_element_factory_make("dspostproc", elementName);
    
    // OSD
    g_snprintf(elementName, sizeof(elementName), "nvosd_%d", index_ + 1);
    osd_ = gst_element_factory_make("nvdsosd", elementName);
    if (!osd_) {
        LOG_ERROR("Failed to create nvdsosd");
        return false;
    }
    
    // 출력 컨버터
    outputConverter_ = gst_element_factory_make("nvvideoconvert", nullptr);
    
    // 모든 요소가 생성되었는지 확인
    if (!queue_ || !scale_ || !inferConverter || !opticalFlow || 
        !postproc_ || !outputConverter_) {
        LOG_ERROR("Failed to create all elements");
        return false;
    }
    
    return true;
}

bool CameraSource::parseAndCreateSource(const std::string& sourceStr) {
    // 소스 문자열 파싱 (예: "v4l2src device=/dev/video0 ! ...")
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
    // 모든 요소를 파이프라인에 추가
    gst_bin_add_many(GST_BIN(pipeline),
                     source_, clockOverlay_, tee_,
                     queue_, scale_, mux_, infer_,
                     postproc_, osd_, outputConverter_,
                     nullptr);
    
    // 트래커가 있으면 추가
    if (tracker_) {
        gst_bin_add(GST_BIN(pipeline), tracker_);
    }
    
    return true;
}

bool CameraSource::linkInternalElements() {
    // 소스 -> 오버레이 -> Tee
    if (!gst_element_link_many(source_, clockOverlay_, tee_, nullptr)) {
        LOG_ERROR("Failed to link source chain");
        return false;
    }
    
    // Tee -> Queue -> Scale
    GstPad* teeSrcPad = gst_element_get_request_pad(tee_, "src_%u");
    GstPad* queueSinkPad = gst_element_get_static_pad(queue_, "sink");
    
    if (gst_pad_link(teeSrcPad, queueSinkPad) != GST_PAD_LINK_OK) {
        LOG_ERROR("Failed to link tee to queue");
        return false;
    }
    
    gst_object_unref(teeSrcPad);
    gst_object_unref(queueSinkPad);
    
    // Scale 설정 및 연결
    GstCaps* scaleCaps = nullptr;
    if (type_ == CameraType::RGB) {
        scaleCaps = gst_caps_from_string("video/x-raw,width=1280,height=720");
    } else {
        scaleCaps = gst_caps_from_string("video/x-raw,width=640,height=480");
    }
    
    if (!gst_element_link_filtered(queue_, scale_, scaleCaps)) {
        LOG_ERROR("Failed to link queue to scale");
        gst_caps_unref(scaleCaps);
        return false;
    }
    gst_caps_unref(scaleCaps);
    
    // Scale -> nvvideoconvert -> Mux sink pad
    GstElement* tempConverter = gst_bin_get_by_name(GST_BIN(gst_element_get_parent(scale_)), 
                                                   "nvvideoconvert0");
    if (tempConverter && !gst_element_link(scale_, tempConverter)) {
        LOG_ERROR("Failed to link scale to converter");
        return false;
    }
    
    // Mux sink 패드 연결
    GstPad* muxSinkPad = gst_element_get_request_pad(mux_, 
                                                     type_ == CameraType::RGB ? "sink_0" : "sink_0");
    if (!muxSinkPad) {
        LOG_ERROR("Failed to get mux sink pad");
        return false;
    }
    
    // 추론 체인 연결
    GstElement* prevElement = mux_;
    
    // Mux -> Infer
    if (!gst_element_link(prevElement, infer_)) {
        LOG_ERROR("Failed to link to infer");
        return false;
    }
    prevElement = infer_;
    
    // Infer -> Tracker (RGB만)
    if (tracker_) {
        if (!gst_element_link(prevElement, tracker_)) {
            LOG_ERROR("Failed to link to tracker");
            return false;
        }
        prevElement = tracker_;
    }
    
    // -> PostProc -> OSD -> Converter
    if (!gst_element_link_many(prevElement, postproc_, osd_, outputConverter_, nullptr)) {
        LOG_ERROR("Failed to link inference chain");
        return false;
    }
    
    // 출력 해상도 설정
    GstCaps* outputCaps = nullptr;
    if (type_ == CameraType::RGB) {
        outputCaps = gst_caps_from_string("video/x-raw,width=1920,height=1080");
    } else {
        outputCaps = gst_caps_from_string("video/x-raw,width=384,height=288");
    }
    
    GstElement* nextElement = gst_bin_get_by_name(GST_BIN(gst_element_get_parent(outputConverter_)), 
                                                  type_ == CameraType::RGB ? "video_enc_tee1_0" : "video_enc_tee1_1");
    if (nextElement) {
        gst_element_link_filtered(outputConverter_, nextElement, outputCaps);
        gst_object_unref(nextElement);
    }
    
    if (outputCaps) {
        gst_caps_unref(outputCaps);
    }
    
    return true;
}

bool CameraSource::setupProbes() {
    // OSD sink 패드에 프로브 추가
    GstPad* osdSinkPad = gst_element_get_static_pad(osd_, "sink");
    if (!osdSinkPad) {
        LOG_ERROR("Failed to get OSD sink pad");
        return false;
    }
    
    // 프로브 추가
    gst_pad_add_probe(osdSinkPad, GST_PAD_PROBE_TYPE_BUFFER,
                      CameraSource::osdSinkPadProbe, this, nullptr);
    
    gst_object_unref(osdSinkPad);
    
    LOG_DEBUG("Probe set up on OSD sink pad");
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