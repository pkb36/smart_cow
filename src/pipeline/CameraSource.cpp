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
    
    // 소스 파싱 및 생성 (전체 소스 체인 포함)
    if (!parseAndCreateSource(config.source)) {
        return false;
    }
    
    // Tee는 이미 소스 체인에 포함되어 있음
    g_snprintf(elementName, sizeof(elementName), "video_src_tee%d", index_);
    tee_ = gst_bin_get_by_name(GST_BIN(source_), elementName);
    if (!tee_) {
        LOG_ERROR("Failed to find tee element in source chain");
        return false;
    }
    
    // 추론 파이프라인 파싱
    // 추론 파이프라인 파싱 - 먼저 외부 참조를 제거
    std::string inferConfig = config.inferConfig;
    
    // "video_src_teeX. !" 부분 제거
    size_t teePos = inferConfig.find("video_src_tee");
    if (teePos != std::string::npos) {
        size_t exclamPos = inferConfig.find("!", teePos);
        if (exclamPos != std::string::npos) {
            inferConfig = inferConfig.substr(exclamPos + 1);
        }
    }
    
    // "RGB.sink_0" 또는 "thermal.sink_0" 부분 제거
    const char* muxName = (type_ == CameraType::RGB) ? "RGB" : "thermal";
    std::string sinkRef = std::string(muxName) + ".sink_0";
    size_t sinkPos = inferConfig.find(sinkRef);
    if (sinkPos != std::string::npos) {
        inferConfig.erase(sinkPos, sinkRef.length());
    }
    
    // Queue 생성
    g_snprintf(elementName, sizeof(elementName), "infer_queue_%d", index_);
    queue_ = gst_element_factory_make("queue", elementName);
    
    // Videoscale 생성
    g_snprintf(elementName, sizeof(elementName), "infer_scale_%d", index_);
    scale_ = gst_element_factory_make("videoscale", elementName);
    
    // nvvideoconvert 생성
    g_snprintf(elementName, sizeof(elementName), "infer_conv_%d", index_);
    GstElement* inferConv = gst_element_factory_make("nvvideoconvert", elementName);
    
    // StreamMux 생성
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
    
    // 추론 엔진 (nvinfer) 생성
    g_snprintf(elementName, sizeof(elementName), "nvinfer_%d", index_ + 1);
    infer_ = gst_element_factory_make("nvinfer", elementName);
    if (!infer_) {
        LOG_ERROR("Failed to create nvinfer");
        return false;
    }
    
    // 추론 설정 파일 경로 추출
    std::string inferConfigPath;
    size_t configPos = config.inferConfig.find("config-file-path=");
    if (configPos != std::string::npos) {
        size_t start = configPos + 17;  // "config-file-path=" 길이
        size_t end = config.inferConfig.find(" ", start);
        if (end == std::string::npos) {
            inferConfigPath = config.inferConfig.substr(start);
        } else {
            inferConfigPath = config.inferConfig.substr(start, end - start);
        }
    }
    
    g_object_set(infer_,
                 "config-file-path", inferConfigPath.c_str(),
                 "unique-id", index_ + 1,
                 nullptr);
    
    // nvof 생성
    GstElement* nvof = gst_element_factory_make("nvof", nullptr);
    
    // nvvideoconvert 생성
    GstElement* postConv = gst_element_factory_make("nvvideoconvert", nullptr);
    
    // dspostproc 생성
    g_snprintf(elementName, sizeof(elementName), "dspostproc_%d", index_ + 1);
    postproc_ = gst_element_factory_make("dspostproc", elementName);
    
    // nvdsosd 생성
    g_snprintf(elementName, sizeof(elementName), "nvosd_%d", index_ + 1);
    osd_ = gst_element_factory_make("nvdsosd", elementName);
    if (!osd_) {
        LOG_ERROR("Failed to create nvdsosd");
        return false;
    }
    
    // 최종 converter 생성
    g_snprintf(elementName, sizeof(elementName), "output_conv_%d", index_);
    GstElement* outputConv = gst_element_factory_make("nvvideoconvert", elementName);
    
    // 모든 요소 확인
    if (!queue_ || !scale_ || !inferConv || !nvof || !postConv || 
        !postproc_ || !outputConv) {
        LOG_ERROR("Failed to create all inference elements");
        return false;
    }
    
    // 추론 체인을 bin으로 묶기
    GstElement* inferBin = gst_bin_new(nullptr);
    gst_bin_add_many(GST_BIN(inferBin),
                     queue_, scale_, inferConv, mux_, infer_, nvof,
                     postConv, postproc_, osd_, outputConv, nullptr);
    
    // 요소들 연결
    if (!gst_element_link(queue_, scale_)) {
        LOG_ERROR("Failed to link queue to scale");
        return false;
    }
    
    // Scale caps 설정
    GstCaps* scaleCaps = nullptr;
    if (type_ == CameraType::RGB) {
        scaleCaps = gst_caps_from_string("video/x-raw,width=1280,height=720");
    } else {
        scaleCaps = gst_caps_from_string("video/x-raw,width=640,height=480");
    }
    
    if (!gst_element_link_filtered(scale_, inferConv, scaleCaps)) {
        LOG_ERROR("Failed to link scale to converter");
        gst_caps_unref(scaleCaps);
        return false;
    }
    gst_caps_unref(scaleCaps);
    
    // Converter를 mux sink pad에 연결
    GstPad* convSrcPad = gst_element_get_static_pad(inferConv, "src");
    GstPad* muxSinkPad = gst_element_get_request_pad(mux_, "sink_0");
    
    if (gst_pad_link(convSrcPad, muxSinkPad) != GST_PAD_LINK_OK) {
        LOG_ERROR("Failed to link converter to mux");
        gst_object_unref(convSrcPad);
        gst_object_unref(muxSinkPad);
        return false;
    }
    gst_object_unref(convSrcPad);
    gst_object_unref(muxSinkPad);
    
    // 나머지 체인 연결
    if (!gst_element_link_many(mux_, infer_, nvof, postConv, 
                               postproc_, osd_, outputConv, nullptr)) {
        LOG_ERROR("Failed to link inference chain");
        return false;
    }
    
    // Ghost pad 추가
    GstPad* sinkPad = gst_element_get_static_pad(queue_, "sink");
    gst_element_add_pad(inferBin, gst_ghost_pad_new("sink", sinkPad));
    gst_object_unref(sinkPad);
    
    GstPad* srcPad = gst_element_get_static_pad(outputConv, "src");
    gst_element_add_pad(inferBin, gst_ghost_pad_new("src", srcPad));
    gst_object_unref(srcPad);
    
    // inferBin을 outputConverter_로 사용
    outputConverter_ = inferBin;
    
    // 트래커는 RGB 카메라에만 (추론 config에 nvtracker가 있는지 확인)
    if (type_ == CameraType::RGB && config.inferConfig.find("nvtracker") == std::string::npos) {
        // nvtracker가 없으면 추가하지 않음 (config에 없으면 필요없다고 가정)
        LOG_INFO("No nvtracker in config for RGB camera");
    }
    
    return true;
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