#include "Detector.h"
#include "../utils/Logger.h"
#include "../utils/DeviceSetting.h"
#include <gstnvdsmeta.h>
#include <chrono>
#include <fstream>
#include <cmath>

Detector::Detector(CameraType cameraType)
    : cameraType_(cameraType)
    , enabled_(true)
    , interval_(0) {
    
    LOG_INFO("Detector created for %s camera",
             (cameraType == CameraType::RGB) ? "RGB" : "THERMAL");
}

Detector::~Detector() {}

bool Detector::init(const std::string& configFile) {
    configFile_ = configFile;
    
    // 설정 파일 검증 (실제 DeepStream 초기화는 파이프라인에서 수행)
    std::ifstream file(configFile);
    if (!file.is_open()) {
        LOG_ERROR("Failed to open detector config file: %s", configFile.c_str());
        return false;
    }
    file.close();
    
    LOG_INFO("Detector initialized with config: %s", configFile.c_str());
    return true;
}

void Detector::setDetectionCallback(DetectionCallback callback) {
    callback_ = callback;
}

void Detector::processBatchMeta(NvDsBatchMeta* batchMeta, uint32_t frameNumber) {
    if (!enabled_ || !batchMeta || !callback_) {
        return;
    }
    
    // 인터벌 체크
    static uint32_t lastProcessedFrame = 0;
    if (interval_ > 0 && (frameNumber - lastProcessedFrame) < static_cast<uint32_t>(interval_)) {
        return;
    }
    lastProcessedFrame = frameNumber;
    
    // 프레임 메타데이터 순회
    for (NvDsMetaList* l_frame = batchMeta->frame_meta_list; 
         l_frame != nullptr; l_frame = l_frame->next) {
        
        NvDsFrameMeta* frameMeta = reinterpret_cast<NvDsFrameMeta*>(l_frame->data);
        if (!frameMeta) continue;
        
        // DetectionData 생성
        DetectionData detection;
        detection.frameNumber = frameNumber;
        detection.cameraType = cameraType_;
        
        // 타임스탬프 설정
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        detection.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
        
        // 객체 메타데이터 처리
        for (NvDsMetaList* l_obj = frameMeta->obj_meta_list;
             l_obj != nullptr; l_obj = l_obj->next) {
            
            NvDsObjectMeta* objMeta = reinterpret_cast<NvDsObjectMeta*>(l_obj->data);
            if (!objMeta) continue;
            
            // 검출된 객체를 DetectedObject로 변환
            DetectedObject obj = convertToDetectedObject(objMeta);
            
            // 필터링 (신뢰도 임계값)
            if (obj.confidence > 0.3) {  // 기본 임계값
                detection.objects.push_back(obj);
            }
        }
        
        // 콜백 호출
        if (!detection.objects.empty()) {
            callback_(detection);
            
            LOG_TRACE("Detection processed: frame=%u, objects=%zu",
                      frameNumber, detection.objects.size());
        }
    }
}

void Detector::setEnabled(bool enabled) {
    enabled_ = enabled;
    LOG_INFO("Detector %s", enabled ? "enabled" : "disabled");
}

bool Detector::isEnabled() const {
    return enabled_;
}

void Detector::setInterval(int interval) {
    interval_ = interval;
    LOG_INFO("Detector interval set to: %d", interval);
}

DetectedObject Detector::convertToDetectedObject(NvDsObjectMeta* objMeta) {
    DetectedObject obj;
    
    obj.classId = objMeta->class_id;
    obj.confidence = objMeta->confidence;
    
    // 바운딩 박스
    obj.bbox.x = static_cast<int>(objMeta->rect_params.left);
    obj.bbox.y = static_cast<int>(objMeta->rect_params.top);
    obj.bbox.width = static_cast<int>(objMeta->rect_params.width);
    obj.bbox.height = static_cast<int>(objMeta->rect_params.height);
    
    // 색상 결정
    obj.color = determineColor(obj.classId, obj);
    obj.hasBbox = (obj.color != BboxColor::NONE);
    
    return obj;
}

BboxColor Detector::determineColor(int classId, const DetectedObject& obj) {
    // PTZ 이동 중이면 박스 없음
    // TODO: g_move_speed 체크 (전역 변수 대신 상태 관리 필요)
    
    // 너무 작거나 큰 객체 필터링
    float diagonal = std::sqrt(obj.bbox.width * obj.bbox.width + 
                              obj.bbox.height * obj.bbox.height);
    
    float minDiag = (cameraType_ == CameraType::RGB) ? 40.0f : 40.0f;
    float maxDiag = (cameraType_ == CameraType::RGB) ? 1000.0f : 1000.0f;
    
    if (diagonal < minDiag || diagonal > maxDiag) {
        return BboxColor::NONE;
    }
    
    // 클래스별 색상 결정
    switch (classId) {
        case CLASS_NORMAL_COW:
        case CLASS_NORMAL_COW_SITTING:
            return BboxColor::GREEN;
            
        case CLASS_HEAT_COW:
            // ResNet50 적용 체크
            if (DeviceSetting::getInstance().get().resnet50Apply) {
                // TODO: heat_count는 별도 추적 필요
                return BboxColor::RED;
            }
            return BboxColor::YELLOW;
            
        case CLASS_FLIP_COW:
            // Optical Flow 적용 체크
            if (DeviceSetting::getInstance().get().optFlowApply) {
                // TODO: opt_flow_detected_count는 별도 추적 필요
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