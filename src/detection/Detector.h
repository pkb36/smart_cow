#ifndef DETECTOR_H
#define DETECTOR_H

#include <memory>
#include <functional>
#include <gst/gst.h>
#include <iostream>
#include <nvdsmeta.h>

#include "../common/Types.h"

class Detector {
public:
    using DetectionCallback = std::function<void(const DetectionData&)>;
    
    Detector(CameraType cameraType);
    ~Detector();
    
    bool init(const std::string& configFile);
    void setDetectionCallback(DetectionCallback callback);
    
    // DeepStream 메타데이터 처리
    void processBatchMeta(NvDsBatchMeta* batchMeta, uint32_t frameNumber);
    
    // 설정
    void setEnabled(bool enabled);
    bool isEnabled() const;
    void setInterval(int interval);
    
private:
    DetectedObject convertToDetectedObject(NvDsObjectMeta* objMeta);
    BboxColor determineColor(int classId, const DetectedObject& obj);
    
private:
    CameraType cameraType_;
    DetectionCallback callback_;
    bool enabled_;
    int interval_;
    std::string configFile_;
};

#endif // DETECTOR_H