#ifndef DETECTION_BUFFER_H
#define DETECTION_BUFFER_H

#include <memory>
#include <vector>
#include <mutex>
#include <deque>
#include "../common/Types.h"

class DetectionBuffer {
public:
    static constexpr size_t DEFAULT_BUFFER_SIZE = 3600;  // 120초 * 30fps
    static constexpr uint64_t BUFFER_DURATION_NS = 120ULL * 1000000000ULL;  // 120초
    
    DetectionBuffer(CameraType cameraType, size_t maxSize = DEFAULT_BUFFER_SIZE);
    ~DetectionBuffer();
    
    // 검출 데이터 추가
    void addDetection(const DetectionData& detection);
    
    // 검출 데이터 조회
    std::vector<DetectionData> getDetectionsInTimeRange(uint64_t startTime, uint64_t endTime) const;
    bool getLatestDetection(DetectionData& detection) const;
    
    // 버퍼 관리
    void clearOldDetections();
    size_t getBufferSize() const;
    void clear();
    
private:
    void removeOldDetections();
    
private:
    CameraType cameraType_;
    size_t maxSize_;
    
    mutable std::mutex mutex_;
    std::deque<DetectionData> buffer_;
};

#endif // DETECTION_BUFFER_H