#include "DetectionBuffer.h"
#include "../utils/Logger.h"
#include <algorithm>
#include <chrono>

DetectionBuffer::DetectionBuffer(CameraType cameraType, size_t maxSize)
    : cameraType_(cameraType)
    , maxSize_(maxSize) {
    
    LOG_INFO("Detection buffer created for %s camera (max size: %zu)",
             (cameraType == CameraType::RGB) ? "RGB" : "THERMAL", maxSize);
}

DetectionBuffer::~DetectionBuffer() {
    clear();
}

void DetectionBuffer::addDetection(const DetectionData& detection) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 타임스탬프가 없으면 현재 시간 사용
    DetectionData data = detection;
    if (data.timestamp == 0) {
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        data.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
    }
    
    // 카메라 타입 설정
    data.cameraType = cameraType_;
    
    // 버퍼에 추가
    buffer_.push_back(data);
    
    // 버퍼 크기 제한
    if (buffer_.size() > maxSize_) {
        buffer_.pop_front();
    }
    
    // 오래된 데이터 제거
    removeOldDetections();
    
    LOG_TRACE("Detection added: frame=%u, objects=%zu, buffer_size=%zu",
              data.frameNumber, data.objects.size(), buffer_.size());
}

std::vector<DetectionData> DetectionBuffer::getDetectionsInTimeRange(
    uint64_t startTime, uint64_t endTime) const {
    
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<DetectionData> results;
    
    for (const auto& detection : buffer_) {
        if (detection.timestamp >= startTime && detection.timestamp <= endTime) {
            results.push_back(detection);
        }
    }
    
    LOG_DEBUG("Found %zu detections in time range [%lu - %lu]",
              results.size(), startTime, endTime);
    
    return results;
}

bool DetectionBuffer::getLatestDetection(DetectionData& detection) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (buffer_.empty()) {
        return false;
    }
    
    detection = buffer_.back();
    return true;
}

void DetectionBuffer::clearOldDetections() {
    std::lock_guard<std::mutex> lock(mutex_);
    removeOldDetections();
}

size_t DetectionBuffer::getBufferSize() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.size();
}

void DetectionBuffer::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.clear();
    LOG_INFO("Detection buffer cleared");
}

void DetectionBuffer::removeOldDetections() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    uint64_t currentTime = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
    uint64_t cutoffTime = currentTime - BUFFER_DURATION_NS;
    
    // 오래된 데이터 제거
    auto it = buffer_.begin();
    while (it != buffer_.end() && it->timestamp < cutoffTime) {
        it = buffer_.erase(it);
    }
    
    LOG_TRACE("Removed old detections, buffer size: %zu", buffer_.size());
}