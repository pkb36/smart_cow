#include "Tracker.h"
#include "../utils/Logger.h"
#include <algorithm>

Tracker::Tracker()
    : maxMissedFrames_(10)
    , minDetectionCount_(3)
    , lastProcessTime_(std::chrono::steady_clock::now()) {
    
    LOG_INFO("Tracker initialized (max_missed=%d, min_detections=%d)",
             maxMissedFrames_, minDetectionCount_);
}

Tracker::~Tracker() {
    tracks_.clear();
}

void Tracker::updateTrack(uint64_t trackId, const DetectedObject& detection) {
    auto now = std::chrono::steady_clock::now();
    
    auto it = tracks_.find(trackId);
    if (it == tracks_.end()) {
        // 새로운 트랙 생성
        TrackedObject track;
        track.trackId = trackId;
        track.classId = detection.classId;
        track.lastBbox = detection.bbox;
        track.lastSeen = now;
        track.detectionCount = 1;
        track.avgConfidence = detection.confidence;
        track.isActive = true;
        track.missedFrames = 0;
        
        tracks_[trackId] = track;
        
        LOG_DEBUG("New track created: id=%lu, class=%d", trackId, detection.classId);
    } else {
        // 기존 트랙 업데이트
        TrackedObject& track = it->second;
        
        // 평균 신뢰도 업데이트
        track.avgConfidence = (track.avgConfidence * track.detectionCount + detection.confidence) / 
                             (track.detectionCount + 1);
        
        track.classId = detection.classId;
        track.lastBbox = detection.bbox;
        track.lastSeen = now;
        track.detectionCount++;
        track.missedFrames = 0;
        track.isActive = true;
        
        LOG_TRACE("Track updated: id=%lu, detections=%d, avg_conf=%.2f",
                  trackId, track.detectionCount, track.avgConfidence);
    }
}

void Tracker::processFrame() {
    auto now = std::chrono::steady_clock::now();
    
    // 모든 트랙의 missed frames 증가
    for (auto& pair : tracks_) {
        TrackedObject& track = pair.second;
        
        // 이번 프레임에서 업데이트되지 않은 트랙
        auto timeSinceLastSeen = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - track.lastSeen).count();
        
        // 약 33ms = 1 프레임 (30fps 기준)
        if (timeSinceLastSeen > 33) {
            track.missedFrames++;
            
            if (track.missedFrames > maxMissedFrames_) {
                track.isActive = false;
                LOG_DEBUG("Track %lu marked inactive (missed %d frames)",
                          track.trackId, track.missedFrames);
            }
        }
    }
    
    // 비활성 트랙 제거
    removeInactiveTracks();
    
    lastProcessTime_ = now;
}

const TrackedObject* Tracker::getTrack(uint64_t trackId) const {
    auto it = tracks_.find(trackId);
    if (it != tracks_.end()) {
        return &it->second;
    }
    return nullptr;
}

std::vector<uint64_t> Tracker::getActiveTrackIds() const {
    std::vector<uint64_t> activeIds;
    
    for (const auto& pair : tracks_) {
        if (pair.second.isActive && 
            pair.second.detectionCount >= minDetectionCount_) {
            activeIds.push_back(pair.first);
        }
    }
    
    return activeIds;
}

void Tracker::setMaxMissedFrames(int frames) {
    maxMissedFrames_ = frames;
    LOG_INFO("Max missed frames set to: %d", maxMissedFrames_);
}

void Tracker::setMinDetectionCount(int count) {
    minDetectionCount_ = count;
    LOG_INFO("Min detection count set to: %d", minDetectionCount_);
}

void Tracker::removeInactiveTracks() {
    auto it = tracks_.begin();
    while (it != tracks_.end()) {
        if (!it->second.isActive) {
            LOG_DEBUG("Removing inactive track: id=%lu, total_detections=%d",
                      it->first, it->second.detectionCount);
            it = tracks_.erase(it);
        } else {
            ++it;
        }
    }
}