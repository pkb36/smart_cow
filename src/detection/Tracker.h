#ifndef TRACKER_H
#define TRACKER_H

#include <memory>
#include <unordered_map>
#include <chrono>
#include "../common/Types.h"

struct TrackedObject {
    uint64_t trackId;
    int classId;
    BoundingBox lastBbox;
    std::chrono::steady_clock::time_point lastSeen;
    int detectionCount;
    float avgConfidence;
    
    // 추적 상태
    bool isActive;
    int missedFrames;
};

class Tracker {
public:
    Tracker();
    ~Tracker();
    
    void updateTrack(uint64_t trackId, const DetectedObject& detection);
    void processFrame();  // 프레임마다 호출하여 오래된 트랙 제거
    
    const TrackedObject* getTrack(uint64_t trackId) const;
    std::vector<uint64_t> getActiveTrackIds() const;
    
    // 설정
    void setMaxMissedFrames(int frames);
    void setMinDetectionCount(int count);
    
private:
    void removeInactiveTracks();
    
private:
    std::unordered_map<uint64_t, TrackedObject> tracks_;
    int maxMissedFrames_;
    int minDetectionCount_;
    std::chrono::steady_clock::time_point lastProcessTime_;
};

#endif // TRACKER_H