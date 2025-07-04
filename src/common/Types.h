#ifndef TYPES_H
#define TYPES_H

#include <cstdint>
#include <vector>
#include <string>
#include <chrono>

enum class CameraType {
    RGB = 0,
    THERMAL = 1
};

enum class BboxColor {
    GREEN = 0,
    YELLOW,
    RED,
    BLUE,
    NONE
};

enum ClassId {
    CLASS_NORMAL_COW = 0,
    CLASS_FLIP_COW = 1,
    CLASS_NORMAL_COW_SITTING = 2,
    CLASS_HEAT_COW = 3,
    CLASS_LABOR_SIGN_COW = 4,
    CLASS_OVER_TEMP = 5,
    NUM_CLASSES = 6
};

struct BoundingBox {
    int x;
    int y;
    int width;
    int height;
};

struct DetectedObject {
    int classId;
    float confidence;
    BoundingBox bbox;
    BboxColor color;
    bool hasBbox;
};

struct DetectionData {
    uint64_t timestamp;
    uint32_t frameNumber;
    CameraType cameraType;
    std::vector<DetectedObject> objects;
};

// struct CameraConfig {
//     CameraType type;
//     std::string source;
//     std::string inferConfig;
//     std::string encoder;
//     std::string encoder2;
//     std::string snapshot;
//     int width;
//     int height;
//     int fps;
// };

struct SourceConfig {
    std::string protocol;  // "udp", "rtsp", "file"
    int port;
    std::string encoding;  // "h264", "h265"
    int width;
    int height;
    int framerate;
};

struct InferenceConfig {
    bool enabled;
    std::string config_file;
    int scale_width;
    int scale_height;
};

struct EncoderConfig {
    std::string codec;  // "h264", "h265"
    std::string preset;  // "fast", "medium", "slow"
    int bitrate;
    int idr_interval;
};

struct CameraConfig {
    std::string name;
    CameraType type;
    SourceConfig source;
    InferenceConfig inference;
    EncoderConfig encoder;
    
    // 기존 필드들은 deprecated
    std::string inferConfig;  // deprecated
};

struct SystemConfig {
    std::string cameraId;
    int deviceCount;
    int maxStreamCount;
    int streamBasePort;
    std::vector<CameraConfig> cameras;
    std::string snapshotPath;
    int apiPort;
};
#endif // TYPES_H