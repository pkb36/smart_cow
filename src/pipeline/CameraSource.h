#ifndef CAMERA_SOURCE_H
#define CAMERA_SOURCE_H

#include <memory>
#include <string>
#include <gst/gst.h>
#include <gstnvdsmeta.h>
#include <unordered_map>
#include <mutex>
#include "../common/Types.h"
#include "../detection/Detector.h"

class DetectionBuffer;

class CameraSource {
public:
    CameraSource(CameraType type, int index);
    ~CameraSource();
    
    bool init(const CameraConfig& config, GstElement* pipeline);
    
    // 검출 버퍼 접근
    DetectionBuffer* getDetectionBuffer() const { return detectionBuffer_.get(); }
    
    // 동적 피어 관리
    bool addPeerOutput(const std::string& peerId);
    bool removePeerOutput(const std::string& peerId);
    
    // 메인 Tee 접근 (필요시)
    GstElement* getMainTee() const { return elements_.main_tee; }
    
private:
    // 파이프라인 구성
    bool createSourceChain(const CameraConfig& config);
    bool createInferenceChain(const CameraConfig& config);
    bool linkElements(const CameraConfig& config);
    bool addProbes();
    
    // 프로브 콜백
    static GstPadProbeReturn osdSinkPadProbe(GstPad* pad, GstPadProbeInfo* info, gpointer userData);
    
    // 이벤트 처리
    void handleDetectionEvent(const DetectionData& detection);
    
private:
    CameraType type_;
    int index_;
    GstElement* pipeline_;
    
    // 검출 관련
    std::unique_ptr<Detector> detector_;
    std::unique_ptr<DetectionBuffer> detectionBuffer_;
    
    // 설정
    CameraConfig config_;
    
    // GStreamer 요소들 (구조체로 통합 관리)
    struct Elements {
        // 소스 체인
        GstElement* intervideosrc;
        GstElement* converter1;
        GstElement* clockoverlay;
        GstElement* videorate;
        GstElement* capsfilter;
        GstElement* queue1;
        GstElement* tee;
        
        // 추론 체인 (옵션)
        GstElement* queue2;
        GstElement* videoscale;
        GstElement* converter2;
        GstElement* mux;
        GstElement* infer;
        GstElement* nvof;
        GstElement* converter3;
        GstElement* postproc;
        GstElement* osd;
        GstElement* converter4;
        
        // 메인 출력 Tee
        GstElement* main_tee;
    } elements_;
    
    // 피어별 출력 (간소화)
    struct PeerOutput {
        std::string peerId;
        GstElement* queue;
        GstElement* converter;
        GstElement* intervideosink;
        GstPad* teeSrcPad;
        GstPad* queueSinkPad;
    };
    
    std::unordered_map<std::string, std::unique_ptr<PeerOutput>> peerOutputs_;
    mutable std::mutex peerOutputsMutex_;
};

#endif // CAMERA_SOURCE_H