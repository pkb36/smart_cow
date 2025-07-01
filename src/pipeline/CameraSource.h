#ifndef CAMERA_SOURCE_H
#define CAMERA_SOURCE_H

#include <memory>
#include <string>
#include <gst/gst.h>
#include <gstnvdsmeta.h>
#include <unordered_map>
#include <mutex>

#include "../common/Types.h"

class DetectionBuffer;

class CameraSource {
public:
    CameraSource(CameraType type, int index);
    ~CameraSource();
    
    bool init(const CameraConfig& config, GstElement* pipeline);
    bool link();
    
    // 요소 접근
    GstElement* getSourceElement() const;
    GstElement* getTeeElement() const;
    GstElement* getOutputElement() const;
    
    // 프로브 콜백
    static GstPadProbeReturn osdSinkPadProbe(GstPad* pad, GstPadProbeInfo* info, gpointer userData);
    
    // 검출 버퍼 접근
    DetectionBuffer* getDetectionBuffer() const;
    bool createPipeline(const CameraConfig& config);
    bool createSourceChain(const CameraConfig& config);
    bool createInferenceChain(const CameraConfig& config);
    bool createEncoderChain(const CameraConfig& config);
    bool linkElements(const CameraConfig& config);
    bool addProbes();

    // 동적 피어 관리
    bool addPeerOutput(const std::string& peerId, int streamPort);
    bool removePeerOutput(const std::string& peerId);
    GstElement* getMainTee() const { return elements_.main_tee; }

private:
    bool parseAndCreateSource(const std::string& sourceStr);
    bool addElementsToPipeline(GstElement* pipeline);
    bool linkInternalElements();
    bool setupProbes();
    BboxColor determineObjectColor(NvDsObjectMeta* objMeta);
    void saveBufferAsImage(GstBuffer* buffer, GstCaps* caps, int frameNumber);
    void convertToJpeg(const char* rawFile, int width, int height, GstVideoFormat format, int frameNumber);
private:
    CameraType type_;
    int index_;
    GstElement* pipeline_; 
    
    // GStreamer 요소들
    GstElement* source_;
    GstElement* decoder_;
    GstElement* converter_;
    GstElement* clockOverlay_;
    GstElement* tee_;
    GstElement* queue_;
    GstElement* scale_;
    GstElement* mux_;
    GstElement* infer_;
    GstElement* tracker_;
    GstElement* postproc_;
    GstElement* osd_;
    GstElement* outputConverter_;
    
    // 검출 버퍼
    std::unique_ptr<DetectionBuffer> detectionBuffer_;
    
    // 설정
    CameraConfig config_;

    struct Elements {
        // 소스 체인
        GstElement* udpsrc;
        GstElement* rtpdepay;
        GstElement* parser;
        GstElement* decoder;
        GstElement* converter1;
        GstElement* clockoverlay;
        GstElement* videorate;
        GstElement* capsfilter;
        GstElement* queue1;
        GstElement* tee;
        
        // 추론 체인
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
        
        // 인코더 체인
        GstElement* queue3;
        GstElement* encoder_convert;
        GstElement* encoder;
        GstElement* payloader;
        GstElement* queue4;
        GstElement* sink;

        GstElement* main_tee;
        GstElement* udpsink;
        GstElement* rtppay;
    } elements_;

    struct PeerOutput {
        std::string peerId;
        int port;
        GstElement* queue;
        GstElement* udpsink;
        GstPad* teeSrcPad;
        GstPad* queueSinkPad;
    };
    
    std::unordered_map<std::string, std::unique_ptr<PeerOutput>> peerOutputs_;
    mutable std::mutex peerOutputsMutex_;
};

#endif // CAMERA_SOURCE_H