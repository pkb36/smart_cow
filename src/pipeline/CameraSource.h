#ifndef CAMERA_SOURCE_H
#define CAMERA_SOURCE_H

#include <memory>
#include <string>
#include <gst/gst.h>
#include <gstnvdsmeta.h>

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
    
private:
    bool createElements(const CameraConfig& config);
    bool parseAndCreateSource(const std::string& sourceStr);
    bool addElementsToPipeline(GstElement* pipeline);
    bool linkInternalElements();
    bool setupProbes();
    BboxColor determineObjectColor(NvDsObjectMeta* objMeta);
    
private:
    CameraType type_;
    int index_;
    
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
};

#endif // CAMERA_SOURCE_H