#ifndef PIPELINE_H
#define PIPELINE_H

#include <memory>
#include <vector>
#include <gst/gst.h>
#include "../common/Types.h"

class Config;
class CameraSource;
class StreamOutput;

class Pipeline {
public:
    Pipeline();
    ~Pipeline();
    
    bool init(const Config& config);
    bool start();
    void stop();
    bool isRunning() const;
    
    // 파이프라인 상태
    GstState getState() const;
    
    // 메시지 처리
    void handleBusMessage(GstMessage* message);
    
    // 카메라 접근
    CameraSource* getCamera(int index) const;
    
private:
    bool createPipeline();
    bool setupCameras(const Config& config);
    bool setupOutputs(const Config& config);
    bool linkElements();
    
    static gboolean busCallback(GstBus* bus, GstMessage* message, gpointer data);
    
private:
    GstElement* pipeline_;
    GstBus* bus_;
    guint busWatchId_;
    
    std::vector<std::unique_ptr<CameraSource>> cameras_;
    std::vector<std::unique_ptr<StreamOutput>> outputs_;
    
    bool isRunning_;
    std::unique_ptr<Config> config_;
};

#endif // PIPELINE_H