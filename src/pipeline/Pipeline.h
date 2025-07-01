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
    CameraSource* getCamera(int index) const {
        if (index >= 0 && index < static_cast<int>(cameras_.size())) {
            return cameras_[index].get();
        }
        return nullptr;
    }
    
    int getCameraCount() const {
        return cameras_.size();
    }

    bool addElementSafely(GstElement* element) {
        if (!element || !pipeline_) return false;
        
        GstState state;
        gst_element_get_state(pipeline_, &state, nullptr, 0);
        
        // PLAYING 상태에서는 먼저 READY로 설정
        if (state == GST_STATE_PLAYING || state == GST_STATE_PAUSED) {
            gst_element_set_state(element, GST_STATE_READY);
        }
        
        // 파이프라인에 추가
        if (!gst_bin_add(GST_BIN(pipeline_), element)) {
            return false;
        }
        
        // 부모와 상태 동기화
        if (state == GST_STATE_PLAYING || state == GST_STATE_PAUSED) {
            gst_element_sync_state_with_parent(element);
        }
        
        return true;
    }
    
    // 동적으로 요소를 제거하는 안전한 방법
    bool removeElementSafely(GstElement* element) {
        if (!element || !pipeline_) return false;
        
        // NULL 상태로 변경
        gst_element_set_state(element, GST_STATE_NULL);
        
        // 파이프라인에서 제거
        return gst_bin_remove(GST_BIN(pipeline_), element);
    }
    
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