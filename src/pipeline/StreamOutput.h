#ifndef STREAM_OUTPUT_H
#define STREAM_OUTPUT_H

#include <memory>
#include <gst/gst.h>
#include "../common/Types.h"

class StreamOutput {
public:
    enum StreamType {
        MAIN_STREAM = 0,
        SUB_STREAM = 1
    };
    
    StreamOutput(int cameraIndex, int streamIndex, StreamType type);
    ~StreamOutput();
    
    bool init(GstElement* pipeline, GstElement* tee, int basePort);
    int getPort() const;
    
private:
    bool createElements();
    bool linkElements(GstElement* tee);
    
private:
    int cameraIndex_;
    int streamIndex_;
    StreamType type_;
    int port_;
    
    // GStreamer 요소들
    GstElement* queue_;
    GstElement* udpSink_;
    GstPad* teeSrcPad_;
    GstPad* queueSinkPad_;
};

#endif // STREAM_OUTPUT_H