#include "StreamOutput.h"
#include "../utils/Logger.h"
#include <gst/gst.h>

StreamOutput::StreamOutput(int cameraIndex, int streamIndex, StreamType type)
    : cameraIndex_(cameraIndex)
    , streamIndex_(streamIndex)
    , type_(type)
    , port_(0)
    , queue_(nullptr)
    , udpSink_(nullptr)
    , teeSrcPad_(nullptr)
    , queueSinkPad_(nullptr) {
    
    LOG_DEBUG("StreamOutput created: camera=%d, stream=%d, type=%s",
              cameraIndex, streamIndex, (type == MAIN_STREAM) ? "MAIN" : "SUB");
}

StreamOutput::~StreamOutput() {
    // 패드 연결 해제
    if (teeSrcPad_ && queueSinkPad_) {
        gst_pad_unlink(teeSrcPad_, queueSinkPad_);
    }
    
    // 패드 참조 해제
    if (teeSrcPad_) {
        gst_element_release_request_pad(gst_pad_get_parent_element(teeSrcPad_), teeSrcPad_);
        gst_object_unref(teeSrcPad_);
    }
    
    if (queueSinkPad_) {
        gst_object_unref(queueSinkPad_);
    }
    
    // 요소들은 파이프라인이 관리하므로 여기서는 unref하지 않음
}

bool StreamOutput::init(GstElement* pipeline, GstElement* tee, int basePort) {
    if (!pipeline || !tee) {
        LOG_ERROR("Invalid pipeline or tee element");
        return false;
    }
    
    // 포트 계산
    // main stream: base_port + (device_cnt * stream_index) + camera_index
    // sub stream: base_port + 100 + (device_cnt * stream_index) + camera_index
    const int STREAM_PORT_OFFSET = 100;
    const int DEVICE_COUNT = 2;  // 고정값 사용 (RGB + Thermal)
    
    if (type_ == MAIN_STREAM) {
        port_ = basePort + (DEVICE_COUNT * streamIndex_) + cameraIndex_;
    } else {
        port_ = basePort + STREAM_PORT_OFFSET + (DEVICE_COUNT * streamIndex_) + cameraIndex_;
    }
    
    // 요소 생성
    if (!createElements()) {
        return false;
    }
    
    // 파이프라인에 추가
    gst_bin_add_many(GST_BIN(pipeline), queue_, udpSink_, nullptr);
    
    // 내부 요소 연결
    if (!gst_element_link(queue_, udpSink_)) {
        LOG_ERROR("Failed to link queue to udpsink");
        return false;
    }
    
    // Tee와 연결
    if (!linkElements(tee)) {
        return false;
    }
    
    LOG_INFO("StreamOutput initialized: camera=%d, stream=%d, port=%d",
             cameraIndex_, streamIndex_, port_);
    
    return true;
}

int StreamOutput::getPort() const {
    return port_;
}

bool StreamOutput::createElements() {
    gchar elementName[64];
    
    // Queue 생성
    g_snprintf(elementName, sizeof(elementName), 
               "stream_queue_%d_%d_%s", cameraIndex_, streamIndex_,
               (type_ == MAIN_STREAM) ? "main" : "sub");
    
    queue_ = gst_element_factory_make("queue", elementName);
    if (!queue_) {
        LOG_ERROR("Failed to create queue element");
        return false;
    }
    
    // Queue 속성 설정
    g_object_set(queue_,
                 "max-size-buffers", 5,
                 "leaky", 2,  // downstream
                 nullptr);
    
    // UDP Sink 생성
    g_snprintf(elementName, sizeof(elementName),
               "udpsink_%d_%d_%s", cameraIndex_, streamIndex_,
               (type_ == MAIN_STREAM) ? "main" : "sub");
    
    udpSink_ = gst_element_factory_make("udpsink", elementName);
    if (!udpSink_) {
        LOG_ERROR("Failed to create udpsink element");
        return false;
    }
    
    // UDP Sink 속성 설정
    g_object_set(udpSink_,
                 "host", "127.0.0.1",
                 "port", port_,
                 "sync", FALSE,
                 "async", FALSE,
                 nullptr);
    
    LOG_DEBUG("Created stream output elements for port %d", port_);
    
    return true;
}

bool StreamOutput::linkElements(GstElement* tee) {
    // Tee의 src 패드 요청
    GstPadTemplate* padTemplate = gst_element_class_get_pad_template(
        GST_ELEMENT_GET_CLASS(tee), "src_%u");
    
    if (!padTemplate) {
        LOG_ERROR("Failed to get pad template from tee");
        return false;
    }
    
    teeSrcPad_ = gst_element_request_pad(tee, padTemplate, nullptr, nullptr);
    if (!teeSrcPad_) {
        LOG_ERROR("Failed to request src pad from tee");
        return false;
    }
    
    // Queue의 sink 패드 가져오기
    queueSinkPad_ = gst_element_get_static_pad(queue_, "sink");
    if (!queueSinkPad_) {
        LOG_ERROR("Failed to get sink pad from queue");
        gst_element_release_request_pad(tee, teeSrcPad_);
        gst_object_unref(teeSrcPad_);
        teeSrcPad_ = nullptr;
        return false;
    }
    
    // 패드 연결
    GstPadLinkReturn linkRet = gst_pad_link(teeSrcPad_, queueSinkPad_);
    if (linkRet != GST_PAD_LINK_OK) {
        LOG_ERROR("Failed to link tee to queue: %d", linkRet);
        gst_element_release_request_pad(tee, teeSrcPad_);
        gst_object_unref(teeSrcPad_);
        gst_object_unref(queueSinkPad_);
        teeSrcPad_ = nullptr;
        queueSinkPad_ = nullptr;
        return false;
    }
    
    LOG_DEBUG("Successfully linked tee to stream output");
    
    return true;
}