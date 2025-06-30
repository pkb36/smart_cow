# Makefile for WebRTC Camera System

# 컴파일러 설정
CXX = g++
CC = gcc

# C++ 표준
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -g -fPIC

# DeepStream 경로 찾기
DEEPSTREAM_PATH := $(shell \
    if [ -d "/opt/nvidia/deepstream/deepstream" ]; then \
        echo "/opt/nvidia/deepstream/deepstream"; \
    elif [ -d "/opt/nvidia/deepstream/deepstream-6.2" ]; then \
        echo "/opt/nvidia/deepstream/deepstream-6.2"; \
    elif [ -d "/opt/nvidia/deepstream/deepstream-6.1" ]; then \
        echo "/opt/nvidia/deepstream/deepstream-6.1"; \
    elif [ -d "/opt/nvidia/deepstream/deepstream-6.0" ]; then \
        echo "/opt/nvidia/deepstream/deepstream-6.0"; \
    else \
        echo ""; \
    fi)

ifeq ($(DEEPSTREAM_PATH),)
    $(error DeepStream not found)
endif

$(info Using DeepStream at: $(DEEPSTREAM_PATH))

# pkg-config를 사용한 라이브러리 설정
GST_CFLAGS := $(shell pkg-config --cflags gstreamer-1.0 gstreamer-base-1.0 gstreamer-video-1.0 gstreamer-app-1.0)
GST_LIBS := $(shell pkg-config --libs gstreamer-1.0 gstreamer-base-1.0 gstreamer-video-1.0 gstreamer-app-1.0)

SOUP_CFLAGS := $(shell pkg-config --cflags libsoup-2.4)
SOUP_LIBS := $(shell pkg-config --libs libsoup-2.4)

JSON_CFLAGS := $(shell pkg-config --cflags json-glib-1.0)
JSON_LIBS := $(shell pkg-config --libs json-glib-1.0)

CURL_CFLAGS := $(shell curl-config --cflags)
CURL_LIBS := $(shell curl-config --libs)

# Include 경로
INCLUDES = -I./src \
           $(GST_CFLAGS) \
           $(SOUP_CFLAGS) \
           $(JSON_CFLAGS) \
           $(CURL_CFLAGS) \
           -I$(DEEPSTREAM_PATH)/sources/includes

# 라이브러리 경로 및 링크
LDFLAGS = -L$(DEEPSTREAM_PATH)/lib \
          -Wl,-rpath,$(DEEPSTREAM_PATH)/lib

LIBS = $(GST_LIBS) \
       $(SOUP_LIBS) \
       $(JSON_LIBS) \
       $(CURL_LIBS) \
       -lgstpbutils-1.0 \
       -lgstsdp-1.0 \
       -lgstwebrtc-1.0 \
       -lnvdsgst_meta \
       -lnvds_meta \
       -lnvds_utils \
       -lnvdsgst_helper \
       -lnvbufsurface \
       -lnvbufsurftransform \
       -lpthread -lm -lrt -ldl

# 소스 파일
SRCS = src/main.cpp \
       src/pipeline/Pipeline.cpp \
       src/pipeline/CameraSource.cpp \
       src/pipeline/StreamOutput.cpp \
       src/detection/Detector.cpp \
       src/detection/Tracker.cpp \
       src/detection/DetectionBuffer.cpp \
       src/control/PTZController.cpp \
       src/control/CommandPipe.cpp \
       src/signaling/SignalingClient.cpp \
       src/webrtc/WebRTCSenderProcess.cpp \
       src/webrtc/PeerManager.cpp \
       src/api/ApiServer.cpp \
       src/utils/Config.cpp \
       src/utils/Logger.cpp \
       src/utils/DeviceSetting.cpp \
       src/utils/SerialComm.cpp \
       src/utils/SocketComm.cpp \
       src/utils/ProcessManager.cpp \
       src/utils/CurlClient.cpp

# 오브젝트 파일
OBJS = $(SRCS:.cpp=.o)

# 타겟
TARGET = WebRTCCamera

# 기본 타겟
all: $(TARGET)

# 실행 파일 생성
$(TARGET): $(OBJS)
	$(CXX) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

# .cpp -> .o 컴파일 규칙
%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# 의존성 파일 생성
DEPS = $(OBJS:.o=.d)

-include $(DEPS)

%.d: %.cpp
	@mkdir -p $(dir $@)
	@$(CXX) $(CXXFLAGS) $(INCLUDES) -MM -MT '$(@:.d=.o)' $< > $@

# 청소
clean:
	rm -f $(OBJS) $(DEPS) $(TARGET)
	find src -name "*.o" -delete
	find src -name "*.d" -delete

# 설치
install: $(TARGET)
	install -m 755 $(TARGET) /usr/local/bin/
	@echo "Installed $(TARGET) to /usr/local/bin/"

# 디버그 정보 출력
info:
	@echo "CXX: $(CXX)"
	@echo "CXXFLAGS: $(CXXFLAGS)"
	@echo "DEEPSTREAM_PATH: $(DEEPSTREAM_PATH)"
	@echo "INCLUDES: $(INCLUDES)"
	@echo "LIBS: $(LIBS)"
	@echo "SRCS: $(SRCS)"
	@echo "OBJS: $(OBJS)"

.PHONY: all clean install info