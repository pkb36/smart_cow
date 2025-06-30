cmake_minimum_required(VERSION 3.10)
project(WebRTCCamera)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# DeepStream 경로 찾기
set(DEEPSTREAM_PATH "")
foreach(path 
    "/opt/nvidia/deepstream/deepstream"
    "/opt/nvidia/deepstream/deepstream-6.2"
    "/opt/nvidia/deepstream/deepstream-6.1"
    "/opt/nvidia/deepstream/deepstream-6.0")
    if(EXISTS ${path})
        set(DEEPSTREAM_PATH ${path})
        break()
    endif()
endforeach()

if(NOT DEEPSTREAM_PATH)
    message(FATAL_ERROR "DeepStream not found")
else()
    message(STATUS "Found DeepStream at: ${DEEPSTREAM_PATH}")
endif()

# 소스 파일
file(GLOB_RECURSE SOURCES 
    "src/*.cpp"
)

# 실행 파일 생성
add_executable(${PROJECT_NAME} ${SOURCES})

# pkg-config 모듈 찾기
find_package(PkgConfig REQUIRED)

# 필수 라이브러리 찾기
pkg_search_module(GSTREAMER REQUIRED gstreamer-1.0)
pkg_search_module(GSTREAMER_BASE REQUIRED gstreamer-base-1.0)
pkg_search_module(GSTREAMER_VIDEO REQUIRED gstreamer-video-1.0)
pkg_search_module(SOUP REQUIRED libsoup-2.4)
pkg_search_module(JSON_GLIB REQUIRED json-glib-1.0)

# CURL
find_package(CURL REQUIRED)

# Include 디렉토리
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${GSTREAMER_INCLUDE_DIRS}
    ${GSTREAMER_BASE_INCLUDE_DIRS}
    ${GSTREAMER_VIDEO_INCLUDE_DIRS}
    ${SOUP_INCLUDE_DIRS}
    ${JSON_GLIB_INCLUDE_DIRS}
    ${CURL_INCLUDE_DIRS}
    /opt/nvidia/deepstream/deepstream-6.2/sources/includes
)

# 라이브러리 디렉토리
target_link_directories(${PROJECT_NAME} PRIVATE
    /opt/nvidia/deepstream/deepstream-6.2/lib
)

# 링크 라이브러리
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${GSTREAMER_LIBRARIES}
    ${GSTREAMER_BASE_LIBRARIES}
    ${GSTREAMER_VIDEO_LIBRARIES}
    ${SOUP_LIBRARIES}
    ${JSON_GLIB_LIBRARIES}
    ${CURL_LIBRARIES}
    gstapp-1.0
    gstpbutils-1.0
    gstsdp-1.0
    gstwebrtc-1.0
    nvdsgst_meta
    nvds_meta
    nvds_utils
    nvdsgst_helper
    nvbufsurface
    nvbufsurftransform
    pthread
    m
    rt
    dl
)

# 컴파일 플래그
add_compile_options(-Wall -Wextra -g -O2)

# DeepStream RPATH
set_target_properties(${PROJECT_NAME} PROPERTIES
    INSTALL_RPATH "/opt/nvidia/deepstream/deepstream-6.2/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
)