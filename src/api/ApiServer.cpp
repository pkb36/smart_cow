#include "ApiServer.h"
#include "../detection/DetectionBuffer.h"
#include "../utils/Logger.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

ApiServer::ApiServer(int port)
    : port_(port)
    , serverSocket_(-1)
    , running_(false) {
    
    // 기본 라우트 등록
    addRoute("POST", "/api/get_detections", 
             [this](const Request& req) { return handleGetDetections(req); });
    addRoute("POST", "/api/get_latest", 
             [this](const Request& req) { return handleGetLatest(req); });
    
    LOG_INFO("API Server created on port %d", port);
}

ApiServer::~ApiServer() {
    stop();
}

bool ApiServer::start() {
    if (running_) {
        LOG_WARN("API Server already running");
        return true;
    }
    
    // 서버 소켓 생성
    serverSocket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket_ < 0) {
        LOG_ERROR("Failed to create server socket: %s", strerror(errno));
        return false;
    }
    
    // SO_REUSEADDR 설정
    int opt = 1;
    if (setsockopt(serverSocket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        LOG_ERROR("Failed to set socket options: %s", strerror(errno));
        close(serverSocket_);
        serverSocket_ = -1;
        return false;
    }
    
    // 바인드
    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(port_);
    
    if (bind(serverSocket_, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        LOG_ERROR("Failed to bind to port %d: %s", port_, strerror(errno));
        close(serverSocket_);
        serverSocket_ = -1;
        return false;
    }
    
    // 리슨
    if (listen(serverSocket_, 10) < 0) {
        LOG_ERROR("Failed to listen: %s", strerror(errno));
        close(serverSocket_);
        serverSocket_ = -1;
        return false;
    }
    
    // 서버 스레드 시작
    running_ = true;
    serverThread_ = std::thread(&ApiServer::serverThread, this);
    
    LOG_INFO("API Server started on port %d", port_);
    return true;
}

void ApiServer::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    
    // 서버 소켓 닫기
    if (serverSocket_ >= 0) {
        shutdown(serverSocket_, SHUT_RDWR);
        close(serverSocket_);
        serverSocket_ = -1;
    }
    
    // 스레드 종료 대기
    if (serverThread_.joinable()) {
        serverThread_.join();
    }
    
    LOG_INFO("API Server stopped");
}

bool ApiServer::isRunning() const {
    return running_;
}

void ApiServer::registerDetectionBuffer(CameraType type, DetectionBuffer* buffer) {
    if (buffer) {
        if (static_cast<size_t>(type) >= detectionBuffers_.size()) {
            detectionBuffers_.resize(static_cast<size_t>(type) + 1, nullptr);
        }
        detectionBuffers_[static_cast<size_t>(type)] = buffer;
        LOG_INFO("Registered detection buffer for %s camera",
                 (type == CameraType::RGB) ? "RGB" : "THERMAL");
    }
}

void ApiServer::addRoute(const std::string& method, const std::string& path, 
                        RequestHandler handler) {
    std::string key = method + ":" + path;
    routes_[key] = handler;
    LOG_DEBUG("Added route: %s %s", method.c_str(), path.c_str());
}

void ApiServer::serverThread() {
    while (running_) {
        struct sockaddr_in clientAddr;
        socklen_t clientLen = sizeof(clientAddr);
        
        // 클라이언트 연결 수락
        int clientSocket = accept(serverSocket_, (struct sockaddr*)&clientAddr, &clientLen);
        if (clientSocket < 0) {
            if (running_ && errno != EINTR) {
                LOG_ERROR("Failed to accept connection: %s", strerror(errno));
            }
            continue;
        }
        
        // 클라이언트 처리 (별도 스레드)
        std::thread clientThread(&ApiServer::handleClient, this, clientSocket);
        clientThread.detach();
    }
}

void ApiServer::handleClient(int clientSocket) {
    char buffer[4096];
    std::string requestData;
    
    // HTTP 요청 읽기
    while (true) {
        ssize_t bytesRead = recv(clientSocket, buffer, sizeof(buffer) - 1, 0);
        if (bytesRead <= 0) {
            break;
        }
        
        buffer[bytesRead] = '\0';
        requestData += buffer;
        
        // 헤더 종료 확인
        if (requestData.find("\r\n\r\n") != std::string::npos) {
            break;
        }
    }
    
    if (requestData.empty()) {
        close(clientSocket);
        return;
    }
    
    // 요청 파싱
    Request request = parseRequest(requestData);
    
    // Content-Length 확인 및 body 읽기
    size_t headerEnd = requestData.find("\r\n\r\n");
    if (headerEnd != std::string::npos) {
        std::string headers = requestData.substr(0, headerEnd);
        size_t contentLengthPos = headers.find("Content-Length:");
        
        if (contentLengthPos != std::string::npos) {
            size_t valueStart = contentLengthPos + 15;
            size_t valueEnd = headers.find("\r\n", valueStart);
            int contentLength = std::stoi(headers.substr(valueStart, valueEnd - valueStart));
            
            // 이미 읽은 body 부분
            request.body = requestData.substr(headerEnd + 4);
            
            // 나머지 body 읽기
            while (static_cast<int>(request.body.length()) < contentLength) {
                ssize_t bytesRead = recv(clientSocket, buffer, sizeof(buffer) - 1, 0);
                if (bytesRead <= 0) break;
                request.body.append(buffer, bytesRead);
            }
        }
    }
    
    // 라우트 찾기 및 처리
    std::string routeKey = request.method + ":" + request.path;
    Response response;
    
    auto it = routes_.find(routeKey);
    if (it != routes_.end()) {
        response = it->second(request);
    } else {
        response = handleNotFound(request);
    }
    
    // 응답 전송
    std::string responseStr = buildResponse(response);
    send(clientSocket, responseStr.c_str(), responseStr.length(), 0);
    
    close(clientSocket);
}

ApiServer::Request ApiServer::parseRequest(const std::string& rawRequest) {
    Request request;
    std::istringstream stream(rawRequest);
    std::string line;
    
    // 첫 줄 파싱 (METHOD PATH HTTP/1.1)
    if (std::getline(stream, line)) {
        std::istringstream lineStream(line);
        lineStream >> request.method >> request.path;
    }
    
    return request;
}

std::string ApiServer::buildResponse(const Response& response) {
    std::ostringstream oss;
    
    // 상태 줄
    oss << "HTTP/1.1 " << response.statusCode << " ";
    switch (response.statusCode) {
        case 200: oss << "OK"; break;
        case 404: oss << "Not Found"; break;
        case 500: oss << "Internal Server Error"; break;
        default: oss << "Unknown"; break;
    }
    oss << "\r\n";
    
    // 헤더
    oss << "Content-Type: " << response.contentType << "\r\n";
    oss << "Content-Length: " << response.body.length() << "\r\n";
    oss << "Access-Control-Allow-Origin: *\r\n";
    oss << "\r\n";
    
    // 바디
    oss << response.body;
    
    return oss.str();
}

ApiServer::Response ApiServer::handleGetDetections(const Request& request) {
    Response response;
    response.contentType = "application/json";
    
    try {
        // 요청 파싱
        json requestJson = json::parse(request.body);
        
        std::string camera = requestJson.value("camera", "");
        std::string startTime = requestJson.value("start_time", "");
        std::string endTime = requestJson.value("end_time", "");
        
        // 카메라 타입 결정
        CameraType camType;
        if (camera == "RGB_Camera") {
            camType = CameraType::RGB;
        } else if (camera == "Thermal_Camera") {
            camType = CameraType::THERMAL;
        } else {
            throw std::runtime_error("Invalid camera type");
        }
        
        // 버퍼 확인
        if (static_cast<size_t>(camType) >= detectionBuffers_.size() || 
            !detectionBuffers_[static_cast<size_t>(camType)]) {
            throw std::runtime_error("Detection buffer not available");
        }
        
        // 시간 변환 (ISO 8601 -> timestamp)
        uint64_t startTs = 0;
        uint64_t endTs = UINT64_MAX;
        
        if (!startTime.empty()) {
            startTs = parseISOTime(startTime);
        }
        if (!endTime.empty()) {
            endTs = parseISOTime(endTime);
        }
        
        // 검출 데이터 조회
        auto detections = detectionBuffers_[static_cast<size_t>(camType)]->
                         getDetectionsInTimeRange(startTs, endTs);
        
        // JSON 응답 생성
        json responseJson;
        responseJson["status"] = "success";
        responseJson["detections"] = json::array();
        
        for (const auto& detection : detections) {
            json detJson;
            detJson["timestamp"] = detection.timestamp;
            detJson["frame_number"] = detection.frameNumber;
            detJson["camera"] = camera;
            detJson["objects"] = json::array();
            
            for (const auto& obj : detection.objects) {
                json objJson;
                objJson["class_id"] = obj.classId;
                objJson["confidence"] = obj.confidence;
                objJson["bbox"] = {obj.bbox.x, obj.bbox.y, 
                                  obj.bbox.x + obj.bbox.width,
                                  obj.bbox.y + obj.bbox.height};
                
                const char* colorNames[] = {"green", "yellow", "red", "blue", "null"};
                objJson["bbox_color"] = colorNames[static_cast<int>(obj.color)];
                objJson["has_bbox"] = obj.hasBbox;
                
                detJson["objects"].push_back(objJson);
            }
            
            responseJson["detections"].push_back(detJson);
        }
        
        response.statusCode = 200;
        response.body = responseJson.dump();
        
    } catch (const std::exception& e) {
        LOG_ERROR("Error handling get_detections: %s", e.what());
        
        json errorJson;
        errorJson["status"] = "error";
        errorJson["message"] = e.what();
        
        response.statusCode = 500;
        response.body = errorJson.dump();
    }
    
    return response;
}

ApiServer::Response ApiServer::handleGetLatest(const Request& request) {
    Response response;
    response.contentType = "application/json";
    
    try {
        // 요청 파싱
        json requestJson = json::parse(request.body);
        std::string camera = requestJson.value("camera", "");
        
        // 카메라 타입 결정
        CameraType camType;
        if (camera == "RGB_Camera") {
            camType = CameraType::RGB;
        } else if (camera == "Thermal_Camera") {
            camType = CameraType::THERMAL;
        } else {
            throw std::runtime_error("Invalid camera type");
        }
        
        // 버퍼 확인
        if (static_cast<size_t>(camType) >= detectionBuffers_.size() || 
            !detectionBuffers_[static_cast<size_t>(camType)]) {
            throw std::runtime_error("Detection buffer not available");
        }
        
        // 최신 검출 데이터 조회
        DetectionData latest;
        bool hasData = detectionBuffers_[static_cast<size_t>(camType)]->
                      getLatestDetection(latest);
        
        // JSON 응답 생성
        json responseJson;
        
        if (hasData) {
            responseJson["status"] = "success";
            responseJson["detection"]["timestamp"] = latest.timestamp;
            responseJson["detection"]["frame_number"] = latest.frameNumber;
            responseJson["detection"]["camera"] = camera;
            responseJson["detection"]["objects"] = json::array();
            
            for (const auto& obj : latest.objects) {
                json objJson;
                objJson["class_id"] = obj.classId;
                objJson["confidence"] = obj.confidence;
                objJson["bbox"] = {obj.bbox.x, obj.bbox.y,
                                  obj.bbox.x + obj.bbox.width,
                                  obj.bbox.y + obj.bbox.height};
                
                responseJson["detection"]["objects"].push_back(objJson);
            }
        } else {
            responseJson["status"] = "success";
            responseJson["detection"] = nullptr;
        }
        
        response.statusCode = 200;
        response.body = responseJson.dump();
        
    } catch (const std::exception& e) {
        LOG_ERROR("Error handling get_latest: %s", e.what());
        
        json errorJson;
        errorJson["status"] = "error";
        errorJson["message"] = e.what();
        
        response.statusCode = 500;
        response.body = errorJson.dump();
    }
    
    return response;
}

ApiServer::Response ApiServer::handleNotFound(const Request& request) {
    Response response;
    response.statusCode = 404;
    response.contentType = "application/json";
    
    json errorJson;
    errorJson["status"] = "error";
    errorJson["message"] = "Endpoint not found";
    errorJson["path"] = request.path;
    
    response.body = errorJson.dump();
    
    return response;
}

uint64_t ApiServer::parseISOTime(const std::string& isoTime) {
    // 간단한 ISO 8601 파싱 (형식: 2024-01-01T12:00:00Z)
    struct tm tm = {};
    
    // ISO 8601 형식 파싱
    if (sscanf(isoTime.c_str(), "%d-%d-%dT%d:%d:%d",
               &tm.tm_year, &tm.tm_mon, &tm.tm_mday,
               &tm.tm_hour, &tm.tm_min, &tm.tm_sec) == 6) {
        
        tm.tm_year -= 1900;  // tm_year는 1900년부터의 연도
        tm.tm_mon -= 1;      // tm_mon은 0-11
        
        // UTC 시간으로 변환
        time_t t = timegm(&tm);
        if (t != -1) {
            // 나노초로 변환
            return static_cast<uint64_t>(t) * 1000000000ULL;
        }
    }
    
    LOG_WARN("Failed to parse ISO time: %s", isoTime.c_str());
    return 0;
}