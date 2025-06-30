#ifndef API_SERVER_H
#define API_SERVER_H

#include <memory>
#include <thread>
#include <atomic>
#include <functional>
#include <string>
#include <vector>
#include "../common/Types.h"

class DetectionBuffer;

class ApiServer {
public:
    struct Request {
        std::string method;
        std::string path;
        std::string body;
    };
    
    struct Response {
        int statusCode;
        std::string contentType;
        std::string body;
    };
    
    using RequestHandler = std::function<Response(const Request&)>;
    
    ApiServer(int port);
    ~ApiServer();
    
    bool start();
    void stop();
    bool isRunning() const;
    
    // 검출 버퍼 등록
    void registerDetectionBuffer(CameraType type, DetectionBuffer* buffer);
    
    // 라우트 등록
    void addRoute(const std::string& method, const std::string& path, RequestHandler handler);
    
private:
    void serverThread();
    void handleClient(int clientSocket);
    Request parseRequest(const std::string& rawRequest);
    std::string buildResponse(const Response& response);
    
    // 기본 핸들러들
    Response handleGetDetections(const Request& request);
    Response handleGetLatest(const Request& request);
    Response handleNotFound(const Request& request);
    
    // 헬퍼 함수
    uint64_t parseISOTime(const std::string& isoTime);
    
private:
    int port_;
    int serverSocket_;
    std::atomic<bool> running_;
    std::thread serverThread_;
    
    // 검출 버퍼들
    std::vector<DetectionBuffer*> detectionBuffers_;
    
    // 라우트 맵
    std::unordered_map<std::string, RequestHandler> routes_;
};

#endif // API_SERVER_H