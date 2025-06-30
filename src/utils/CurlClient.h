#ifndef CURL_CLIENT_H
#define CURL_CLIENT_H

#include <string>
#include <memory>
#include <functional>
#include <map>

class CurlClient {
public:
    struct Response {
        int statusCode;
        std::string body;
        std::map<std::string, std::string> headers;
        std::string error;
    };
    
    using ProgressCallback = std::function<bool(double total, double current)>;
    
    CurlClient();
    ~CurlClient();
    
    // HTTP 메소드
    Response get(const std::string& url);
    Response post(const std::string& url, const std::string& data, 
                 const std::string& contentType = "application/json");
    Response put(const std::string& url, const std::string& data,
                const std::string& contentType = "application/json");
    Response del(const std::string& url);
    
    // 파일 업로드/다운로드
    Response uploadFile(const std::string& url, const std::string& filePath,
                       const std::string& fieldName = "file");
    bool downloadFile(const std::string& url, const std::string& outputPath,
                     ProgressCallback progress = nullptr);
    
    // 설정
    void setTimeout(int seconds);
    void setHeader(const std::string& key, const std::string& value);
    void clearHeaders();
    void setBasicAuth(const std::string& username, const std::string& password);
    void setBearerToken(const std::string& token);
    
    // SSL 설정
    void setSSLVerify(bool verify);
    void setSSLCert(const std::string& certPath);
    void setSSLKey(const std::string& keyPath);
    
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // CURL_CLIENT_H