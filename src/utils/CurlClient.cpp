#include "CurlClient.h"
#include "Logger.h"
#include <curl/curl.h>
#include <sstream>
#include <fstream>

class CurlClient::Impl {
public:
    Impl() : timeout_(30), sslVerify_(true) {
        curl_global_init(CURL_GLOBAL_ALL);
    }
    
    ~Impl() {
        curl_global_cleanup();
    }
    
    Response performRequest(const std::string& method, const std::string& url, 
                           const std::string& data = "") {
        Response response;
        CURL* curl = curl_easy_init();
        
        if (!curl) {
            response.statusCode = -1;
            response.error = "Failed to initialize CURL";
            return response;
        }
        
        // 응답 데이터를 저장할 버퍼
        std::string responseBuffer;
        std::string headerBuffer;
        
        // 기본 옵션 설정
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout_);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, sslVerify_ ? 1L : 0L);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, sslVerify_ ? 2L : 0L);
        
        // 응답 콜백 설정
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &responseBuffer);
        curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, headerCallback);
        curl_easy_setopt(curl, CURLOPT_HEADERDATA, &headerBuffer);
        
        // 헤더 설정
        struct curl_slist* headers = nullptr;
        for (const auto& header : headers_) {
            std::string headerStr = header.first + ": " + header.second;
            headers = curl_slist_append(headers, headerStr.c_str());
        }
        
        // HTTP 메소드별 설정
        if (method == "POST") {
            curl_easy_setopt(curl, CURLOPT_POST, 1L);
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data.c_str());
            curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, data.length());
        } else if (method == "PUT") {
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data.c_str());
            curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, data.length());
        } else if (method == "DELETE") {
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "DELETE");
        }
        
        if (headers) {
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        }
        
        // SSL 인증서 설정
        if (!sslCert_.empty()) {
            curl_easy_setopt(curl, CURLOPT_SSLCERT, sslCert_.c_str());
        }
        if (!sslKey_.empty()) {
            curl_easy_setopt(curl, CURLOPT_SSLKEY, sslKey_.c_str());
        }
        
        // 요청 실행
        CURLcode res = curl_easy_perform(curl);
        
        if (res != CURLE_OK) {
            response.statusCode = -1;
            response.error = curl_easy_strerror(res);
            LOG_ERROR("CURL request failed: %s", response.error.c_str());
        } else {
            // HTTP 상태 코드 가져오기
            long httpCode = 0;
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);
            response.statusCode = static_cast<int>(httpCode);
            response.body = responseBuffer;
            
            // 헤더 파싱
            parseHeaders(headerBuffer, response.headers);
            
            LOG_DEBUG("%s %s - Status: %d", method.c_str(), url.c_str(), response.statusCode);
        }
        
        // 정리
        if (headers) {
            curl_slist_free_all(headers);
        }
        curl_easy_cleanup(curl);
        
        return response;
    }
    
    bool downloadFileInternal(const std::string& url, const std::string& outputPath,
                             ProgressCallback progress) {
        CURL* curl = curl_easy_init();
        if (!curl) {
            LOG_ERROR("Failed to initialize CURL");
            return false;
        }
        
        std::ofstream file(outputPath, std::ios::binary);
        if (!file.is_open()) {
            LOG_ERROR("Failed to open output file: %s", outputPath.c_str());
            curl_easy_cleanup(curl);
            return false;
        }
        
        // 진행률 콜백 데이터
        ProgressData progressData = {progress, 0, 0};
        
        // 옵션 설정
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout_);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, sslVerify_ ? 1L : 0L);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, fileWriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &file);
        
        if (progress) {
            curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0L);
            curl_easy_setopt(curl, CURLOPT_PROGRESSFUNCTION, progressCallback);
            curl_easy_setopt(curl, CURLOPT_PROGRESSDATA, &progressData);
        }
        
        // 헤더 설정
        struct curl_slist* headers = nullptr;
        for (const auto& header : headers_) {
            std::string headerStr = header.first + ": " + header.second;
            headers = curl_slist_append(headers, headerStr.c_str());
        }
        if (headers) {
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        }
        
        // 다운로드 실행
        CURLcode res = curl_easy_perform(curl);
        
        bool success = (res == CURLE_OK);
        if (!success) {
            LOG_ERROR("Download failed: %s", curl_easy_strerror(res));
        }
        
        // 정리
        file.close();
        if (headers) {
            curl_slist_free_all(headers);
        }
        curl_easy_cleanup(curl);
        
        // 실패 시 파일 삭제
        if (!success) {
            std::remove(outputPath.c_str());
        }
        
        return success;
    }
    
private:
    static size_t writeCallback(void* contents, size_t size, size_t nmemb, void* userp) {
        size_t totalSize = size * nmemb;
        std::string* str = static_cast<std::string*>(userp);
        str->append(static_cast<char*>(contents), totalSize);
        return totalSize;
    }
    
    static size_t headerCallback(void* contents, size_t size, size_t nmemb, void* userp) {
        size_t totalSize = size * nmemb;
        std::string* str = static_cast<std::string*>(userp);
        str->append(static_cast<char*>(contents), totalSize);
        return totalSize;
    }
    
    static size_t fileWriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
        size_t totalSize = size * nmemb;
        std::ofstream* file = static_cast<std::ofstream*>(userp);
        file->write(static_cast<char*>(contents), totalSize);
        return totalSize;
    }
    
    struct ProgressData {
        ProgressCallback callback;
        double lastTotal;
        double lastCurrent;
    };
    
    static int progressCallback(void* clientp, double dltotal, double dlnow, 
                               double ultotal, double ulnow) {
        ProgressData* data = static_cast<ProgressData*>(clientp);
        if (data->callback && (dltotal > 0)) {
            // 콜백이 false를 반환하면 중단
            if (!data->callback(dltotal, dlnow)) {
                return 1;  // CURL 중단
            }
        }
        return 0;
    }
    
    void parseHeaders(const std::string& headerStr, std::map<std::string, std::string>& headers) {
        std::istringstream stream(headerStr);
        std::string line;
        
        while (std::getline(stream, line)) {
            size_t pos = line.find(':');
            if (pos != std::string::npos) {
                std::string key = line.substr(0, pos);
                std::string value = line.substr(pos + 1);
                
                // 공백 제거
                key.erase(key.find_last_not_of(" \r\n") + 1);
                value.erase(0, value.find_first_not_of(" "));
                value.erase(value.find_last_not_of(" \r\n") + 1);
                
                headers[key] = value;
            }
        }
    }
    
public:
    int timeout_;
    bool sslVerify_;
    std::string sslCert_;
    std::string sslKey_;
    std::map<std::string, std::string> headers_;
};

// CurlClient 구현
CurlClient::CurlClient() : pImpl(std::make_unique<Impl>()) {}
CurlClient::~CurlClient() = default;

CurlClient::Response CurlClient::get(const std::string& url) {
    return pImpl->performRequest("GET", url);
}

CurlClient::Response CurlClient::post(const std::string& url, const std::string& data, 
                                     const std::string& contentType) {
    pImpl->headers_["Content-Type"] = contentType;
    return pImpl->performRequest("POST", url, data);
}

CurlClient::Response CurlClient::put(const std::string& url, const std::string& data,
                                    const std::string& contentType) {
    pImpl->headers_["Content-Type"] = contentType;
    return pImpl->performRequest("PUT", url, data);
}

CurlClient::Response CurlClient::del(const std::string& url) {
    return pImpl->performRequest("DELETE", url);
}

bool CurlClient::downloadFile(const std::string& url, const std::string& outputPath,
                             ProgressCallback progress) {
    return pImpl->downloadFileInternal(url, outputPath, progress);
}

void CurlClient::setTimeout(int seconds) {
    pImpl->timeout_ = seconds;
}

void CurlClient::setHeader(const std::string& key, const std::string& value) {
    pImpl->headers_[key] = value;
}

void CurlClient::clearHeaders() {
    pImpl->headers_.clear();
}

void CurlClient::setBasicAuth(const std::string& username, const std::string& password) {
    std::string auth = username + ":" + password;
    setHeader("Authorization", "Basic " + auth);  // Base64 인코딩 필요
}

void CurlClient::setBearerToken(const std::string& token) {
    setHeader("Authorization", "Bearer " + token);
}

void CurlClient::setSSLVerify(bool verify) {
    pImpl->sslVerify_ = verify;
}

void CurlClient::setSSLCert(const std::string& certPath) {
    pImpl->sslCert_ = certPath;
}

void CurlClient::setSSLKey(const std::string& keyPath) {
    pImpl->sslKey_ = keyPath;
}