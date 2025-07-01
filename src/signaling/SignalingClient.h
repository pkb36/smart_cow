#ifndef SIGNALING_CLIENT_H
#define SIGNALING_CLIENT_H

#include <memory>
#include <thread>
#include <atomic>
#include <functional>
#include <string>
#include <queue>
#include <mutex>
#include <glib.h>
#include <gio/gio.h>

// Forward declarations
typedef struct _SoupWebsocketConnection SoupWebsocketConnection;
typedef struct _SoupSession SoupSession;

class SignalingClient {
public:
    enum class ConnectionState {
        DISCONNECTED,
        CONNECTING,
        CONNECTED,
        RECONNECTING
    };
    
    struct Message {
        std::string type;
        std::string peerId;
        std::string data;
    };
    
    using MessageCallback = std::function<void(const Message&)>;
    using StateCallback = std::function<void(ConnectionState)>;
    
    SignalingClient(const std::string& serverUrl, const std::string& cameraId);
    ~SignalingClient();
    
    bool connect();
    void disconnect();
    bool isConnected() const;
    bool isWebSocketOpen() const;
    
    // 메시지 전송
    bool sendMessage(const std::string& type, const std::string& data);
    bool sendToPeer(const std::string& peerId, const std::string& type, const std::string& data);
    
    // 콜백 등록
    void setMessageCallback(MessageCallback callback);
    void setStateCallback(StateCallback callback);
    
    // 카메라 등록
    bool registerCamera();
    void sendSdpOffer(const std::string& peerId, const std::string& sdp);
    void sendIceCandidate(const std::string& peerId, int mlineindex, const std::string& candidate);
    void sendCameraStatus();
    // 재연결
    void enableAutoReconnect(bool enable);
    void setReconnectInterval(int seconds);

    // 상태 전송 관련
    void startStatusReporting(int intervalSeconds = 30);
    void stopStatusReporting();
    void updateCameraStatus(const std::string& recStatus, 
                           double cpuTemp, 
                           double gpuTemp, 
                           int recUsage);

    // 카메라 상태 정보
    static double getCpuTemperature();
    static double getGpuTemperature();
    static int getDiskUsage();
    
private:
    static void onConnected(SoupSession* session, GAsyncResult* result, gpointer userData);
    static void onMessage(SoupWebsocketConnection* conn, gint type, GBytes* message, gpointer userData);
    static void onClosed(SoupWebsocketConnection* conn, gpointer userData);
    static void onError(SoupWebsocketConnection* conn, GError* error, gpointer userData);
    
    void handleMessage(const std::string& message);
    void reconnect();
    void reconnectThread();
    
private:
    std::string serverUrl_;
    std::string cameraId_;
    
    SoupSession* session_;
    SoupWebsocketConnection* connection_;
    
    std::atomic<ConnectionState> state_;
    std::atomic<bool> autoReconnect_;
    int reconnectInterval_;
    
    MessageCallback messageCallback_;
    StateCallback stateCallback_;
    
    std::thread reconnectThread_;
    std::mutex connectionMutex_;

    // 상태 정보
    struct CameraStatus {
        std::string recStatus = "Off";
        double cpuTemperature = 0.0;
        double gpuTemperature = 0.0;
        int recUsage = 0;
    };
    
    CameraStatus cameraStatus_;
    guint statusTimerId_;
    int statusInterval_;
    std::mutex statusMutex_;
    
    static gboolean statusTimerCallback(gpointer userData);
};

#endif // SIGNALING_CLIENT_H