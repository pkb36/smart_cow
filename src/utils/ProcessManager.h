#ifndef PROCESS_MANAGER_H
#define PROCESS_MANAGER_H

#include <string>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <sys/types.h>

class ProcessManager {
public:
    struct ProcessInfo {
        pid_t pid;
        std::string name;
        std::string command;
        bool isRunning;
    };
    
    static ProcessManager& getInstance();
    
    // 프로세스 시작/종료
    pid_t startProcess(const std::string& name, const std::string& command);
    bool stopProcess(const std::string& name);
    bool stopProcess(pid_t pid);
    void stopAllProcesses();
    
    // 프로세스 상태 확인
    bool isProcessRunning(const std::string& name);
    bool isProcessRunning(pid_t pid);
    pid_t getProcessPid(const std::string& name);
    
    // 프로세스 관리
    void checkProcesses();  // 좀비 프로세스 정리
    std::vector<ProcessInfo> getProcessList() const;
    
    // 녹화 관련 헬퍼 함수
    bool startRecording(int deviceCount, int streamPort, const std::string& codecName, 
                       const std::string& location, int duration);
    bool stopRecording();
    bool isRecordingActive();
    
private:
    ProcessManager();
    ~ProcessManager();
    ProcessManager(const ProcessManager&) = delete;
    ProcessManager& operator=(const ProcessManager&) = delete;
    
    static void signalHandler(int sig);
    void cleanupZombies();
    
private:
    mutable std::mutex mutex_;
    std::unordered_map<std::string, ProcessInfo> processes_;
    std::unordered_map<pid_t, std::string> pidToName_;
};

#endif // PROCESS_MANAGER_H