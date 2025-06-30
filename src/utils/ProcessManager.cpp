#include "ProcessManager.h"
#include "Logger.h"
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#include <cstring>
#include <sstream>

ProcessManager::ProcessManager() {
    // SIGCHLD 시그널 핸들러 설정
    struct sigaction sa;
    sa.sa_handler = &ProcessManager::signalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    
    if (sigaction(SIGCHLD, &sa, nullptr) == -1) {
        LOG_ERROR("Failed to set SIGCHLD handler");
    }
}

ProcessManager::~ProcessManager() {
    stopAllProcesses();
}

ProcessManager& ProcessManager::getInstance() {
    static ProcessManager instance;
    return instance;
}

pid_t ProcessManager::startProcess(const std::string& name, const std::string& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 이미 실행 중인지 확인
    auto it = processes_.find(name);
    if (it != processes_.end() && it->second.isRunning) {
        LOG_WARN("Process %s is already running (pid: %d)", name.c_str(), it->second.pid);
        return it->second.pid;
    }
    
    // fork로 새 프로세스 생성
    pid_t pid = fork();
    
    if (pid < 0) {
        LOG_ERROR("Failed to fork process: %s", strerror(errno));
        return -1;
    } else if (pid == 0) {
        // 자식 프로세스
        
        // 새로운 프로세스 그룹 설정 (부모가 죽어도 계속 실행)
        setsid();
        
        // 명령어를 토큰으로 분리
        std::vector<std::string> args;
        std::istringstream iss(command);
        std::string token;
        while (iss >> token) {
            args.push_back(token);
        }
        
        if (args.empty()) {
            _exit(1);
        }
        
        // char* 배열로 변환
        std::vector<char*> argv;
        for (auto& arg : args) {
            argv.push_back(const_cast<char*>(arg.c_str()));
        }
        argv.push_back(nullptr);
        
        // 프로세스 실행
        execvp(argv[0], argv.data());
        
        // execvp가 실패한 경우만 여기 도달
        LOG_ERROR("Failed to execute %s: %s", argv[0], strerror(errno));
        _exit(1);
    }
    
    // 부모 프로세스
    ProcessInfo info;
    info.pid = pid;
    info.name = name;
    info.command = command;
    info.isRunning = true;
    
    processes_[name] = info;
    pidToName_[pid] = name;
    
    LOG_INFO("Started process %s (pid: %d): %s", name.c_str(), pid, command.c_str());
    return pid;
}

bool ProcessManager::stopProcess(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = processes_.find(name);
    if (it == processes_.end()) {
        LOG_WARN("Process %s not found", name.c_str());
        return false;
    }
    
    if (!it->second.isRunning) {
        LOG_WARN("Process %s is not running", name.c_str());
        return false;
    }
    
    pid_t pid = it->second.pid;
    
    // SIGTERM 전송
    if (kill(pid, SIGTERM) == -1) {
        if (errno == ESRCH) {
            // 프로세스가 이미 종료됨
            it->second.isRunning = false;
            pidToName_.erase(pid);
            LOG_INFO("Process %s (pid: %d) already terminated", name.c_str(), pid);
            return true;
        }
        LOG_ERROR("Failed to send SIGTERM to process %s (pid: %d): %s", 
                  name.c_str(), pid, strerror(errno));
        return false;
    }
    
    // 잠시 대기 후 강제 종료
    usleep(100000);  // 100ms
    
    // 프로세스가 아직 살아있는지 확인
    if (kill(pid, 0) == 0) {
        // SIGKILL 전송
        if (kill(pid, SIGKILL) == -1 && errno != ESRCH) {
            LOG_ERROR("Failed to send SIGKILL to process %s (pid: %d): %s",
                      name.c_str(), pid, strerror(errno));
            return false;
        }
    }
    
    it->second.isRunning = false;
    pidToName_.erase(pid);
    
    LOG_INFO("Stopped process %s (pid: %d)", name.c_str(), pid);
    return true;
}

bool ProcessManager::stopProcess(pid_t pid) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = pidToName_.find(pid);
    if (it != pidToName_.end()) {
        return stopProcess(it->second);
    }
    
    // 직접 종료 시도
    if (kill(pid, SIGTERM) == -1) {
        if (errno != ESRCH) {
            LOG_ERROR("Failed to stop process (pid: %d): %s", pid, strerror(errno));
            return false;
        }
    }
    
    return true;
}

void ProcessManager::stopAllProcesses() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    for (auto& pair : processes_) {
        if (pair.second.isRunning) {
            kill(pair.second.pid, SIGTERM);
        }
    }
    
    // 잠시 대기
    usleep(200000);  // 200ms
    
    // 강제 종료
    for (auto& pair : processes_) {
        if (pair.second.isRunning && kill(pair.second.pid, 0) == 0) {
            kill(pair.second.pid, SIGKILL);
            pair.second.isRunning = false;
        }
    }
    
    pidToName_.clear();
    LOG_INFO("All processes stopped");
}

bool ProcessManager::isProcessRunning(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = processes_.find(name);
    if (it == processes_.end()) {
        return false;
    }
    
    // 실제로 살아있는지 확인
    if (it->second.isRunning && kill(it->second.pid, 0) == -1) {
        it->second.isRunning = false;
        pidToName_.erase(it->second.pid);
    }
    
    return it->second.isRunning;
}

bool ProcessManager::isProcessRunning(pid_t pid) {
    return kill(pid, 0) == 0;
}

pid_t ProcessManager::getProcessPid(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = processes_.find(name);
    if (it != processes_.end() && it->second.isRunning) {
        return it->second.pid;
    }
    
    return -1;
}

void ProcessManager::checkProcesses() {
    cleanupZombies();
}

std::vector<ProcessManager::ProcessInfo> ProcessManager::getProcessList() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<ProcessInfo> list;
    for (const auto& pair : processes_) {
        list.push_back(pair.second);
    }
    
    return list;
}

bool ProcessManager::startRecording(int deviceCount, int streamPort, 
                                   const std::string& codecName, 
                                   const std::string& location, int duration) {
    std::ostringstream cmd;
    cmd << "./webrtc_recorder"
        << " --stream_cnt=" << deviceCount
        << " --stream_base_port=" << streamPort
        << " --codec_name=" << codecName
        << " --location=" << location
        << " --duration=" << duration;
    
    pid_t pid = startProcess("recorder", cmd.str());
    return pid > 0;
}

bool ProcessManager::stopRecording() {
    return stopProcess("recorder");
}

bool ProcessManager::isRecordingActive() {
    return isProcessRunning("recorder");
}

void ProcessManager::signalHandler(int sig) {
    if (sig == SIGCHLD) {
        // 자식 프로세스 종료 처리
        getInstance().cleanupZombies();
    }
}

void ProcessManager::cleanupZombies() {
    pid_t pid;
    int status;
    
    // 모든 종료된 자식 프로세스 처리
    while ((pid = waitpid(-1, &status, WNOHANG)) > 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto it = pidToName_.find(pid);
        if (it != pidToName_.end()) {
            std::string name = it->second;
            
            if (WIFEXITED(status)) {
                LOG_INFO("Process %s (pid: %d) exited with code %d",
                         name.c_str(), pid, WEXITSTATUS(status));
            } else if (WIFSIGNALED(status)) {
                LOG_INFO("Process %s (pid: %d) terminated by signal %d",
                         name.c_str(), pid, WTERMSIG(status));
            }
            
            processes_[name].isRunning = false;
            pidToName_.erase(it);
        }
    }
}