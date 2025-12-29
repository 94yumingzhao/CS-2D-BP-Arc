// logger.cpp - 日志系统实现（改进版）
//
// 改进内容:
// 1. 移除DualStreambuf，避免流重定向
// 2. 纯文件输出，与CPLEX完全隔离
// 3. 线程安全：使用mutex保护文件写入
// 4. 自动flush：确保崩溃时日志不丢失
// 5. 批量写入：性能优于逐字符处理
//
// 核心设计:
// - Logger::Write(): 线程安全的文件写入
// - Logger::WriteFormat(): 格式化输出（类似printf）
// - g_logger全局指针：方便宏访问
// - 不修改cout/cerr：避免与CPLEX冲突

#include "logger.h"

using namespace std;

// 全局Logger指针定义
Logger* g_logger = nullptr;

// Logger构造函数
// 功能: 创建日志文件，初始化日志系统
// 参数: log_prefix - 日志文件路径前缀（自动添加.log后缀）
Logger::Logger(const string& log_prefix) {
    log_file_path_ = log_prefix + ".log";

    // 确保日志文件所在目录存在
    filesystem::path log_path(log_file_path_);
    if (log_path.has_parent_path()) {
        filesystem::create_directories(log_path.parent_path());
    }

    // 打开日志文件（覆盖模式）
    log_file_.open(log_file_path_, ios::out | ios::trunc);

    if (!log_file_.is_open()) {
        fprintf(stderr, "[ERROR] 无法创建日志文件: %s\n", log_file_path_.c_str());
    }

    // 设置全局Logger指针
    g_logger = this;
}

// Logger析构函数
// 功能: 关闭日志文件，清空全局指针
Logger::~Logger() {
    try {
        // 关闭日志文件
        if (log_file_.is_open()) {
            log_file_.flush();
            log_file_.close();
        }

        // 清空全局指针
        if (g_logger == this) {
            g_logger = nullptr;
        }
    } catch (...) {
        // 忽略析构过程中的异常，避免程序崩溃
    }
}

// 获取当前时间戳
// 格式: [YYYY-MM-DD HH:MM:SS]
// 返回值: 格式化的时间戳字符串
string Logger::GetTimestamp() {
    auto now = chrono::system_clock::now();
    auto time_t_val = chrono::system_clock::to_time_t(now);

    tm tm_buf;
#ifdef _WIN32
    localtime_s(&tm_buf, &time_t_val);
#else
    localtime_r(&time_t_val, &tm_buf);
#endif

    stringstream ss;
    ss << "[" << put_time(&tm_buf, "%Y-%m-%d %H:%M:%S") << "] ";
    return ss.str();
}

// 写入日志消息（带时间戳）
// 功能: 线程安全地写入日志文件
// 参数: msg - 日志消息
// 说明:
//   - 自动添加时间戳
//   - 立即flush，确保崩溃时不丢日志
//   - 线程安全
void Logger::Write(const string& msg) {
    if (!log_file_.is_open()) {
        return;
    }

    // 线程安全：加锁保护文件写入
    lock_guard<mutex> lock(mutex_);

    try {
        // 写入时间戳 + 消息
        string timestamp = GetTimestamp();
        log_file_ << timestamp << msg;

        // 立即flush，确保日志写入磁盘
        log_file_.flush();
    } catch (const exception& e) {
        // 捕获异常，避免日志错误导致程序崩溃
        fprintf(stderr, "[ERROR] 日志写入失败: %s\n", e.what());
    }
}

// 格式化写入日志
// 功能: 类似printf的格式化日志输出
// 参数: fmt - 格式化字符串, ... - 参数列表
// 说明:
//   - 支持printf风格的格式化
//   - 自动添加时间戳
//   - 线程安全
void Logger::WriteFormat(const char* fmt, ...) {
    if (!log_file_.is_open()) {
        return;
    }

    // 格式化字符串
    char buffer[4096];  // 缓冲区大小：4KB
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    // 调用Write写入（自动加时间戳和flush）
    Write(buffer);
}
