// logger.h - 日志系统头文件（改进版）
//
// 功能: 提供安全的文件日志功能
//
// 改进点:
// 1. 纯文件输出，避免流重定向导致的崩溃
// 2. 不修改全局cout状态，与CPLEX完全隔离
// 3. 线程安全的文件写入
// 4. 批量写入提升性能
// 5. 自动flush确保崩溃时不丢日志
//
// 使用方式:
//   int main() {
//       Logger logger("logs/run_log");  // 创建日志文件
//       LOG("程序开始运行");              // 写入日志文件
//       LOG_FMT("处理 %d 个任务\n", 100); // 格式化日志
//   }  // 析构时自动关闭日志文件

#ifndef LOGGER_H_
#define LOGGER_H_

#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <mutex>
#include <filesystem>
#include <cstdio>
#include <cstdarg>

// 简化的文件日志类
// 特点:
// - 只写文件，不重定向流
// - 线程安全
// - 每次写入自动flush
class Logger {
public:
    // 构造函数 - 创建日志文件
    // 参数: log_prefix - 日志文件路径前缀（自动添加.log后缀）
    explicit Logger(const std::string& log_prefix);

    // 析构函数 - 关闭日志文件
    ~Logger();

    // 禁用复制和移动
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    Logger(Logger&&) = delete;
    Logger& operator=(Logger&&) = delete;

    // 写入日志消息（带时间戳）
    // 参数: msg - 日志消息
    void Write(const std::string& msg);

    // 格式化写入日志
    // 参数: fmt - 格式化字符串, ... - 参数列表
    void WriteFormat(const char* fmt, ...);

    // 获取日志文件路径
    std::string GetLogFilePath() const { return log_file_path_; }

private:
    // 获取当前时间戳 "[YYYY-MM-DD HH:MM:SS] "
    std::string GetTimestamp();

    std::ofstream log_file_;    // 日志文件流
    std::mutex mutex_;          // 线程安全互斥锁
    std::string log_file_path_; // 日志文件完整路径
};

// 全局Logger指针
extern Logger* g_logger;

// 获取时间戳字符串（用于文件名）
// 返回格式: YYYYMMDD_HHMMSS
inline std::string GetTimestampString() {
    auto now = std::chrono::system_clock::now();
    auto time_t_val = std::chrono::system_clock::to_time_t(now);

    std::tm tm_buf;
#ifdef _WIN32
    localtime_s(&tm_buf, &time_t_val);
#else
    localtime_r(&time_t_val, &tm_buf);
#endif

    std::stringstream ss;
    ss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
    return ss.str();
}

// 日志输出宏 - 只写文件

// 带换行的日志输出（只写文件）
#define LOG(msg) do { \
    if (g_logger) { \
        g_logger->Write(std::string(msg) + "\n"); \
    } \
} while(0)

// 不带换行的日志输出（只写文件）
#define LOG_NO_NL(msg) do { \
    if (g_logger) { \
        g_logger->Write(msg); \
    } \
} while(0)

// 格式化日志输出（只写文件）
#define LOG_FMT(fmt, ...) do { \
    if (g_logger) { \
        g_logger->WriteFormat(fmt, ##__VA_ARGS__); \
    } \
} while(0)

// 控制台输出宏（独立，不经过Logger）
// 用于关键进度信息的实时显示

// 控制台输出（带换行）
#define CONSOLE(msg) fprintf(stderr, "%s\n", msg)

// 控制台格式化输出
#define CONSOLE_FMT(fmt, ...) fprintf(stderr, fmt, ##__VA_ARGS__)

// 格式化已用时间为 [MM:SS.s] 格式
// 参数: elapsed_sec - 已用秒数 (double)
// 返回: "[01:23.4]" 格式的字符串
inline std::string FormatElapsed(double elapsed_sec) {
    int minutes = static_cast<int>(elapsed_sec) / 60;
    double seconds = elapsed_sec - minutes * 60;
    char buf[16];
    snprintf(buf, sizeof(buf), "[%02d:%04.1f]", minutes, seconds);
    return std::string(buf);
}

// 带时间戳的控制台输出宏
// 参数: elapsed - 已用时间(秒), fmt - 格式字符串, ... - 参数
#define PROGRESS(elapsed, fmt, ...) \
    fprintf(stderr, "%s " fmt, FormatElapsed(elapsed).c_str(), ##__VA_ARGS__)

#endif  // LOGGER_H_
