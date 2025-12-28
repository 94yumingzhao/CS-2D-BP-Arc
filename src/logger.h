// =============================================================================
// logger.h - 日志系统头文件
// =============================================================================
// 功能: 提供双输出日志功能 (终端 + 文件同步输出)
//
// 使用方式:
//   int main() {
//       Logger logger("output/run_log");  // 自动重定向 cout
//       cout << "[启动] 程序开始运行\n";  // 同时输出到控制台和文件
//   }  // 析构时自动恢复 cout
// =============================================================================

#ifndef LOGGER_H_
#define LOGGER_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <memory>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <filesystem>


// 双输出流缓冲区
class DualStreambuf : public std::streambuf {
public:
    // 构造函数
    DualStreambuf(std::streambuf* console_buf, std::streambuf* file_buf);

    // 禁用复制和移动
    DualStreambuf(const DualStreambuf&) = delete;
    DualStreambuf& operator=(const DualStreambuf&) = delete;
    DualStreambuf(DualStreambuf&&) = delete;
    DualStreambuf& operator=(DualStreambuf&&) = delete;

protected:
    // 单字符输出处理
    int overflow(int c) override;

    // 批量字符输出处理
    std::streamsize xsputn(const char* s, std::streamsize count) override;

private:
    std::streambuf* console_buf_;   // 控制台缓冲区指针
    std::streambuf* file_buf_;      // 文件缓冲区指针
    bool need_timestamp_;           // 标记: 下一个字符是否需要时间戳

    // 获取当前时间戳 "[YYYY-MM-DD HH:MM:SS.mmm] "
    std::string GetCurrentTimestamp();

    // 向两个缓冲区写入时间戳
    void WriteTimestamp();
};


// 日志管理器
class Logger {
public:
    // 构造函数 - 初始化日志系统
    explicit Logger(const std::string& log_prefix);

    // 析构函数 - 恢复标准输出并关闭日志文件
    ~Logger();

    // 禁用复制和移动
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    Logger(Logger&&) = delete;
    Logger& operator=(Logger&&) = delete;

    // 获取日志文件路径
    std::string GetLogFilePath() const { return log_file_path_; }

private:
    std::ofstream log_file_;                    // 日志文件输出流
    std::streambuf* old_cout_buf_;              // 原始 cout 缓冲区
    std::unique_ptr<DualStreambuf> dual_buf_;   // 双输出缓冲区
    std::string log_file_path_;                 // 日志文件完整路径
};


// 获取时间戳字符串 (用于文件名)
// 返回格式: YYYYMMDD_HHMMSS_mmm
inline std::string GetTimestampString() {
    auto now = std::chrono::system_clock::now();
    auto time_t_val = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::tm tm_buf;
#ifdef _WIN32
    localtime_s(&tm_buf, &time_t_val);
#else
    localtime_r(&time_t_val, &tm_buf);
#endif

    std::stringstream ss;
    ss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S")
       << "_" << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}


// 日志输出宏

// 带换行的日志输出
#define LOG(msg) std::cout << msg << std::endl

// 不带换行的日志输出
#define LOG_NO_NL(msg) std::cout << msg

// 格式化日志输出
#define LOG_FMT(fmt, ...) do { \
    char _log_buf[1024]; \
    snprintf(_log_buf, sizeof(_log_buf), fmt, ##__VA_ARGS__); \
    std::cout << _log_buf; \
} while(0)

#endif  // LOGGER_H_
