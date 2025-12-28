// logger.cpp - 日志系统实现
//
// 本文件实现双输出流日志系统, 支持:
// - 同时输出到控制台和日志文件
// - 每行自动添加时间戳 (精确到毫秒)
// - 日志文件自动创建和目录管理
//
// 核心类:
// - DualStreambuf: 双输出流缓冲区, 实现同时写入控制台和文件
// - Logger: 日志管理器, 封装初始化和清理逻辑
//
// 使用方式:
// 1. 在main()开头创建Logger对象
// 2. 使用LOG/LOG_FMT宏输出日志
// 3. Logger析构时自动恢复标准输出并关闭日志文件

#include "logger.h"

using namespace std;

// DualStreambuf构造函数
// 功能: 初始化双输出流缓冲区
// 参数:
//   - console_buf: 控制台输出缓冲区 (通常为cout.rdbuf())
//   - file_buf: 文件输出缓冲区 (日志文件的rdbuf)
DualStreambuf::DualStreambuf(streambuf* console_buf, streambuf* file_buf)
    : console_buf_(console_buf)
    , file_buf_(file_buf)
    , need_timestamp_(true) {  // 每行开头需要时间戳
}

// 获取当前时间戳字符串
// 格式: [YYYY-MM-DD HH:MM:SS.mmm]
// 返回值: 格式化的时间戳字符串
string DualStreambuf::GetCurrentTimestamp() {
    auto now = chrono::system_clock::now();
    auto time_t_val = chrono::system_clock::to_time_t(now);
    auto ms = chrono::duration_cast<chrono::milliseconds>(
        now.time_since_epoch()) % 1000;  // 毫秒部分

    tm tm_buf;
#ifdef _WIN32
    localtime_s(&tm_buf, &time_t_val);  // Windows线程安全版本
#else
    localtime_r(&time_t_val, &tm_buf);  // POSIX线程安全版本
#endif

    stringstream ss;
    ss << "[" << put_time(&tm_buf, "%Y-%m-%d %H:%M:%S")
       << "." << setfill('0') << setw(3) << ms.count() << "] ";
    return ss.str();
}

// 写入时间戳到两个缓冲区
// 功能: 在每行开头输出时间戳
void DualStreambuf::WriteTimestamp() {
    string timestamp = GetCurrentTimestamp();

    // 写入控制台
    if (console_buf_) {
        console_buf_->sputn(timestamp.c_str(), timestamp.length());
    }

    // 写入日志文件
    if (file_buf_) {
        file_buf_->sputn(timestamp.c_str(), timestamp.length());
    }
}

// 单字符输出处理 (streambuf虚函数)
// 功能: 处理每个输出字符, 在行首添加时间戳
// 参数: c - 输出字符
// 返回值: 输出的字符, EOF表示错误
int DualStreambuf::overflow(int c) {
    if (c != EOF) {
        // 如果需要时间戳 (行首), 先输出时间戳
        if (need_timestamp_) {
            WriteTimestamp();
            need_timestamp_ = false;
        }

        // 输出字符到两个缓冲区
        if (console_buf_) {
            console_buf_->sputc(c);
        }
        if (file_buf_) {
            file_buf_->sputc(c);
        }

        // 换行符后, 下一个字符需要时间戳
        if (c == '\n') {
            need_timestamp_ = true;
        }
    }
    return c;
}

// 批量字符输出处理 (streambuf虚函数)
// 功能: 批量输出字符串, 逐字符处理以保证时间戳正确
// 参数: s - 字符串指针, count - 字符数
// 返回值: 实际输出的字符数
streamsize DualStreambuf::xsputn(const char* s, streamsize count) {
    for (streamsize i = 0; i < count; ++i) {
        overflow(s[i]);  // 逐字符处理
    }
    return count;
}

// Logger构造函数 - 初始化日志系统
// 功能: 创建日志文件, 替换cout缓冲区为双输出缓冲区
// 参数: log_prefix - 日志文件路径前缀 (自动添加.log后缀)
Logger::Logger(const string& log_prefix)
    : old_cout_buf_(nullptr)
    , dual_buf_(nullptr) {

    log_file_path_ = log_prefix + ".log";

    // 确保日志文件所在目录存在
    filesystem::path log_path(log_file_path_);
    if (log_path.has_parent_path()) {
        filesystem::create_directories(log_path.parent_path());
    }

    // 打开日志文件 (覆盖模式)
    log_file_.open(log_file_path_, ios::out | ios::trunc);

    if (log_file_.is_open()) {
        // 保存原始cout缓冲区, 用于析构时恢复
        old_cout_buf_ = cout.rdbuf();
        // 创建双输出缓冲区
        dual_buf_ = make_unique<DualStreambuf>(old_cout_buf_, log_file_.rdbuf());
        // 替换cout缓冲区
        cout.rdbuf(dual_buf_.get());
    }
}

// Logger析构函数 - 恢复标准输出并关闭日志文件
// 功能: 恢复cout原始缓冲区, 关闭日志文件
Logger::~Logger() {
    try {
        // 恢复原始cout缓冲区
        if (old_cout_buf_) {
            cout.rdbuf(old_cout_buf_);
        }

        // 关闭日志文件
        if (log_file_.is_open()) {
            log_file_.close();
        }
    } catch (...) {
        // 忽略析构过程中的异常, 避免程序崩溃
    }
}
