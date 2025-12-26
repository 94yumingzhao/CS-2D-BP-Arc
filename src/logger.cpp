// =============================================================================
// logger.cpp - 日志系统实现
// =============================================================================
//
// 功能: 实现双输出流缓冲区和日志管理器
//
// =============================================================================

#include "logger.h"

using namespace std;

// =============================================================================
// DualStreambuf 实现
// =============================================================================

// -----------------------------------------------------------------------------
// 构造函数
// -----------------------------------------------------------------------------
DualStreambuf::DualStreambuf(streambuf* console_buf, streambuf* file_buf)
    : console_buf_(console_buf)
    , file_buf_(file_buf)
    , need_timestamp_(true) {
}

// -----------------------------------------------------------------------------
// GetCurrentTimestamp - 获取当前时间戳
// -----------------------------------------------------------------------------
string DualStreambuf::GetCurrentTimestamp() {
    auto now = chrono::system_clock::now();
    auto time_t = chrono::system_clock::to_time_t(now);
    auto ms = chrono::duration_cast<chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    stringstream ss;
    ss << "[" << put_time(localtime(&time_t), "%Y-%m-%d %H:%M:%S")
       << "." << setfill('0') << setw(3) << ms.count() << "] ";
    return ss.str();
}

// -----------------------------------------------------------------------------
// WriteTimestamp - 写入时间戳到两个缓冲区
// -----------------------------------------------------------------------------
void DualStreambuf::WriteTimestamp() {
    string timestamp = GetCurrentTimestamp();

    if (console_buf_) {
        console_buf_->sputn(timestamp.c_str(), timestamp.length());
    }

    if (file_buf_) {
        file_buf_->sputn(timestamp.c_str(), timestamp.length());
    }
}

// -----------------------------------------------------------------------------
// overflow - 单字符输出处理
// -----------------------------------------------------------------------------
int DualStreambuf::overflow(int c) {
    if (c != EOF) {
        if (need_timestamp_) {
            WriteTimestamp();
            need_timestamp_ = false;
        }

        if (console_buf_) {
            console_buf_->sputc(c);
        }
        if (file_buf_) {
            file_buf_->sputc(c);
        }

        if (c == '\n') {
            need_timestamp_ = true;
        }
    }
    return c;
}

// -----------------------------------------------------------------------------
// xsputn - 批量字符输出处理
// -----------------------------------------------------------------------------
streamsize DualStreambuf::xsputn(const char* s, streamsize count) {
    for (streamsize i = 0; i < count; ++i) {
        overflow(s[i]);
    }
    return count;
}

// =============================================================================
// Logger 实现
// =============================================================================

// -----------------------------------------------------------------------------
// 构造函数 - 初始化日志系统
// -----------------------------------------------------------------------------
Logger::Logger(const string& log_prefix)
    : old_cout_buf_(nullptr)
    , dual_buf_(nullptr) {

    log_file_path_ = log_prefix + ".log";

    log_file_.open(log_file_path_, ios::out | ios::trunc);

    if (log_file_.is_open()) {
        old_cout_buf_ = cout.rdbuf();
        dual_buf_ = make_unique<DualStreambuf>(old_cout_buf_, log_file_.rdbuf());
        cout.rdbuf(dual_buf_.get());
    }
}

// -----------------------------------------------------------------------------
// 析构函数 - 恢复标准输出并关闭日志文件
// -----------------------------------------------------------------------------
Logger::~Logger() {
    try {
        if (old_cout_buf_) {
            cout.rdbuf(old_cout_buf_);
        }

        if (log_file_.is_open()) {
            log_file_.close();
        }
    } catch (...) {
        // 忽略析构过程中的异常
    }
}
