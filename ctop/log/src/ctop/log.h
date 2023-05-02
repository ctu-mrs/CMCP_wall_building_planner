//
// Created by Michal Němec on 01/01/2020.
//

#ifndef CTOP_PLANNERLOG_H
#define CTOP_PLANNERLOG_H

#include <iostream>
#include <cassert>
#include "../../../../lib/fmt/include/fmt/format.h"


#ifdef __linux__
#include <unistd.h>
#include <sys/syscall.h>
#elif __APPLE__
#include <unistd.h>
#include <sys/syscall.h>
#else
#include <thread>
#endif

#include <cstring>
#include "log/ColorModifier.h"

#define CTOP_FILENAME(x) (strrchr(x, '/') ? strrchr(x, '/') + 1 : x)
#define CTOP_METHOD(x) ctop::method_name(x)

namespace ctop {

std::string tid();
std::string method_name(const char* fn);

struct BaseLogger {
    virtual void log(int level, const char* tag, const char* message) = 0;
    virtual void log_debug(int level, int line, const char* file, const char* function, const char* tag, const char* message) = 0;
};

struct StdLogger : public BaseLogger{
    void log(int level, const char* tag, const char* message) override;
    void log_debug(int level, int line, const char* file, const char* function, const char* tag, const char* message) override;
};

namespace log {

struct timestamp {
    uint8_t month;
    uint8_t day;
    std::string date;
    std::string time;
    std::string timezone;
    std::string timezone_offset;
};

timestamp current_detail_timestamp();

BaseLogger* log_ptr();
void set_logger(BaseLogger* logger);
void reset_logger();

enum class level : int {
    DEBUG = 0,
    ERROR,
    INFO,
    VERBOSE,
    WARN,
    ASSERT,
    MAX_KNOWN
};

log::Modifier level_modifier(int level);
std::string level_str(int level);

template<typename... Args>
void d_debug(const char* tag, int line, const char* file, const char* function, const std::string& format, Args&&... args) {
    auto str = fmt::format(format, std::forward<Args>(args)...);
    log_ptr()->log_debug(static_cast<int>(level::DEBUG), line, file, function, tag, str.c_str());
}

template<typename... Args>
void e_debug(const char* tag, int line, const char* file, const char* function, const std::string& format, Args&&... args) {
    auto str = fmt::format(format, std::forward<Args>(args)...);
    log_ptr()->log_debug(static_cast<int>(level::ERROR), line, file, function, tag, str.c_str());
}

template<typename... Args>
void i_debug(const char* tag, int line, const char* file, const char* function, const std::string& format, Args&&... args) {
    auto str = fmt::format(format, std::forward<Args>(args)...);
    log_ptr()->log_debug(static_cast<int>(level::INFO), line, file, function, tag, str.c_str());
}

template<typename... Args>
void v_debug(const char* tag, int line, const char* file, const char* function, const std::string& format, Args&&... args) {
    auto str = fmt::format(format, std::forward<Args>(args)...);
    log_ptr()->log_debug(static_cast<int>(level::VERBOSE), line, file, function, tag, str.c_str());
}

template<typename... Args>
void w_debug(const char* tag, int line, const char* file, const char* function, const std::string& format, Args&&... args) {
    auto str = fmt::format(format, std::forward<Args>(args)...);
    log_ptr()->log_debug(static_cast<int>(level::WARN), line, file, function, tag, str.c_str());
}

template<typename... Args>
void d(const char* tag, const std::string& format, Args&&... args) {
    auto str = fmt::format(format, std::forward<Args>(args)...);
    log_ptr()->log(static_cast<int>(level::DEBUG), tag, str.c_str());
}

template<typename... Args>
void e(const char* tag, const std::string& format, Args&&... args) {
    auto str = fmt::format(format, std::forward<Args>(args)...);
    log_ptr()->log(static_cast<int>(level::ERROR), tag, str.c_str());
}

template<typename... Args>
void i(const char* tag, const std::string& format, Args&&... args) {
    auto str = fmt::format(format, std::forward<Args>(args)...);
    log_ptr()->log(static_cast<int>(level::INFO), tag, str.c_str());
}

template<typename... Args>
void v(const char* tag, const std::string& format, Args&&... args) {
    auto str = fmt::format(format, std::forward<Args>(args)...);
    log_ptr()->log(static_cast<int>(level::VERBOSE), tag, str.c_str());
}

template<typename... Args>
void w(const char* tag, const std::string& format, Args&&... args) {
    auto str = fmt::format(format, std::forward<Args>(args)...);
    log_ptr()->log(static_cast<int>(level::WARN), tag, str.c_str());
}

void a(const char* expr, int line, const char* file, const char* function);

}

}


#define CTOP_USE_DEBUG_LOG
#define CTOP_LOG_PTR(x) static_cast<void*>(x)

#if defined(__GNUC__) || defined(__clang__)
#define CTOP_FUNCTION __PRETTY_FUNCTION__
#elif defined(_MSC_VER)
#define CTOP_FUNCTION __FUNCSIG__
#elif defined(__SUNPRO_CC)
#define CTOP_FUNCTION __func__
#else
#define CTOP_FUNCTION __FUNCTION__
#endif

#ifndef NDEBUG
#define CTOP_ASSERT(expr) \
    do { \
        if (!static_cast <bool>(expr)) { \
            ctop::log::a(#expr, __LINE__, __FILE__, CTOP_FUNCTION); \
        } \
        assert(expr); \
    } while (false)
#else
#define CTOP_ASSERT(tag, expr) assert(expr)
#endif

#ifdef CTOP_ENABLE_LOG
    #define CTOP_LOG_COMMA ,
    #ifdef CTOP_USE_DEBUG_LOG
        #define CTOP_LOG_D(...) ctop::log::d_debug("", __LINE__, __FILE__, CTOP_FUNCTION, __VA_ARGS__)
        #define CTOP_LOG_E(...) ctop::log::e_debug("", __LINE__, __FILE__, CTOP_FUNCTION, __VA_ARGS__)
        #define CTOP_LOG_I(...) ctop::log::i_debug("", __LINE__, __FILE__, CTOP_FUNCTION, __VA_ARGS__)
        #define CTOP_LOG_V(...) ctop::log::v_debug("", __LINE__, __FILE__, CTOP_FUNCTION, __VA_ARGS__)
        #define CTOP_LOG_W(...) ctop::log::w_debug("", __LINE__, __FILE__, CTOP_FUNCTION, __VA_ARGS__)
    #else
        #define CTOP_LOG_D(...) ctop::log::d(__VA_ARGS__)
        #define CTOP_LOG_E(...) ctop::log::e(__VA_ARGS__)
        #define CTOP_LOG_I(...) ctop::log::i(__VA_ARGS__)
        #define CTOP_LOG_V(...) ctop::log::v(__VA_ARGS__)
        #define CTOP_LOG_W(...) ctop::log::w(__VA_ARGS__)
    #endif
#else
    #define CTOP_LOG_COMMA
    #define CTOP_LOG_D(...)
    #define CTOP_LOG_E(...)
    #define CTOP_LOG_I(...)
    #define CTOP_LOG_V(...)
    #define CTOP_LOG_W(...)
#endif
#endif //CTOP_PLANNERLOG_H
