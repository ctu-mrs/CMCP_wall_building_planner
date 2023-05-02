//
// Created by Michal Němec on 01/01/2020.
//

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <iostream>

#include "log.h"
#include "log/ColorModifier.h"

#if MAC_OS_X_VERSION_MAX_ALLOWED >= MAC_OS_X_VERSION_10_12
#include <pthread.h>
#endif

using namespace ctop;

namespace {
    ctop::StdLogger _std_log{};
    ctop::BaseLogger* _log = static_cast<ctop::BaseLogger*>(&_std_log);
}


log::timestamp
ctop::log::current_detail_timestamp()
{
#ifdef __APPLE__
    auto now = std::chrono::system_clock::now();
#else
    auto now = std::chrono::high_resolution_clock::now();
#endif
    auto time_now = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_now);

    auto epoch = now.time_since_epoch();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(epoch).count() % 1000000LL;



    timestamp tms{};

    tms.month = 1+tm.tm_mon;
    tms.day = tm.tm_mday;
    {
        std::ostringstream oss;
        oss << std::put_time(&tm, "%F");
        tms.date = oss.str();
    }
    {
        std::ostringstream oss;
        oss << std::put_time(&tm, "%T.") << fmt::format("{:0>6}", us);
        tms.time = oss.str();
    }
    {
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Z");
        tms.timezone = oss.str();
    }
    {
        std::ostringstream oss;
        oss << std::put_time(&tm, "%z");
        tms.timezone_offset = oss.str();
    }
    return tms;
}

ctop::BaseLogger*
ctop::log::log_ptr()
{
    return _log;
}

void
ctop::log::set_logger(BaseLogger* logger)
{
    _log = logger;
}

void
ctop::log::reset_logger()
{
    _log = static_cast<ctop::BaseLogger*>(&_std_log);
}

void
StdLogger::log(int level, const char* tag, const char* message)
{
    std::cout << "(" << level << ")[" << tag << "] " << message << std::endl;
}

void
StdLogger::log_debug(int level, int line, const char* file, const char* function, const char* tag, const char* message)
{
    static log::Modifier def(log::FG_DEFAULT);
    auto tm = ctop::log::current_detail_timestamp();
    std::cout << log::level_modifier(level)
              << ctop::log::level_str(level)
              << fmt::format("{:0>2}", tm.month)
              << fmt::format("{:0>2}", tm.day)
              << " " << tm.time
              << " " << ::getpid() << " " << ctop::tid()
              << " [" <<  CTOP_FILENAME(file) << ":" << line << "] "
              << CTOP_METHOD(function) << ": "
              << message
              << def
              << std::endl;
}

std::string
ctop::log::level_str(int l) {
    if (l >= 0 && l < static_cast<int>(level::MAX_KNOWN)) {
        switch (static_cast<level>(l)) {
            case level::DEBUG:
                return "D";
            case level::ERROR:
                return "E";
            case level::INFO:
                return "I";
            case level::VERBOSE:
                return "V";
            case level::WARN:
                return "W";
            case level::ASSERT:
                return "A";
            case level::MAX_KNOWN:
                CTOP_ASSERT(0); //should not happen
                break;
            default:
                // throw it into default number handling
                break;
        }
    }
    return fmt::format("{}", l);
}

log::Modifier
ctop::log::level_modifier(int l)
{
    if (l >= 0 && l < static_cast<int>(level::MAX_KNOWN)) {
        switch (static_cast<level>(l)) {
            case level::DEBUG:
                return {log::FG_DEFAULT};
            case level::ERROR:
                return {log::FG_RED};
            case level::INFO:
                return {log::FG_GREEN};
            case level::VERBOSE:
                return {log::FG_BLUE};
            case level::WARN:
                return {log::FG_MAGENTA};
            case level::ASSERT:
                return {log::FG_RED};
            case level::MAX_KNOWN:
                CTOP_ASSERT(0); //should not happen
                break;
            default:
                // throw it into default number handling
                break;
        }
    }
    return  {log::FG_DEFAULT};
}

void
ctop::log::a(const char* expr, int line, const char* file, const char* function) {
    auto str = fmt::format("{}: Assertion \'{}\' failed", function, expr);
    log_ptr()->log_debug(static_cast<int>(level::ASSERT), line, file, function, "assert", str.c_str());
}

std::string
ctop::tid()
{
#ifdef __NR_gettid
    uint64_t th_id = syscall(__NR_gettid);
    return fmt::format("{}", th_id);
#elif SYS_thread_selfid
#if MAC_OS_X_VERSION_MAX_ALLOWED >= MAC_OS_X_VERSION_10_12
    uint64_t tid64;
    pthread_threadid_np(nullptr, &tid64);
    return fmt::format("{}", tid64);
#else
    uint64_t  th_id = syscall(SYS_thread_selfid);
    return fmt::format("{}", th_id);
#endif

#else
    auto this_th_id = std::this_thread::get_id();
    std::ostringstream oss;
    oss << this_th_id;
    return oss.str();
#endif
}

std::string
ctop::method_name(const char* fn)
{
    //const char* end_ptr = nullptr;
    const char* start_ptr = nullptr;
    const char *ptr = fn;
    while(*ptr != '\0') {
        if(*ptr == '(') {
            //end_ptr = ptr;
            break;
        }
        ptr++;
    }
    if(*ptr == '\0') {
        return fn;
    }
    int len = 0;
    while(ptr != fn) {
        if(*ptr == ':' || *ptr == ' ' || *ptr == '<') {
            start_ptr = ptr+1;
            len--;
            break;
        }
        len++;
        ptr--;
    }
    if(ptr == fn) {
        start_ptr = fn;
    }
    auto ret = std::string(start_ptr, len);
    if(ret.empty()) return fn;
    return ret;
}