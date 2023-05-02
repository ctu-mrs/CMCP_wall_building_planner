//
// Created by Michal Němec on 01/01/2020.
//

#include <chrono>
#include <sstream>
#include "Logger.h"

using namespace ctop;


Logger::Logger() = default;

void
Logger::set_on_log_listener(log_cb cb)
{
    cb_ = std::move(cb);
}

void
Logger::log(int level, const char* tag, const char* message)
{
    LogItem item{};
    item.tid = ctop::tid();
    item.pid = ::getpid();
    item.timestamp = ctop::log::current_detail_timestamp();
    item.type = log_type::NORMAL;
    item.message = message;
    item.level = level;
    cb_(item);
}

void
Logger::log_debug(int level, int line, const char* file, const char* function, const char* tag, const char* message)
{
    LogItem item{};
    item.tid = ctop::tid();
    item.pid = ::getpid();
    item.timestamp = ctop::log::current_detail_timestamp();
    item.type = log_type::DEBUG;
    item.message = message;
    item.level = level;
    item.line = line;
    item.function = function;
    item.file = file;
    cb_(item);
}

void
Logger::default_log(const LogItem& item)
{
    static log::Modifier def(log::FG_DEFAULT);
    std::cout
            << log::level_modifier(item.level)
            << ctop::log::level_str(item.level)
            << fmt::format("{:0>2}", item.timestamp.month)
            << fmt::format("{:0>2}", item.timestamp.day)
            << " "
            << item.timestamp.time << " ";
#ifdef CTOP_USE_DEBUG_LOG
    std::cout  << item.pid << " " << item.tid
               << " [" <<  CTOP_FILENAME(item.file) << ":" << item.line << "] " << CTOP_METHOD(item.function) << ": " << item.message;
#else
    std::cout << "(" << item.level << ")[" << item.tag << "] " << item.message;
#endif
    std::cout << def << std::endl;
}