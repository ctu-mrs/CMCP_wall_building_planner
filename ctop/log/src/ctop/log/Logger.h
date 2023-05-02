//
// Created by Michal Němec on 01/01/2020.
//

#ifndef CTOP_PLANNERLOGGER_H
#define CTOP_PLANNERLOGGER_H

#include <functional>
#include "../log.h"

namespace ctop {

enum class log_type : int {
    NORMAL,
    DEBUG,
    UNKNOWN
};

struct LogItem{
    log_type type = log_type::UNKNOWN;
    ctop::log::timestamp timestamp;
    int level = -1;
    int line = -1;
    std::string tid;
    pid_t pid;
    const char* file = nullptr;
    const char* function = nullptr;
    const char* message = nullptr;
};

using log_cb =  std::function<void(LogItem&)>;

class Logger : public BaseLogger {
    log_cb cb_ = log_cb();

public:
    Logger();

    virtual ~Logger() {
        if(log::log_ptr() == this) {
            log::reset_logger();
            CTOP_LOG_W("destroyed with active global pointer, changed logger to default");
        }
    }

    void set_on_log_listener(log_cb cb);
    void log(int level, const char* tag, const char* message) override;
    void log_debug(int level, int line, const char* file, const char* function, const char* tag, const char* message) override;

    static void default_log(const LogItem& item);
};

}

#endif //CTOP_PLANNERLOGGER_H
