//
// Created by michalks on 4/8/19.
//

#include "../../../../../lib/fmt/include/fmt/format.h"
#include "timestamp.h"

using namespace ctop;

std::string
ctop::current_timestamp()
{
#ifdef __APPLE__
    auto now = std::chrono::system_clock::now();
#else
    auto now = std::chrono::high_resolution_clock::now();
#endif
    auto time = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time);

    auto epoch = now.time_since_epoch();
    auto ns = std::chrono::duration_cast<std::chrono::microseconds>(epoch).count() % 1000000LL;

    std::ostringstream oss;
    oss << std::put_time(&tm, "%FT%T.") << fmt::format("{:0>6}", ns) << std::put_time(&tm, "%z");
    return oss.str();
}

timestamp
ctop::current_detail_timestamp()
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

std::tm
ctop::string_to_tm(const std::string& date, const std::string& fmt)
{
    std::tm tm = {};
    std::stringstream ss(date);
    ss >> std::get_time(&tm, fmt.c_str());
    return tm;
}

std::string
ctop::tm_to_string(const std::tm& tm, const std::string& fmt)
{
    std::ostringstream oss;
    oss << std::put_time(&tm, fmt.c_str());
    return oss.str();
}

std::chrono::system_clock::time_point
ctop::parseISO8601(const std::string &date)
{
    std::string cp(date);
    std::tm tm = {};
    std::stringstream ss(date);
    ss >> std::get_time(&tm, ISO8601_FORMAT);
    auto time = std::chrono::system_clock::from_time_t(std::mktime(&tm));
    //remote date/h:m:s prefix
    {
        std::ostringstream oss;
        oss << std::put_time(&tm, "%FT%T");
        auto prefix =  oss.str();
        if(!prefix.empty()) {
            ctop::replace(cp, prefix, "");
        }
    }
    //remote timezone postfix
    {
        std::ostringstream oss;
        oss << std::put_time(&tm, "%z");
        auto posfix =  oss.str();
        if(!posfix.empty()) {
            ctop::replace(cp, posfix, "");
        }
    }
    //cp should contain only ".mmmsss" string
    if(!cp.empty() && cp[0] == '.') {
        cp[0] = ' ';
        //ctop::replace(cp, ".", "");
        std::size_t microsecods;
        if(cp.size() < 6) {
            cp += std::string(6-cp.size(), '0');
        }
        std::stringstream us_ss(cp);
        if(us_ss >> microsecods) {
            time += std::chrono::microseconds{microsecods};
        }
    }
    return time;
}

std::string
ctop::toISO8601(const std::chrono::system_clock::time_point& dt) {
    auto time = std::chrono::system_clock::to_time_t(dt);
    auto tm = *std::localtime(&time);
    auto epoch = dt.time_since_epoch();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(epoch).count() % 1000000LL;
    std::ostringstream oss;
    if(us == 0) {
        oss << std::put_time(&tm, ISO8601_FORMAT);
    } else {
        // trim zero values at the end
        // .130000 -> .13
        uint32_t prev = us; //us is <0, 999999> uint32 is enough
        int places_zero = 6;
        while(places_zero != 0) {
            if(prev % 10 == 0) {
                prev /= 10;
                --places_zero;
            } else {
                break;
            }
        }
        if(places_zero == 0) {
            oss << std::put_time(&tm, ISO8601_FORMAT);
        } else {
            oss << std::put_time(&tm, "%FT%T.") << fmt::format("{:0>{}}", prev, places_zero) << std::put_time(&tm, "%z");
        }
    }
    return oss.str();
}