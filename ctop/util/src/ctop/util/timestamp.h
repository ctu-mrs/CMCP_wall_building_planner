//
// Created by michalks on 4/8/19.
//

#ifndef CTOP_PLANNERTIMESTAMP_H
#define CTOP_PLANNERTIMESTAMP_H

#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include "../../../../../lib/fmt/include/fmt/format.h"

#include "replace.h"

#define ISO8601_FORMAT "%FT%T%z"

namespace ctop {

struct timestamp {
    std::string date;
    std::string time;
    std::string timezone;
    std::string timezone_offset;
};

/**
 * Return actual time in standard format
 * @return current time "YYYY-MM-DD'T'HH:MM:SS.UUUSSS'Z(+-TIMEZONE)'", eg. "2019-05-11T12:01:10.123456+0100"
 */
std::string current_timestamp();

timestamp current_detail_timestamp();

std::tm string_to_tm(const std::string& date, const std::string& fmt = ISO8601_FORMAT);
std::string tm_to_string(const std::tm& tm, const std::string& fmt = ISO8601_FORMAT);

template <class Rep, class Period>
std::string epoch_to_string(const std::chrono::duration<Rep, Period>& duration, const std::string& fmt = ISO8601_FORMAT) {
    std::chrono::time_point<std::chrono::system_clock> dt(duration);
    auto time = std::chrono::system_clock::to_time_t(dt);
    auto tm = *std::localtime(&time);
    std::ostringstream oss;
    oss << std::put_time(&tm, fmt.c_str());
    return oss.str();
}

template <class Rep, class Period>
std::chrono::system_clock::time_point epoch_to_time_point(const std::chrono::duration<Rep, Period>& duration) {
    return std::chrono::system_clock::time_point{duration};
}

template <class T>
std::size_t string_to_epoch(const std::string& date, const std::string& fmt = ISO8601_FORMAT) {
    auto tm = string_to_tm(date, fmt);
    auto time = std::chrono::system_clock::from_time_t(std::mktime(&tm));
    return std::chrono::duration_cast<T>(time.time_since_epoch()).count();
}

std::chrono::system_clock::time_point parseISO8601(const std::string &date);
std::string toISO8601(const std::chrono::system_clock::time_point& dt);

std::chrono::high_resolution_clock::time_point parseISO8601_hi(const std::string &date);
std::string toISO8601_hi(const std::chrono::high_resolution_clock::time_point& dt);


template <class Rep, class Period>
std::string toISO8601(const std::chrono::duration<Rep, Period>& duration) {
    std::chrono::time_point<std::chrono::system_clock> dt(duration);
    return toISO8601(dt);
}

template <class Rep, class Period>
std::string epoch_toISO8601(const std::chrono::duration<Rep, Period>& duration) {
    return toISO8601(ctop::epoch_to_time_point(duration));
}

template <class Rep, class Period>
std::string duration_to_string(const std::chrono::duration<Rep, Period>& diff) {
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(diff);
    auto wall_time = ns.count();
    if(wall_time == 0) {
        return "0 ns";
    }
    auto nano = wall_time%1000LL;
    auto micro = (wall_time/1000LL)%1000LL;
    auto milli = (wall_time/1000000LL)%1000LL;
    auto secs_base = wall_time/1000000000LL;
    auto hours = secs_base/3600LL;
    auto minutes = (secs_base/60LL)%60LL;
    auto seconds = secs_base%60LL;

    bool enable_tm[6];
    bool enable_tm_back[6];
    int64_t enable_tm_vals[6] = { nano, micro, milli, seconds, minutes, hours };
    for(int i = 0; i != 6; ++i) {
        enable_tm_back[i] = (enable_tm_vals[i] == 0);
        enable_tm[i] = (enable_tm_vals[i] > 0);
    }
    const char* enable_tm_end[6] = { "ns", "us", "ms", "s", "min", "h"};
    uint8_t enable_tm_sizes[6] = { 3, 3, 3, 2, 2, 0};
    uint8_t enable_tm_separator[6] = { '\0', '\'', '\'', '.', ':', ':'};

    // formatting is done in format
    // format: hh::mm::ss.mmm'uuu'nnnn
    //    idx:  5   4   3   2   1    0
    // we want to exclude highest zero values
    {
        // enable_tm contains all positions that will be formatted
        // eg. 00:30:00.333222000 enable_tm would contain
        // enable_tm[0] = false
        // enable_tm[1] = true
        // enable_tm[2] = true
        // enable_tm[3] = false
        // enable_tm[4] = true
        // enable_tm[5] = false
        //
        // after loop we set true value to all before minutes
        // enable_tm[0] = true
        // enable_tm[1] = true
        // enable_tm[2] = true
        // enable_tm[3] = true
        // enable_tm[4] = true
        // enable_tm[5] = false
        //
        // these values will be formatted
        auto prev = enable_tm[5];
        for(uint8_t i = 4; i != 0; i--) {
            if(prev) {
                enable_tm[i] = true;
            }
            prev = enable_tm[i];
        }
    }

    // find first nonzero element in time elements
    // format: hh::mm::ss.mmm'uuu'nnnn
    //    idx:  5   4   3   2   1    0
    // if nanosecods and microseconds are zero then we want to exclude them
    // stop_i would be 2
    // from previous example if we are printing 00:30:00.333222000
    // we would format only values 30:00.333222
    //  ns: enable_tm[0] = true -> nanoseconds contains zero
    //  us: enable_tm[1] = true -> format only microseconds to minutes
    //  ms: enable_tm[2] = true
    //   s: enable_tm[3] = true
    // min: enable_tm[4] = true
    //   h: enable_tm[5] = false
    uint8_t stop_i = 0;
    for(; stop_i != 6; stop_i++) {
        if(!enable_tm_back[stop_i]) break;
    }

    std::ostringstream oss;
    {
        auto prev = false;
        const char* end = nullptr;
        for(int8_t i = 5; i != -1; i--) {
            if(enable_tm[i]) {
                char sep = enable_tm_separator[i];
                if(!prev) {
                    oss << fmt::format("{}", enable_tm_vals[i]);
                    if(sep == '\'') {
                        // separator '\'' is for numbers behand decimal place
                        // if time is sub-seconds we want to use '.' instead
                        // eg. 32'333 us -> 32.333 us
                        sep = '.';
                    }
                    end = enable_tm_end[i];
                } else {
                    // i < 3 is for nano, micro, mili sec (idx 0,1,2)
                    if(i < 3 && i == stop_i) {
                        // when formating last element behind decimal place
                        // cutoff all zeroes at the end
                        // 1.101'100 ms -> 1.101'1 ms
                        auto reduced = enable_tm_vals[i];
                        int places_zero = 3;
                        while(places_zero != 0) {
                            if(reduced % 10 == 0) {
                                reduced /= 10;
                                --places_zero;
                            } else {
                                break;
                            }
                        }
                        if(places_zero > 0) {
                            oss << fmt::format("{:0>{}}", reduced, places_zero);
                        }
                    } else {
                        oss << fmt::format("{:0>{}}", enable_tm_vals[i], enable_tm_sizes[i]);
                    }
                }
                if(i > stop_i) {
                    if(sep != '\0') {
                        oss << sep;
                    }
                }
            }
            if(i == stop_i) {
                break;
            }
            prev = enable_tm[i];
        }
        if(end != nullptr) {
            oss << " " << end;
        } else {
            // should not happen
            oss << "0 ns";
        }
    }
    return oss.str();
}

}


#endif //CTOP_PLANNERTIMESTAMP_H
