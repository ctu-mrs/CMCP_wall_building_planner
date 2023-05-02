//
// Created by michalks on 8/10/18.
//

#include "readable_size.h"

namespace ctop {

std::string readable_fs(double size, bool siu) {
    int i = 0;
    static std::string units[] = {"B", "kB", "MB", "GB", "TB", "PB", "EB", "ZB", "YB"};
    static std::string units2[] = {"B", "KiB", "MiB", "GiB", "TiB", "PiB", "EiB", "ZiB", "YiB"};
    double div = siu ? 1024.0 : 1000.0;
    while (size > div) {
        size /= div;
        i++;
    }
    std::ostringstream oss;
    oss << std::setprecision(4) << size << " " << (siu ? units2[i] : units[i]);
    return oss.str();
}

}
