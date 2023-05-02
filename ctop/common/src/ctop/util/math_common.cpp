/*
 * MathCommon.cpp
 *
 *  Created on: 16. 11. 2014
 *      Author: Robert Pěnička
 */

#include "math_common.h"
#include <random>
#include <chrono>

double ctop::randDoubleMinMax(double min, double max) {
    // initialize the random number generator with time-dependent seed
    uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    auto lower_int = uint32_t(timeSeed & 0xffffffff);
    auto upper_int = uint32_t(timeSeed>>32);

    std::seed_seq ss{lower_int, upper_int};
    std::mt19937_64 rng{ss};

    std::uniform_real_distribution<double> unif(min, max);

    return unif(rng);
}

int ctop::randIntMinMax(int min, int max) {
    // initialize the random number generator with time-dependent seed
    uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    auto lower_int = uint32_t(timeSeed & 0xffffffff);
    auto upper_int = uint32_t(timeSeed>>32);

    std::seed_seq ss{lower_int, upper_int};
    std::mt19937_64 rng{ss};

    std::uniform_int_distribution<int> unif(min, max);

	return unif(rng);
}

double ctop::normalizeAngle(double angle, double min, double max) {
	double normalized = angle;
	while (normalized < min) {
		normalized += M_2PI;
	}
	while (normalized > max) {
		normalized -= M_2PI;
	}
	return normalized;
}

double ctop::diffAngle(double a1, double a2) {
    auto c1 = std::cos(a1);
    auto s1 = std::sin(a1);

    auto c2 = std::cos(a2);
    auto s2 = std::sin(a2);

    auto dot = c1*c2 + s1*s2;
    auto cross = c1*s2 - s1*c2;

    return std::atan2(cross, dot);
}

std::vector<double> ctop::range(double min, double max, double step) {
    constexpr double tol = 10e-7;
	std::vector<double> range_values;
	if (min <= max) {
        auto sz = static_cast<int>((max-min)/step);
        range_values.reserve(sz);
        double val = 0;
        auto diff = std::abs(max-val);
		while(diff > tol && val < max) {
            val = val + step;
            diff = std::abs(max-val);
            if(diff <= tol) {
                range_values.push_back(val);
            }
        }
		range_values.push_back(max);
	}
	return range_values;
}
