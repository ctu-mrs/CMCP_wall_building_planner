/**
 * MathCommon.h
 *
 *  Created on: 16. 11. 2014
 *    @au Robert Pěnička
 */

#ifndef MATHCOMMON_H_
#define MATHCOMMON_H_

#include <cstdlib>
#include <ctime>
#include <cmath>
#include <cfloat>
#include <vector>

#ifndef PI_
    #define PI_ 3.1415926535897932385
#endif

#define M_2PI (2 * PI_)
#define POW(x) ((x)*(x))
#define MIN(x,y) ((x > y) ? y : x)
#define MAX(x,y) ((x > y) ? ( x ) : ( y ))
#define ABS(x) ((x < 0) ? (-(x)) : ( x ))
#define ANGLE_MIN (0)
#define ANGLE_MAX (M_2PI)

namespace ctop {

double randDoubleMinMax(double min, double max);
int randIntMinMax(int min, int max);

/**
 * Normalizes angle in range <0,2pi].
 * @param angle
 * @return angle in range <0,2pi]
 */
double normalizeAngle(double angle,double min=ANGLE_MIN,double max=ANGLE_MAX);
double diffAngle(double a1, double a2);

std::vector<double> range(double min ,double max,double step = 1.0);

template <typename T> int sgn(T val) {
    if (val > 0) {
        return 1;
    } else if (val < 0) {
        return -1;
    } else {
        return 0;
    }
}

}

#endif /* MATHCOMMON_H_ */
