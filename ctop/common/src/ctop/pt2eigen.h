//
// Created by Michal Němec on 11/03/2020.
//

#ifndef CTOP_PLANNER_PT2EIGEN_H
#define CTOP_PLANNER_PT2EIGEN_H

#include <Eigen/Dense>
#include "geometry/Point3D.h"

namespace ctop {

Eigen::Vector3f pt2eigen(const Point3D& pt);

}


#endif //CTOP_PLANNER_PT2EIGEN_H
