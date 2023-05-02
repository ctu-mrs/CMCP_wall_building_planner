//
// Created by Michal Němec on 16/02/2020.
//
#include "pt2eigen.h"

namespace ctop {

Eigen::Vector3f
pt2eigen(const Point3D& pt)
{
    Eigen::Vector3f v{};
    v << pt.x, pt.y, pt.z;
    return v;
}

}
