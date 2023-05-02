//
// Created by Michal Němec on 11/03/2020.
//

#ifndef CTOP_PLANNER_PLANE_H
#define CTOP_PLANNER_PLANE_H

#include <Eigen/Dense>

namespace ctop {

struct Plane {
    Eigen::Vector3f p0{};
    Eigen::Vector3f w1{};
    Eigen::Vector3f w2{};

    Eigen::Vector3f eval_at(const Eigen::Vector2f& uv) {
        Eigen::Matrix<float, 3, 2> w{};
        w.col(0) = w1;
        w.col(1) = w2;
        return p0 + w*uv;
    }

};

}
#endif //CTOP_PLANNER_PLANE_H
