//
// Created by Michal Němec on 12/9/19.
//

#ifndef CTOP_PLANNER_CLUSTERNODEDIST_H
#define CTOP_PLANNER_CLUSTERNODEDIST_H

namespace ctop {

struct ClusterNodeDist {
    int idClusterNode = -1;
    double distance = 0;

    ClusterNodeDist();
    ClusterNodeDist(int idClusterNode_, double distance_);
};

}

#endif //CTOP_PLANNER_CLUSTERNODEDIST_H