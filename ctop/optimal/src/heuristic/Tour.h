//
// Created by Michal Němec on 12/9/19.
//

#ifndef CTOP_PLANNER_TOUR_H
#define CTOP_PLANNER_TOUR_H

#include <vector>

#include "GraphNode.h"

namespace ctop {

struct Tour {

    double reward;
    double length;
    std::vector<GraphNode> tour;

    Tour();
    Tour(double reward_, double length_, std::vector<GraphNode> tour_);

    /*
    std::vector<Point> getPointTour() {
        std::vector<Point> pointTour;
        int S = tour.size();
        for (int var = 0; var < S; ++var) {
            pointTour.push_back(tour[var].toPoint());
        }
        return pointTour;
    }
    */
};

}

#endif //CTOP_PLANNER_TOUR_H
