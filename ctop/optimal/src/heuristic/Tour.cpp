//
// Created by Michal Němec on 12/9/19.
//

#include "Tour.h"

using namespace ctop;

Tour::Tour()
: reward(0), length(0)
{}

Tour::Tour(double reward_, double length_, std::vector<GraphNode> tour_)
: reward(reward_), length(length_), tour(std::move(tour_))
{}