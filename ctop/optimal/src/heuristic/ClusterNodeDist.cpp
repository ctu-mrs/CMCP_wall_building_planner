//
// Created by Michal Němec on 12/9/19.
//

#include "ClusterNodeDist.h"

using namespace ctop;

ClusterNodeDist::ClusterNodeDist() = default;

ClusterNodeDist::ClusterNodeDist(int idClusterNode_, double distance_)
: idClusterNode(idClusterNode_), distance(distance_)
{}