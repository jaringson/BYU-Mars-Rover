#pragma once

#include "../../astar-algorithm-cpp/cpp/stlastar.h" // See header for copyright and usage information
#include "MapNode.h"

class AstarPlanner {
public:
    MatrixXf* map;

    AstarPlanner(MatrixXf *map_);

    bool FunStuff( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
    //AstarSearch *astar_srch:

    void testfun();
    
};



