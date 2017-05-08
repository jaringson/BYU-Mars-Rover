#pragma once

#include "../../astar-algorithm-cpp/cpp/stlastar.h" // See header for copyright and usage information
#include "MapNode.h"

class AstarPlanner {
public:
    MatrixXf* map_;
    MatrixXf path_;
    AStarSearch<MapSearchNode> astarsearch;
    MapSearchNode start;
    MapSearchNode goal;


    AstarPlanner(MatrixXf *map);
    void SetGoal(int x, int y);
    MatrixXf GetPath();
    void PrintMap();

private:
    unsigned int SearchCount;
    unsigned int NumSearches;
    bool pathfound;

};