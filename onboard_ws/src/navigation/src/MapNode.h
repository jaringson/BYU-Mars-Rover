#pragma once

#include "../../astar-algorithm-cpp/cpp/stlastar.h" // See header for copyright and usage information
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0



class MapSearchNode
{
public:
    int x;   // the (x,y) positions of the node
    int y;

    MatrixXf* map_; // Pointer to the map
    
    MapSearchNode() { x = y = 0; }
    MapSearchNode( int px, int py, MatrixXf* map ) { x=px; y=py; map_ = map; }



    float GoalDistanceEstimate( MapSearchNode &nodeGoal );
    bool IsGoal( MapSearchNode &nodeGoal );
    bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
    float GetCost( MapSearchNode &successor );
    bool IsSameState( MapSearchNode &rhs );

    void PrintNodeInfo(); 
    int GetMap(int xind, int yind);

};




