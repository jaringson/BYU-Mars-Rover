#include "MapNode.h"


using namespace std;
using namespace Eigen;

// Global data

// The world map

const int MAP_WIDTH = 20;
const int MAP_HEIGHT = 20;

/*

int world_map[ MAP_WIDTH * MAP_HEIGHT ] = 
{

// 0001020304050607080910111213141516171819
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,   // 00
    1,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,1,   // 01
    1,9,9,1,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 02
    1,9,9,1,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 03
    1,9,1,1,1,1,9,9,1,9,1,9,1,1,1,1,9,9,1,1,   // 04
    1,9,1,1,9,1,1,1,1,9,1,1,1,1,9,1,1,1,1,1,   // 05
    1,9,9,9,9,1,1,1,1,1,1,9,9,9,9,1,1,1,1,1,   // 06
    1,9,9,9,9,9,9,9,9,1,1,1,9,9,9,9,9,9,9,1,   // 07
    1,9,1,1,1,1,1,1,1,1,1,9,1,1,1,1,1,1,1,1,   // 08
    1,9,1,9,9,9,9,9,9,9,1,1,9,9,9,9,9,9,9,1,   // 09
    1,9,1,1,1,1,9,1,1,9,1,1,1,1,1,1,1,1,1,1,   // 10
    1,9,9,9,9,9,1,9,1,9,1,9,9,9,9,9,1,1,1,1,   // 11
    1,9,1,9,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 12
    1,9,1,9,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 13
    1,9,1,1,1,1,9,9,1,9,1,9,1,1,1,1,9,9,1,1,   // 14
    1,9,1,1,9,1,1,1,1,9,1,1,1,1,9,1,1,1,1,1,   // 15
    1,9,9,9,9,1,1,1,1,1,1,9,9,9,9,1,1,1,1,1,   // 16
    1,1,9,9,9,9,9,9,9,1,1,1,9,9,9,1,9,9,9,9,   // 17
    1,9,1,1,1,1,1,1,1,1,1,9,1,1,1,1,1,1,1,1,   // 18
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,   // 19

};

// map helper functions

int GetMap( int x, int y )
{
    if( x < 0 ||
        x >= MAP_WIDTH ||
         y < 0 ||
         y >= MAP_HEIGHT
      )
    {
        return 9;    
    }

    return world_map[(y*MAP_WIDTH)+x];
}
*/


// Definitions



bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{

    // same state in a maze search is simply when (x,y) are the same
    if( (x == rhs.x) &&
        (y == rhs.y) )
    {
        return true;
    }
    else
    {
        return false;
    }

}

void MapSearchNode::PrintNodeInfo()
{
    char str[100];
    sprintf( str, "Node position : (%d,%d)\n", x,y );

    cout << str;
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal. 

float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
    return fabsf(x - nodeGoal.x) + fabsf(y - nodeGoal.y);   
}

bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
{

    if( (x == nodeGoal.x) &&
        (y == nodeGoal.y) )
    {
        return true;
    }

    return false;
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool MapSearchNode::GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node )
{

    int parent_x = -1; 
    int parent_y = -1; 
    
    if( parent_node )
    {
        parent_x = parent_node->x;
        parent_y = parent_node->y;
    }
    
    MapSearchNode NewNode;

    //cout << "x=" << x << " y=" << y << endl;
    //cout << "px=" << parent_x << " py=" << parent_y << endl;
    // push each possible move except allowing the search to go backwards

    //cout << (GetMap( x-1, y ) < 9) << " " << !((parent_x == x-1) && (parent_y == y)) << endl;
    if( (GetMap( x-1, y ) < 9) 
        && !((parent_x == x-1) && (parent_y == y))
      ) 
    {
        NewNode = MapSearchNode( x-1, y, map_ );
        astarsearch->AddSuccessor( NewNode );
        //cout << "Top" << endl;
    }   

    //cout << (GetMap( x, y-1 ) < 9)<< " " << !((parent_x == x) && (parent_y == y-1)) << endl;
    if( (GetMap( x, y-1 ) < 9) 
        && !((parent_x == x) && (parent_y == y-1))
      ) 
    {
        NewNode = MapSearchNode( x, y-1, map_ );
        astarsearch->AddSuccessor( NewNode );
        //cout << "Left" << endl;
    }   

    //cout << (GetMap( x+1, y ) < 9)<< " " << !((parent_x == x+1) && (parent_y == y)) << endl;
    if( (GetMap( x+1, y ) < 9)
        && !((parent_x == x+1) && (parent_y == y))
      ) 
    {
        NewNode = MapSearchNode( x+1, y, map_ );
        astarsearch->AddSuccessor( NewNode );
        //cout << "Bottom" << endl;
    }   

    //cout << (GetMap( x, y+1 ) < 9)<< " " << !((parent_x == x) && (parent_y == y+1)) << endl;    
    if( (GetMap( x, y+1 ) < 9) 
        && !((parent_x == x) && (parent_y == y+1))
        )
    {
        NewNode = MapSearchNode( x, y+1, map_ );
        astarsearch->AddSuccessor( NewNode );
        //cout << "Right" << endl;
    }   
    
    return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is 
// conceptually where we're moving

float MapSearchNode::GetCost( MapSearchNode &successor )
{
    return (float) GetMap(x,y);

}

int MapSearchNode::GetMap(int xind, int yind){
    if ((xind >= 0 && xind < map_->rows() ) && 
       (yind >= 0 && yind < map_->cols() )) {
        return (*map_)( xind, yind );
    }
    else {
        return 9;
    }
}



