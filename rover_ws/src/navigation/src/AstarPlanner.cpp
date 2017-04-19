#include "AstarPlanner.h"
#include <vector>

using namespace std;

#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0


AstarPlanner::AstarPlanner(MatrixXf *map) {
    this->map_ = map;
    MapSearchNode node;

    start = MapSearchNode(0,0,map);
    goal = MapSearchNode(5,0,map);

    SearchCount = 0;
    NumSearches = 1;
    pathfound = false;
}

void AstarPlanner::SetGoal(int x, int y) {
    goal.x = x;
    goal.y = y;
    pathfound = false;
}

void AstarPlanner::PrintMap() {
    MatrixXf map = *map_;
    map(start.x,start.y) = 10;
    int i = 0;
    if (pathfound){
        for (;i<path_.rows();i++){
            map(path_(i,0),path_(i,1)) = i + 11;
        }
    }
    map(goal.x, goal.y)  = i + 10;
    cout << map << endl;
}

MatrixXf AstarPlanner::GetPath() {
    vector<int> x;
    vector<int> y;

    // Set Start and goal states

    astarsearch.SetStartAndGoalStates( start, goal );

    unsigned int SearchState;
    unsigned int SearchSteps = 0;
    int steps = 0;

    //start.GetSuccessors(&astarsearch, &goal);

    do
    {
        SearchState = astarsearch.SearchStep();

        SearchSteps++;

        #if DEBUG_LISTS

            cout << "Steps:" << SearchSteps << "\n";

            int len = 0;

            cout << "Open:\n";
            MapSearchNode *p = astarsearch.GetOpenListStart();
            while( p )
            {
                len++;
                #if !DEBUG_LIST_LENGTHS_ONLY
                    ((MapSearchNode *)p)->PrintNodeInfo();
                #endif
                p = astarsearch.GetOpenListNext();

            }

            cout << "Open list has " << len << " nodes\n";

            len = 0;

            cout << "Closed:\n";
            p = astarsearch.GetClosedListStart();
            while( p )
            {
                len++;
                #if !DEBUG_LIST_LENGTHS_ONLY
                    p->PrintNodeInfo();
                #endif
                p = astarsearch.GetClosedListNext();
            }

            cout << "Closed list has " << len << " nodes\n";
        #endif

    } while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );

    if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
    {
        cout << "Search found goal state\n";

        MapSearchNode *node = astarsearch.GetSolutionStart();

        #if DISPLAY_SOLUTION
            cout << "Displaying solution\n";
        #endif


        node->PrintNodeInfo();
        for( ;; )
        {
            node = astarsearch.GetSolutionNext();

            if( !node )
            {
                break;
            }

            node->PrintNodeInfo();
            x.push_back(node->x);
            y.push_back(node->y);
            steps ++;

        };

        cout << "Solution steps " << steps << endl;

        // Once you're done with the solution you can free the nodes up
        astarsearch.FreeSolutionNodes();
        pathfound = true;

    }
    else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED )
    {
        cout << "Search terminated. Did not find goal state\n";

    }

    // Display the number of loops the search went through
    cout << "SearchSteps : " << SearchSteps << "\n";

    SearchCount ++;

    astarsearch.EnsureMemoryFreed();

    MatrixXf path(steps,2);
    for (int i=0; i<steps; i++){
        path(i,0) = x[i];
        path(i,1) = y[i];
    }
    path_ = path;
    return path;
}
