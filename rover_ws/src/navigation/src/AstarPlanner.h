
#include "astar.h"

class AstarPlanner {
public:
    MatrixXf* map;

    AstarPlanner(MatrixXf *map_);
    //AstarSearch<MapSearchNode> astarsearch:

    void testfun();
    
};


AstarPlanner::AstarPlanner(MatrixXf *map_) {
    this->map = map_;
    //AstarSearch<MapSearchNode> astarsearch;
    MapSearchNode node;
}

void AstarPlanner::testfun() {
    cout << "Test function" << endl;
}