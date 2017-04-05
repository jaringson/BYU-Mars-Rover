#include "AstarPlanner.h"

using namespace std;


AstarPlanner::AstarPlanner(MatrixXf *map_) {
    this->map = map_;
    MapSearchNode node;
}

bool FunStuff( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node ) {
    AstarSearch<MapSearchNode>* astr;
    return true;
}

void AstarPlanner::testfun() {
    //AstarSearch<MapSearchNode> astr;
    cout << "Test function" << endl;
}