#include "AstarPlanner.h"


using namespace std;

int main() {

    MapSearchNode node;
    node.testfun2();
    
    Eigen::MatrixXf* map = new Eigen::MatrixXf(10,10);
    AstarPlanner astar(map);
    astar.testfun();
    //AstarSearch<MapSearchNode> astarsearch;

    cout << "You Rock!" << endl;

    //astar.testfun();
    return 0;
}