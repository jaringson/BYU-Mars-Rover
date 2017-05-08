#include "AstarPlanner.h"


using namespace std;

int main() {

    MatrixXf mapinit(10,10);
    mapinit << 1,1,1,1,1,1,1,1,1,1,
           1,1,1,1,1,1,1,1,1,1,
           1,9,1,1,1,1,1,1,1,1,
           9,9,9,9,9,9,1,1,1,1,
           1,1,1,1,1,1,1,1,1,1,
           1,1,1,1,1,1,1,1,1,1,
           1,1,1,1,1,1,1,1,1,1,
           1,1,1,1,1,1,1,1,1,1,
           1,1,1,1,1,1,1,1,1,1,
           1,1,1,1,1,1,1,1,1,1;


    Eigen::MatrixXf* map = new Eigen::MatrixXf(10,10);
    *map = mapinit;

    AstarPlanner astar(map);
    astar.SetGoal(9,0);



    //MapSearchNode* start = &astar.start;
    //cout << start->x << endl;
    astar.GetPath();
    astar.PrintMap();

    //AstarSearch<MapSearchNode> astarsearch;
    //cout << map->size() << endl;
    cout << "You Rock!" << endl;

    //astar.testfun();
    return 0;
}
