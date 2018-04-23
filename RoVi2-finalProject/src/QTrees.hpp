//
// Created by daniel on 4/23/18.
// This class will handle the qState tree used for robotics applications
//

#ifndef OBJECTAVOIDANCE_QTREES_H
#define OBJECTAVOIDANCE_QTREES_H


// --------------------  includes ----------------------------
//RobWork includes
#include <rw/math/Q.hpp>



// --------------------  namespaces ----------------------------
using namespace rw::common;
using namespace rw::math;

using namespace std;



// --------------------  defines ----------------------------
#define MAX_DISTANCE    1000



class QTrees {
public:
    QTrees();
    QTrees(Q qInit);
    ~QTrees();

    unsigned int addVertex(Q vertex);
    void addEdge(unsigned int p1, unsigned int p2);
    int nearestVertex(Q qNew);
    Q getVertex(unsigned int uiVertex);

    void clear();

private:
    //The tree
    vector<Q> qVertex;
    vector<pair<unsigned int,unsigned int> > qTree;
};


#endif //OBJECTAVOIDANCE_QTREES_H
