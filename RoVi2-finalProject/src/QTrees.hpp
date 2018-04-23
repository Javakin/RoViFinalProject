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




class QTrees {
public:
    QTrees();
    ~QTrees();

    bool addVertex(Q vertex);
    bool addEdge(unsigned int p1, unsigned int p2);

    void clear();

private:
    //The tree
    vector<Q> qVertex;
    vector<pair<unsigned int,unsigned int>> qTree;
};


#endif //OBJECTAVOIDANCE_QTREES_H
