//
// Created by daniel on 4/23/18.
// This class will handle the qState tree used for robotics applications
//

#ifndef OBJECTAVOIDANCE_QTREES_H
#define OBJECTAVOIDANCE_QTREES_H


// --------------------  includes ----------------------------
//RobWork includes
#include <rw/math/Q.hpp>
//#include <rwlibs/pathplanners/rrt/RRTTree.hpp>
//#include <rw/common/PropertyType.hpp>
#include <rw/trajectory.hpp>

//std libs
#include <vector>

// --------------------  namespaces ----------------------------
using namespace rw::common;
using namespace rw::math;
using namespace std;
//using namespace rwlibs::pathplanners;


// --------------------  defines ----------------------------
//#define MAX_DISTANCE    1000

/*namespace
{
    typedef RRTNode<rw::math::Q> Node;
    typedef RRTTree<rw::math::Q> Tree;
}*/


struct Node {
    Q q;
    Node* parent;
    double nodeCost;
    Node( Q aQ, Node* aParent, double aNodeCost){
        q = aQ;
        parent = aParent;
        nodeCost = aNodeCost;
    }
};



class QTrees {
public:
    QTrees();
    QTrees(Q qInit);
    ~QTrees();



    void add(Q qNew, Node* nParent);
    Node* nearestNeighbor(Q qRand);
    void getRootPath(Node* lastNode, rw::trajectory::QPath& aPath);
    void setC(double C);


private:
    vector<Node*> qTree;
    double db;
    double cb;
    double C;
};


#endif //OBJECTAVOIDANCE_QTREES_H
