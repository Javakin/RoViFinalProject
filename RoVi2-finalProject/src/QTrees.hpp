//
// Created by daniel on 4/23/18.
// This class will handle the qState tree used for robotics applications
//

#ifndef OBJECTAVOIDANCE_QTREES_H
#define OBJECTAVOIDANCE_QTREES_H


// --------------------  includes ----------------------------
//RobWork includes
#include <rw/math/Q.hpp>
#include <rw/trajectory.hpp>



//std libs
#include <vector>
#include <string>

//opencv includes for exporting the image
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// --------------------  namespaces ----------------------------
using namespace rw::common;
using namespace rw::math;
using namespace std;


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
    double x;
    double y;
    Node( Q aQ, Node* aParent, double aNodeCost, double aX, double aY){
        q = aQ;
        parent = aParent;
        nodeCost = aNodeCost;
        y = aY;
        x = aX;
    }
};



class QTrees {
public:
    QTrees();
    QTrees(Q qInit, double aX, double ay);
    ~QTrees();



    void add(Q qNew, Node* nParent, double aX, double aY);
    Node* nearestNeighbor(Q qRand);
    void getRootPath(Node* lastNode, rw::trajectory::QPath& aPath);
    void setC(double C);

    void exportTree(string fileName);


private:
    vector<Node*> qTree;
    double db;
    double cb;
    double C;
};


#endif //OBJECTAVOIDANCE_QTREES_H
