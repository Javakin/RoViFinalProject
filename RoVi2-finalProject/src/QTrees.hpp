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
#include <opencv2/opencv.hpp>

// for drawing the workspace
//#include <Lego.hpp>

// --------------------  namespaces ----------------------------
using namespace rw::common;
using namespace rw::math;
using namespace std;


// --------------------  defines ----------------------------
#define X_CONVEYOR             0.58
#define Y_CONVEYOR             0.15         // max is 0.15

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
    QTrees(Q qInit, double aX, double ay, double C_space, double cost_b);
    ~QTrees();



    void add(Q qNew, Node* nParent, double aX, double aY);
    Node* nearestNeighbor(Q qRand, bool constrained);
    void getRootPath(Node* lastNode, rw::trajectory::QPath& aPath);
    Node* getRootNode();
    void setC(double aC);
    void setCb(double aCb);

    double getC();
    double getCb();

    void exportTree(string fileName, vector< vector<double> > vBricks);


private:

    Node* constrainedNearestNeighbor(Q qRand);
    Node* nearestNeighbor(Q qRand);

    vector<Node*> qTree;
    double db;
    double cb;
    double C;

};


#endif //OBJECTAVOIDANCE_QTREES_H
