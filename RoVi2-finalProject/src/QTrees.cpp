//
// Created by daniel on 4/23/18.
//

#include "QTrees.hpp"

QTrees::QTrees() {
    qTree = NULL;
}

QTrees::~QTrees() {
    delete qTree;
}

QTrees::QTrees(Q qInit) {
    qTree = new Tree(qInit);
}

void QTrees::add(Q qNew, Node *nParent) {
    qTree->add(qNew, nParent);
}

Node *QTrees::nearestNeighbor(Q qRand) {
    Q conf;
    Node* minNode;
    double dMinDist = 10000;
    double length;
    BOOST_FOREACH(Node* node, qTree->getNodes())
    {
        length = (node->getValue()-qRand).norm2();
        if(length < dMinDist){
            // nearer neighbore found update variables
            minNode = node;
            dMinDist = length;
        }
    }

    return minNode;
}

void QTrees::getRootPath(Node &lastNode, rw::trajectory::QPath &aPath) {
    // get the full path
    vector<Q> vPath;
    qTree->getRootPath(lastNode, vPath);

    // translate to QPath data type
    aPath.clear();
    for (unsigned int i = 0; i <vPath.size(); i++){
        aPath.push_back(vPath[i]);
    }


}



