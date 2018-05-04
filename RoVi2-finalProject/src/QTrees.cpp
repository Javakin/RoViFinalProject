//
// Created by daniel on 4/23/18.
//

#include "QTrees.hpp"

QTrees::QTrees() {
    qTree = nullptr;
}

QTrees::~QTrees() {
    for (unsigned int i = 0; i<qTree->size(); i++){
        delete qTree[i];
    }

    delete qTree;
}

QTrees::QTrees(Q qInit) {
    auto* rootNode = new Node(qInit, nullptr, 0);

    qTree = new vector<Node*>;
    qTree->push_back(rootNode);
}

void QTrees::add(Q qNew, Node *nParent) {
    auto* newNode = new Node(qNew, nParent, nParent->nodeCost + (qNew-nParent->q).norm2());
    qTree->push_back(newNode);
}

Node *QTrees::nearestNeighbor(Q qRand) {
    Q conf;
    Node* minNode = NULL;
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

vector<Node *> QTrees::kNearestNeighbor(Q qRand, unsigned int K) {
    Q conf;
    vector<Node*> minNodes = ;
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



