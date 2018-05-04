//
// Created by daniel on 4/23/18.
//

#include "QTrees.hpp"

QTrees::QTrees() {
    qTree.clear();
}

QTrees::~QTrees() {
    for (unsigned int i = 0; i< qTree.size(); i++){
        delete qTree[i];
    }

}

QTrees::QTrees(Q qInit) {
    Node* rootNode = new Node(qInit, nullptr, 0);

    cout << "root node:\n";
    cout << "in the tree root node is:" << rootNode->q << endl;
    qTree.clear();
    qTree.push_back(rootNode);

   
}

void QTrees::add(Q qNew, Node *nParent) {
    Node* newNode = new Node(qNew, nParent, nParent->nodeCost + (qNew-nParent->q).norm2());
    qTree.push_back(newNode);
}

Node* QTrees::nearestNeighbor(Q qRand) {
    // setting up initial variables
    Q conf;
    Node* minNode = qTree[0];
    double dMinDist = (qTree[0]->q-qRand).norm2(), length;

    // finde the closest neighbor
    for(unsigned int i = 1; i < qTree.size(); i++){
        length = (qTree[i]->q-qRand).norm2();
        if(length < dMinDist){
            // nearer neighbore found update variables
            minNode = qTree[i];
            dMinDist = length;
        }
    }

    return minNode;
}
/*
Node * QTrees::nearestNeighbor(Q qRand,  unsigned int K) {
    // setting up initial variables
    Q conf;
    vector<Node*> minNode = qTree[1];
    double dMinDist = (qTree[0]->q-qRand).norm2(), length;

    // finde the closest neighbor
    for(unsigned int i = 1; i < qTree.size(); i++){
        length = (qTree[i]->q-qRand).norm2();
        if(length < dMinDist){
            // nearer neighbore found update variables
            minNode = qTree[i];
            dMinDist = length;
        }
    }

    return minNode;
}*/

void QTrees::getRootPath(Node* lastNode, rw::trajectory::QPath &aPath) {
    // setting up the node
    aPath.clear();
    Node * currentNode = lastNode;

    // track down to the root
    while (currentNode->parent != nullptr){
        aPath.push_back(currentNode->q);
        currentNode= currentNode->parent;
    }
    aPath.push_back(currentNode->q);

}



