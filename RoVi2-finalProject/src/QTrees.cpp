//
// Created by daniel on 4/23/18.
//

#include "QTrees.hpp"

QTrees::QTrees() {
    qVertex.clear();
    qTree.clear();


}

QTrees::~QTrees() {

}

QTrees::QTrees(Q qInit) {
    addVertex(qInit);

}

unsigned int QTrees::addVertex(Q vertex) {
    qVertex.push_back(vertex);

    // return the ID of the vertex
    return (unsigned int)(qVertex.size()-1);

}

void QTrees::addEdge(unsigned int p1, unsigned int p2) {
    qTree.push_back(pair<unsigned int, unsigned int>(p1,p2));
}



void QTrees::clear() {
    qVertex.clear();
    qTree.clear();

}

int QTrees::nearestVertex(Q qNew) {
    // setup variables
    int VertexID = -1;

    // initial conditions
    if (qTree.size() == 0){
        return VertexID;
    }

    // Search the tree for neares neightboor
    double dMinDist = MAX_DISTANCE;
    for(unsigned int i = 0; i<qVertex.size(); i++){
        if((qVertex[i]-qNew).norm2() < dMinDist){
            // nearer neighbore found update variables
            VertexID = i;
            dMinDist = (qVertex[i]-qNew).norm2();
        }
    }

    return VertexID;
}

Q QTrees::getVertex(unsigned int uiVertex) {
    //initial conditions
    if(uiVertex > (qVertex.size()-1)){
        cout << "GetVertex index out of bounce" << endl;
        return Q();
    }

    return qVertex[uiVertex];
}
