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

QTrees::QTrees(Q qInit, double aX, double aY) {
    Node* rootNode = new Node(qInit, nullptr, 0, aX, aY);

    qTree.clear();
    qTree.push_back(rootNode);
    C = 10000;
    db = 1;
    cb = 0;
}

void QTrees::add(Q qNew, Node *nParent, double aX, double aY) {
    Node* newNode = new Node(qNew, nParent, nParent->nodeCost + (qNew-nParent->q).norm2(), aX, aY);
    qTree.push_back(newNode);
}


Node* QTrees::nearestNeighbor(Q qRand) {
    // setting up initial variables
    Q conf;
    Node* minNode = qTree[0];
    double dMinDist = db*(qTree[0]->q-qRand).norm2() + cb*qTree[0]->nodeCost, length;

    // Find the closest neighbor
    for(unsigned int i = 1; i < qTree.size(); i++){
        length = db*(qTree[i]->q-qRand).norm2() + cb*qTree[i]->nodeCost;
        if(length < dMinDist){
            // nearer neighbore found update variables
            minNode = qTree[i];
            dMinDist = length;
        }
    }

    if (dMinDist < C){
        return minNode;
    }
    return nullptr;

}

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

void QTrees::setC( double C) {
    this->C = C;
}

// todo add the legobrixks
void QTrees::exportTree(string fileName, vector<vector<double> > vBricks) {
    // setup defines
    double PIXEL_MM = 700;

    // create an image for all the nodes
    cout << "stuff \n";

    cv::Mat res(1000,1000, CV_8UC3, cv::Scalar(255,255,255));

    // add all the nodes and and edges to the image
    Node* parent;



    // create a rectangle for the conveuor belt
    cv::Point p1= cv::Point(500-X_RANGE*PIXEL_MM,500-Y_RANGE*PIXEL_MM);
    cv::Point p2= cv::Point(500+X_RANGE*PIXEL_MM,500+Y_RANGE*PIXEL_MM);
    cv::rectangle( res, p1, p2, cv::Scalar( 0, 0, 0 ), 3 , 8 );

    p1= cv::Point(5,6);
    p2= cv::Point(10,11);
    cv::rectangle( res, p1, p2, cv::Scalar( 255, 0, 0 ), 3 , 8 );

    cout << "get rect\n ";

    // draw the tree
    for (unsigned int i = 0; i<qTree.size(); i++){
        parent = qTree[i];
        int xval = (int)(parent->x*PIXEL_MM) + 500;
        int yval = (int)(parent->y*PIXEL_MM) + 500;

        cout << "xval: " << xval << " "<< (parent->x*PIXEL_MM) << " yval; " << yval << " " << (parent->y*PIXEL_MM)<< endl;
        if (xval > 0 && xval < 999 && yval > 0 && yval < 999 ){
            res.at<cv::Vec3b>(xval, yval) = cv::Vec3b::all(0);
        }
    }

    for(unsigned int i = 0; i<vBricks.size(); i++){

        cout << i << ": ";
        for(unsigned int j = 0; j<vBricks[i].size(); j++){
            cout << vBricks[i][j] << " ";
        }
        cout << endl;

    }

    // save the image to a file
    cv::imshow(fileName, res);
}



