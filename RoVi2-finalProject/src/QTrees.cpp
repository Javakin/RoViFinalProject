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
    cv::Mat res(700,1000, CV_8UC3, cv::Scalar(255,255,255));


    // create a rectangle for the conveuor belt
    cv::Point p1= cv::Point((unsigned int)(res.cols/2-X_CONVEYOR*PIXEL_MM),(unsigned int)(res.rows/2-Y_CONVEYOR*PIXEL_MM));
    cv::Point p2= cv::Point((unsigned int)(res.cols/2+X_CONVEYOR*PIXEL_MM),(unsigned int)(res.rows/2+Y_CONVEYOR*PIXEL_MM));
    cv::rectangle( res, p1, p2, cv::Scalar( 0, 0, 0 ), 2 , 8 );


    // Draw the legoBricks
    for(unsigned int i = 0; i<vBricks.size(); i++) {
        cv::RotatedRect rRect = cv::RotatedRect(
                cv::Point2f(vBricks[i][0] * PIXEL_MM + res.cols / 2, vBricks[i][1] * PIXEL_MM + res.rows / 2),
                cv::Size2f(0.016 * PIXEL_MM, 0.048 * PIXEL_MM), vBricks[i][3] * 180 / 3.1415);
        cv::Point2f vertices[4];
        rRect.points(vertices);
        vector<cv::Point> point;

        for (int j = 0; j < 4; j++) {
            point.push_back(cv::Point((int)vertices[j].x, (int)vertices[j].y));  //point1
        }

        cv::fillConvexPoly(res, point, cv::Scalar(0, 200, 255), 8, 0);
    }

    // Draw the Q Tree
    Node* parent = qTree[0];
    for (unsigned int i = 0; i<qTree.size(); i++){
        parent = qTree[i];
        int xval = (unsigned int)(parent->x*PIXEL_MM + res.cols/2);
        int yval = (unsigned int)(parent->y*PIXEL_MM + res.rows/2);

        //cout << "xval: " << xval << " "<< (parent->x*PIXEL_MM) << " yval; " << yval << " " << (parent->y*PIXEL_MM)<< endl;
        if (xval > 0 && xval < 999 && yval > 0 && yval < 999 ){
            res.at<cv::Vec3b>(cv::Point(xval, yval)) = cv::Vec3b::all(0);
        }
    }

    // Highlight the goalConfiguration
    parent = qTree[0];
    p1= cv::Point( (unsigned int)(parent->x*PIXEL_MM + res.cols/2),(unsigned int)(parent->y*PIXEL_MM + res.rows/2));
    cv::circle(res, p1, 4, cv::Scalar(255,0,0), 3);

    // Highlight the robotConfiguration
    parent = qTree[qTree.size()-1];
    p1= cv::Point( (unsigned int)(parent->x*PIXEL_MM + res.cols/2),(unsigned int)(parent->y*PIXEL_MM + res.rows/2));
    cv::circle(res, p1, 4, cv::Scalar(0,0,255), 3);

    // save the image to a file
    cv::imshow(fileName, res);
}



