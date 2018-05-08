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
    setCb(cb);
}

QTrees::QTrees(Q qInit, double aX, double aY, double C_space, double cost_b) {
    Node* rootNode = new Node(qInit, nullptr, 0, aX, aY);

    qTree.clear();
    qTree.push_back(rootNode);
    C = C_space;
    setCb(cost_b);
}

void QTrees::add(Q qNew, Node *nParent, double aX, double aY) {
    Node* newNode = new Node(qNew, nParent, nParent->nodeCost + (qNew-nParent->q).norm2(), aX, aY);
    qTree.push_back(newNode);
    //cout << C << " " << newNode->nodeCost << endl;
}


Node* QTrees::nearestNeighbor(Q qRand, int constrained) {
    Node* nearestNode;

    if(constrained <= 0){
        nearestNode = nearestNeighbor(qRand);
    } else{
        nearestNode = constrainedNearestNeighbor(qRand, constrained);
    }

    return nearestNode;
}

Node* QTrees::nearestNeighbor(Q qRand) {
    // setting up initial variables
    Q conf;
    Node* minNode = qTree[0];
    double dMinDist = (qTree[0]->q-qRand).norm2(), length;

    // Find the closest neighbor
    for(unsigned int i = 1; i < qTree.size(); i++){
        length = (qTree[i]->q-qRand).norm2();
        if(length < dMinDist){
            // nearer neighbore found update variables
            minNode = qTree[i];
            dMinDist = length;
        }
    }
    //cout << "yay"<< endl;
    return minNode;
}

Node* QTrees::constrainedNearestNeighbor(Q qRand, int constrained){
    // setting up initial variables
    //cout << "hehe" << constrained<< endl;
    Q conf;
    vector<Node*> minNodes((unsigned int)constrained, qTree[0]);
    double dMinDist = db*(qTree[0]->q-qRand).norm2() + cb*qTree[0]->nodeCost, length, length2;



    // Find the closest k - neighbors
    double maxNodeDist = (qTree[0]->q-qRand).norm2();
    int maxNodeID = 0;
    for(unsigned int i = 1; i < qTree.size(); i++){
        length = (qTree[i]->q-qRand).norm2();

        // add new nearest
        if(length < maxNodeDist) {
            minNodes[maxNodeID] = qTree[i];

            //cout << length << endl;
            // finde new maxNodeDist
            maxNodeDist = 0;
            for (unsigned int j = 0; j < minNodes.size(); j++) {
                length2 = (minNodes[j]->q - qRand).norm2();
                if (length2 > maxNodeDist) {
                    maxNodeDist = length2;
                    maxNodeID = j;
                }
            }
        }
    }

    //cout << "Done "<< endl;

    // select the best candidat
    Node* minNode = minNodes[0];
    dMinDist = db*(minNodes[0]->q-qRand).norm2() + cb*minNodes[0]->nodeCost;

    for(unsigned int i = 0; i < (unsigned int)constrained; i++){
        length = db*(minNodes[i]->q-qRand).norm2() + cb*minNodes[i]->nodeCost;
        //cout << length << endl;
        if(length < dMinDist){
            // nearer neighbore found update variables
            minNode = minNodes[i];
            dMinDist = length;
        }
    }
    //cout << "completed KNN" << endl;

    if (minNode->nodeCost < C){
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

void QTrees::setC( double aC) {
    C = aC;
}


void QTrees::exportTree(string fileName, vector<vector<double> > vBricks) {
    // setup defines
    double PIXEL_MM = 700;

    // create an image for all the nodes
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
        if (xval > 0 && xval < res.cols-1 && yval > 0 && yval < res.rows-1 ){
            // Draw point
            cv::circle(res, cv::Point(xval, yval), 2, cv::Scalar(0, 0, 0), -1);
            //res.at<cv::Vec3b>(cv::Point(xval, yval)) = cv::Vec3b::all(0);


            if (parent->parent != nullptr) {
                int xval2 = (unsigned int) (parent->parent->x * PIXEL_MM + res.cols / 2);
                int yval2 = (unsigned int) (parent->parent->y * PIXEL_MM + res.rows / 2);

                if (xval2 > 0 && xval2 < res.cols - 1 && yval2 > 0 && yval2 < res.rows - 1) {
                    // Draw line
                    cv::line(res, cv::Point(xval, yval), cv::Point(xval2, yval2), cv::Scalar(0, 0, 0));
                }
            }
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

Node *QTrees::getRootNode() {
    return qTree[0];
}

void QTrees::setCb(double aCb) {
    if(aCb < 0){
        aCb = 0;
    }
    if(aCb > 1){
        aCb = 1;
    }

    cb = aCb;
    db = (1-cb);
    cout << "cb " << cb << " db " << db << endl;
}

double QTrees::getC() {
    return C;
}

double QTrees::getCb() {
    return cb;
}



