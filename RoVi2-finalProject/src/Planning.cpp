//
// Created by daniel on 4/17/18.
//

#include "Planning.hpp"





Planning::Planning() {
    _workcell = NULL;
    C =  VelocityScrew6D<>(0,0,1,1,1,0);

}

Planning::Planning(WorkCell::Ptr _workcell, rw::kinematics::State::Ptr  _state,  Robot* _RobotHandle) {
    this->_workcell = _workcell;
    this->_state = _state;
    C =  VelocityScrew6D<>(0,0,1,1,1,1);

    device = this->_workcell->findDevice("UR1");
    gripper = this->_workcell->findDevice("WSG50");

    // initialize random function
    srand (time(NULL));

    // set up collision detector
    detector = new rw::proximity::CollisionDetector(this->_workcell, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());

    // construct a QSampler
    qSamples = rw::pathplanning::QSampler::makeUniform(device);

    // setup robothandleren
    this->_RobotHandle = _RobotHandle;

    // initialize the mazerunner
    robotDirection = 0;
    pausePlan = 1;

    // initialize trees
    _R = nullptr;
    InUse = 0;
}

Planning::~Planning() {
    // kill the thread
    isAlive = false;

    if (_R != nullptr){
        delete _R;
    }

}

bool Planning::constrainedRRT(QTrees* _T, Q qGoal, double eps, int numOfNearestNodes, double goalSampleProb){
    state = *_state;
    unsigned int N = 0;
    VelocityScrew6D<> dx;
    Q dMax = Q(6,0.003,0.003,0.003,0.003,0.003,0.003);


    // initial conditions
    Q qRobot = _RobotHandle->getQRobot();
    Q qGol = qGoal;
    if(RGDNewConfig(qRobot, dMax, 500,500,RGD_MIN_ERROR) && RGDNewConfig(qGol, dMax, 500,500,RGD_MIN_ERROR)){
        //cout << "inside the loop\n";
        // Grow RRT tree
        Q qS;
        for(N = 0; N <= MAX_RRT_ITERATIONS; N++){

            //cout << "N: " << N;
            qRobot = _RobotHandle->getQRobot();
            Q qRand = sampler(qRobot, goalSampleProb);

            Node* nearestNode= _T->nearestNeighbor(qRand, 0);

            Q qNear = nearestNode->q;

            Q qDir = (qRand-qNear)/((qRand-qNear).norm2());

            // can the sampled point be reached
            if ((qRand-qNear).norm2() < eps){
                qS = qRand;

                // Has the goal been reached
                if((qS - qRobot).norm2() < 0.01){
                    dx = computeDisplacement(qS);
                    if(_T->add(qS, nearestNode, dx[0], dx[1])){
                        // goal is close end loop
                        //cout <<  "Goal reached in interations N: " << N << endl;
                        break;
                    }
                }
            }else{
                qS = qNear + qDir*eps;
            }


            // constrain the point
            if(RGDNewConfig(qS, dMax, 500,500,RGD_MIN_ERROR)){

                //cout << "qS " << qS << endl;

                nearestNode = _T->nearestNeighbor(qS, numOfNearestNodes);
                if(nearestNode != nullptr){
                    qNear = nearestNode->q;

                    qDir = (qS-qNear)/((qS-qNear).norm2())*eps;

                    if (qDir.norm2() < (qS-qNear).norm2()){
                        qS = qNear + qDir;
                    }


                    // check for edge colliitons
                    if(expandedBinarySearch(qS, qNear, EDGE_CHECK_EBS)){
                        dx = computeDisplacement(qS);
                        _T->add(qS, nearestNode, dx[0], dx[1]);


                    }
                }
            }
        }
    }else {
        cout << "Robot or goals is in collition\n";
        return false;
    }

    //cout << "ended \n";
    return N <= MAX_RRT_ITERATIONS;
}

rw::trajectory::QPath Planning::getConstraintPath(Q qGoal, Q qRobot, double eps,  double goalSampleProb) {
    // setup variables
    rw::trajectory::QPath path;
    //cout << "goocs" << endl;
    VelocityScrew6D<> dx = computeDisplacement(qGoal);
    QTrees* _T = new QTrees(qGoal, dx[0], dx[1]);
    Q dMax = Q(6,0.003,0.003,0.003,0.003,0.003,0.003);
    state = *_state;

    // Grow initial constrained RRT
    bool status = constrainedRRT(_T, qGoal, eps, 0, goalSampleProb);

    // post path planning check
    if(status){
        //cout << "Solution found \n";
        printTree(_T, state);

        // Fetch the path
        Node* nearestNode= _T->nearestNeighbor(qRobot, 0);
        _T->getRootPath(nearestNode, path);

        cout << "Returning path of length: " << path.size() << " and length " << nearestNode->nodeCost << endl;

        // Update tree structure
        if (_R != nullptr)
            delete _R;
        _R = _T;


    }else{
        cout << "No soluiton found\n";
        // update the newest Tree for later use
        if(_T != nullptr)
            delete _T;
    }


    return path;
}


QPath Planning::RRTC(State astate, Q qRobot, Q qGoal, double epsilon) {
    state = astate;

    rw::math::Math::seed(time(NULL));

    //CollisionDetector detector(_workcell, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(detector,device,state);
    //QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, qSamples, metric, epsilon, RRTPlanner::RRTConnect);


    if (inCollision(qGoal))
        return 0;

    if (inCollision(qRobot))
        return 0;

    //cout << "Planning from " << qRobot << " to \n" << qGoal << endl;
    QPath path;
    planner->query(qRobot,qGoal,path,30);
    return path;
}

void Planning::printTree(QTrees* _tree, rw::kinematics::State aState){
    // print out the tree in the map
    if(_tree != nullptr){
        Lego* _LegoHandle = new Lego(&aState, _workcell);

        vector<vector< double> > v = _LegoHandle->getPoses();
        _tree->exportTree("Tree", v);
        delete _LegoHandle;
    }

}

void Planning::printTree(){
    // print out the tree in the map
    printTree(_R, *_state);
}



void Planning::pausePlanner(int aStatus){
    pausePlan = aStatus;
}

void Planning::run(){
    // uptimize on current tree if active
    isAlive= true;

    Q q1 = Q(6,0.584,-1.067, -2.170, -1.477, 1.571, 1.805);
    Q q2 = Q(6,0.446, -2.076, -1.155, -1.479, 1.568, 1.664);
    Q qGoal;

    //int ite = 0;
    //double epsilon = 0.01;


    cout << "lets get to it \n";
    while (isAlive){
        // Setup


        if(pausePlan == 0){
            // If the route is complete make a new one
            if (_RobotHandle->pathCompleted()){

                //cout << "hello world " << robotDirection << endl;
                //cout << "in loop\n";
                if(robotDirection == 0){
                    qGoal = q2;
                }
                else if(robotDirection == 1){
                    qGoal = q1;
                }

                QPath aPath = getConstraintPath(qGoal, _RobotHandle->getQRobot(), RRT_EPSILON, GOAL_SAMPLE_PROB);


                if (aPath.size() != 0){
                    robotDirection = !robotDirection;
                    _RobotHandle->setPath(aPath);
                }

            }else{
                // check for errors in the tree

                QPath robotpath;
                robotpath = validate(VALIDAITON_DEPTH);

                cout << robotpath.size() << endl;


                if(robotpath.size() != 0){
                    // Collision detected
                    cout << "Collision detected\n";
                    printTree();
                    _RobotHandle->setPath(robotpath);

                    // Repare tree
                    robotpath = repareTree();
                    while(robotpath.size() == 0){
                        robotpath = repareTree();
                    }
                    _RobotHandle->setPath(robotpath);
                }

                // Search for a better solution
                if(_R!= nullptr){
                    cout << "stuff" << endl;
                    QPath aPath = updateConstraindPath(qGoal, RRT_EPSILON);
                    //cout << "done updating " << aPath.size() << endl;
                    if (aPath.size() != 0){

                        _RobotHandle->setPath(aPath);
                    }
                }

            }

        }
        //myLegoPointer->move(0.01);
        // sleep for a duration of time
        usleep( 50000 );
    }

    cout << "i am dying\n";

}
void Planning::setLegoHandle(Lego* alego){
    myLegoPointer = alego;
}

QPath Planning::validate(double CheckingDebth){
    // setup
    state = *_state;
    QPath path;
    vector< Node* > nPath;

    if(_R != nullptr){
        Node* currentNode = _R->nearestNeighbor(_RobotHandle->getQRobot(), 0);
        double initialCost = currentNode->nodeCost;
        int collisionDetected = 0;

        // Go through the final nodes
        while(currentNode->parent != nullptr && initialCost - currentNode->nodeCost < CheckingDebth) {
            if(inCollision(&state, currentNode->q)){
                collisionDetected = 1;
                //cout << "test2\n";
                break;
            }
            if(!expandedBinarySearch(&state, currentNode->q, currentNode->parent->q, EDGE_CHECK_EBS )){
                collisionDetected = 1;
                //cout << "test3\n";
                break;
            }

            //cout << (initialCost - currentNode->nodeCost)<< endl;

            nPath.push_back(currentNode);

            currentNode = currentNode->parent;
        }

        //cout << collisionDetected  << " " <<  nPath.size() << endl;
        // maintain a safe distance to the collition
        if(collisionDetected){
            if(nPath.size() != 0){
                double initial_distance = nPath[nPath.size()-1]->nodeCost;
                cout << (nPath[nPath.size()-1]->nodeCost-initial_distance) << "this is off" << endl;
                while(nPath.size() > 0 && nPath[nPath.size()-1]->nodeCost-initial_distance < VALIDATION_BOUND){
                    cout << (nPath[nPath.size()-1]->nodeCost-initial_distance) << endl;
                    nPath.pop_back();
                    cout << "popping\n";
                }
            }
        }

        //cout << collisionDetected  << " " <<  nPath.size() << " test2" << endl;
        // create the new path
        if(collisionDetected){
            if(nPath.size() > 1){
                // is the closest point behinde it in the path
                if((_RobotHandle->getQRobot() - nPath[1]->q).norm2() > ( nPath[0]->q -  nPath[1]->q).norm2()){
                    path.push_back(nPath[0]->q);
                }
                // add the remaining points that within the clearance
                for(unsigned int i = 1; i < nPath.size(); i++){
                    path.push_back(nPath[i]->q);
                }
            }

        }

        if(collisionDetected && path.size() == 0){
            path.push_back(_RobotHandle->getQRobot());
        }
        //cout << collisionDetected  << " " <<  nPath.size() << " test1" << endl;
    }



    return path;
}


// todo make a repare algorithm that uthilizes the previus tree
QPath Planning::repareTree(){

    rw::trajectory::QPath path;
    state = *_state;
    VelocityScrew6D<> dx = computeDisplacement(_R->getRootNode()->q);
    QTrees* _T = new QTrees(_R->getRootNode()->q, dx[0], dx[1]);
    bool sucess;
    if(_R != nullptr) {
        sucess = constrainedRRT(_T, _T->getRootNode()->q, RRT_EPSILON, 0, GOAL_SAMPLE_PROB);


        // post path planning check
        // Fetch the path
        Q qRobot = _RobotHandle->getQRobot();
        Node *nearestNodeT = _T->nearestNeighbor(qRobot, 0);

        printTree(_T, *_state);

        if (sucess) {
            _T->getRootPath(nearestNodeT, path);
            cout << "Returning path of length: " << path.size() << " and length " << nearestNodeT->nodeCost
                 << " with cost " << _T->getC() << endl;

            // Update tree structure
            if (_R != nullptr)
                delete _R;
            _R = _T;

            printTree();


        } else {
            cout << "No soluiton found\n";
            // update the newest Tree for later use
            if (_T != nullptr)
                delete _T;
        }
    }
    return path;


/*
    //remove invalid nodes
    if(_R != nullptr){
        VelocityScrew6D<> dx = computeDisplacement(_R->getRootNode()->q);
        QTrees* _T = new QTrees(_R->getRootNode()->q, dx[0], dx[1]);

        cout << "fejl i treet\n";
        printTree();
        int i = 1;
        Node* iNode = _R->getNode(i);
        while(iNode != nullptr){
            // check if it still have a parrent
            if(iNode->parent == nullptr){
                delete iNode;
                cout << "removed node " << i << endl;
            }else {
                // check for collision
                if(expandedBinarySearch(iNode->q, iNode->parent->q, EDGE_CHECK_EBS)){
                    dx = computeDisplacement(iNode->q);
                    _T->add(iNode->q, iNode->parent, dx[0], dx[1]);
                }else{
                    delete iNode;
                }
            }

            // iterate through tree
            i++;
            iNode = _R->getNode(i);
        }
        // Update tree structure
        if(_R != nullptr)
            delete _R;
        _R = _T;

        cout << "så er treet renset" << endl;
        printTree(_T, state);

        // Grow a new tree
        bool sucess = constrainedRRT(_T, _T->getRootNode()->q, RRT_EPSILON, 0);



        // post path planning check
        rw::trajectory::QPath path;
        // Fetch the path
        Q qRobot = _RobotHandle->getQRobot();
        Node* nearestNodeT = _T->nearestNeighbor(qRobot,0);

        if (sucess) {
            _T->getRootPath(nearestNodeT, path);
            cout << "Returning path of length: " << path.size() << " and length " << nearestNodeT->nodeCost << " with cost " << _T->getC() << endl;

            // Update tree structure
            if(_R != nullptr)
                delete _R;
            _R = _T;

            printTree();


        } else {
            cout << "No soluiton found\n";
            // update the newest Tree for later use
            if(_T != nullptr)
                delete _T;
        }
        return path;
    }

*/
    //return outputPath;
}

QPath Planning::updateConstraindPath(Q qGoal, double eps) {

    //cout << "begining to update " << endl;

    // setup variables
    VelocityScrew6D<> dx = computeDisplacement(qGoal);
    double C_space = _R->nearestNeighbor(_RobotHandle->getQRobot(),0)->nodeCost;
    Q dMax = Q(6, 0.003, 0.003, 0.003, 0.003, 0.003, 0.003);

    // setup tree with new parameters
    QTrees *_T = new QTrees(qGoal, dx[0], dx[1], C_space*(1 - IMPROVEMENT_FACTOR), _R->getCb() + IMPROVEMENT_FACTORCB);
    //cout << "qGoal:  " << qGoal << endl << "qRobot: " << endl;

    // Grow a new tree
    bool sucess = constrainedRRT(_T, qGoal, eps, NUM_OF_NEIGHBORS, GOAL_SAMPLE_PROB);



    // post path planning check
    rw::trajectory::QPath path;
    // Fetch the path
    Q qRobot = _RobotHandle->getQRobot();
    Node *nearestNodeT = _T->nearestNeighbor(qRobot,0);
    Node *nearestNodeR = _R->nearestNeighbor(qRobot,0);

    if (sucess && nearestNodeT->nodeCost*(1-IMPROVEMENT_FACTOR) < nearestNodeR->nodeCost) {


        _T->getRootPath(nearestNodeT, path);


        cout << "Returning path of length: " << path.size() << " and length " << nearestNodeT->nodeCost << " with cost " << nearestNodeT->nodeCost << endl;

        // Update tree structure
        if(_R != nullptr)
            delete _R;
        _R = _T;

        printTree();


    } else {
        cout << "No soluiton found\n";
        // update the newest Tree for later use
        if(_T != nullptr)
            delete _T;
    }


    return path;
}

// ******************************************************************
//
// Private functions
//
// ******************************************************************


VelocityScrew6D<> Planning::computeTaskError(Q qSample) {

    Frame* TaskFrame = _workcell->findFrame("TaskFrame");


    Transform3D<> wTt = TaskFrame->wTf(state);
    Transform3D<> tTw = inverse(wTt);


    device->setQ(qSample, state);
    Frame* EndEffector = _workcell->findFrame("EndEff");
    Transform3D<> wTe = EndEffector->wTf(state);
    Transform3D<> eTt = tTw*wTe;



    //calculate the dx_error
    VelocityScrew6D<> dx_error =  VelocityScrew6D<>(0,0,0,0,0,0);
    for (unsigned int i = 0; i<3; i++){
        dx_error[i] = C[i]*eTt.P()[i];
        dx_error[3+i] = C[3+i]*(RPY<>(eTt.R()))[i];
    }

    return dx_error;
}

VelocityScrew6D<> Planning::computeDisplacement(Q qSample){
    Frame* TaskFrame = _workcell->findFrame("TaskFrame");

    Transform3D<> wTt = TaskFrame->wTf(state);
    Transform3D<> tTw = inverse(wTt);

    device->setQ(qSample, state);
    Frame* EndEffector = _workcell->findFrame("EndEff");
    Transform3D<> wTe = EndEffector->wTf(state);
    Transform3D<> eTt = tTw*wTe;


    //calculate the dx_error
    VelocityScrew6D<> dx =  VelocityScrew6D<>(0,0,0,0,0,0);
    for (unsigned int i = 0; i<3; i++){
        dx[i] = eTt.P()[i];
        dx[3+i] = (RPY<>(eTt.R()))[i];
    }

    return dx;
}


Q Planning::randomDisplacement(Q dMax) {

    Q qOut(6,0,0,0,0,0,0);
    for(unsigned int i = 0; i < dMax.size(); i++){
        qOut[i] = (((double)rand()/RAND_MAX)*2-1)*dMax[i];
    }

    return qOut;
}

bool Planning::RGDNewConfig(Q &qs, Q dMax, int MaxI, int MaxJ, double eps) {
    // setting up variables

    int i = 0; int j = 0;
    VelocityScrew6D<> dx_error = computeTaskError(qs);
    VelocityScrew6D<> dx_error_prime;
    Q qs_prime;

    // Constraint the configuration
    while (i < MaxI && j < MaxJ &&  dx_error.norm2() > eps){
        j++;

        qs_prime = qs + randomDisplacement(dMax);
        dx_error_prime = computeTaskError(qs_prime);

        //cout << j << ": " << dx_error << endl;

        if(dx_error_prime.norm2() < dx_error.norm2()){
            // a better guess was found
            i++;
            qs = qs_prime;
            dx_error = dx_error_prime;
            //cout << i  << "," << j << ": " << dx_error.norm2() << endl;
            j = 0;
        }

    }

    // check that the solution is good
    if(dx_error.norm2() <= eps){
        //cout << "status " << !inCollision(_state,qs) << endl;
        return !inCollision(_state,qs);
    }
    return false;
}

Q Planning::sampler(Q qGoal, double goalSampleProb) {
    Q outPut = qGoal;

    if (((double)rand()/RAND_MAX) > goalSampleProb){
        outPut = qSamples->sample();
    }

    return outPut;
}

bool Planning::inCollision(const Q &q) {
    rw::kinematics::State astate = *_state;
    rw::proximity::CollisionDetector::QueryResult data;
    device->setQ(q, astate);
    bool collision = detector->inCollision(astate, &data);
    if(collision)
    {
        return true;
    }

    return false;
}

bool Planning::inCollision(rw::kinematics::State::Ptr  _astate, const Q &q) {
    rw::kinematics::State astate = *_state;
    rw::proximity::CollisionDetector::QueryResult data;
    device->setQ(q, astate);
    bool collision = detector->inCollision(astate, &data);
    if(collision)
    {
        return true;
    }

    return false;
}


bool Planning::expandedBinarySearch(const Q StartConf, const Q EndConf, double eps){
    // Setup variables
    rw::proximity::CollisionDetector::QueryResult data;

    Q qi, step;
    Q dq = EndConf-StartConf;
    double n = (dq.norm2()/eps);
    unsigned int levels = ceil(log2(n));
    Q dqNew = pow(2, levels)*eps*dq/dq.norm2();
    unsigned int steps;

    // precondition
    if((StartConf-EndConf).norm2() < eps){
        return true;
    }

    // check for collision in the end
    device->setQ(EndConf, state);
    if(detector->inCollision(state, &data)){
        //cout << "EndConf collision status: " <<  detector->inCollision(state, &data) << endl;
        return false;
    }






    // Perform algorithm
    for(unsigned int i = 1; i <= levels; i++ ){
        steps = (int)pow(2,i-1);
        step = dqNew/steps;

        for(unsigned int j = 1; j<=steps; j++){
            qi = StartConf+(j - 0.5)*step;

            device->setQ(qi, state);

            if((qi-StartConf).norm2() < dq.norm2()){
                if (detector->inCollision(state, &data)){
                    // collision detected
                    return false;
                }
            }

        }

    }
    return true;
}

bool Planning::expandedBinarySearch(rw::kinematics::State::Ptr  _state, const Q StartConf, const Q EndConf, double eps){
    // Setup variables
    rw::proximity::CollisionDetector::QueryResult data;

    Q qi, step;
    Q dq = EndConf-StartConf;
    double n = (dq.norm2()/eps);
    unsigned int levels = ceil(log2(n));
    Q dqNew = pow(2, levels)*eps*dq/dq.norm2();
    unsigned int steps;

    // check for collision in the end
    device->setQ(EndConf, *_state);
    if(detector->inCollision(*_state, &data)){
        //cout << "EndConf collision status: " <<  detector->inCollision(state, &data) << endl;
        return false;
    }

    // precondition
    if((StartConf-EndConf).norm2() < eps){
        return true;
    }

    // Perform algorithm
    for(unsigned int i = 1; i <= levels; i++ ){
        steps = (int)pow(2,i-1);
        step = dqNew/steps;


        for(unsigned int j = 1; j<=steps; j++){
            qi = StartConf+(j - 0.5)*step;

            device->setQ(qi, *_state);

            if((qi-StartConf).norm2() < dq.norm2()){
                if (detector->inCollision(*_state, &data)){
                    // collision detected
                    return false;
                }
            }

        }

    }
    return true;
}






