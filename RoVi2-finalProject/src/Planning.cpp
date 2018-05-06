//
// Created by daniel on 4/17/18.
//

#include "Planning.hpp"





Planning::Planning() {
    _workcell = NULL;
    C =  VelocityScrew6D<>(0,0,1,1,1,0);

}

Planning::Planning(WorkCell::Ptr _workcell) {
    this->_workcell = _workcell;
    C =  VelocityScrew6D<>(0,0,1,1,1,1);

    device = this->_workcell->findDevice("UR1");
    gripper = this->_workcell->findDevice("WSG50");

    // initialize random function
    srand (time(NULL));

    // set up collision detector
    detector = new rw::proximity::CollisionDetector(_workcell, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());

    // construct a QSampler
    qSamples = rw::pathplanning::QSampler::makeUniform(device);
}

Planning::~Planning() {


}


rw::trajectory::QPath Planning::getConstraintPath(State _state, Q qGoal, Q qRobot, double eps) {
    // setup variables
    this->_state = _state;
    rw::trajectory::QPath path;
    VelocityScrew6D<> dx = computeDisplacement(qGoal);
    QTrees T = QTrees(qGoal, dx[0], dx[1]);
    Q dMax = Q(6,0.003,0.003,0.003,0.003,0.003,0.003);

    // initial conditions
    if(!RGDNewConfig(qRobot, dMax, 500,500,RGD_MIN_ERROR))
        return path;

    if(!RGDNewConfig(qGoal, dMax, 500,500,RGD_MIN_ERROR))
        return path;

    cout << "inside the loop\n";
    // Grow RRT tree
    unsigned int N = 0;
    for(N = 0; N <= MAX_RRT_ITERATIONS; N++){


        //cout << "N: " << N;
        Q qRand = sampler(qRobot, GOAL_SAMPLE_PROB);

        Node* nearestNode= T.nearestNeighbor(qRand);

        Q qNear = nearestNode->q;

        Q qDir = (qRand-qNear)/((qRand-qNear).norm2());

        Q qS = qNear + qDir*eps;

        // constrain the point
        if(RGDNewConfig(qS, dMax, 500,500,RGD_MIN_ERROR)){

            //cout << "qS " << qS << endl;

            nearestNode = T.nearestNeighbor(qS);

            qNear = nearestNode->q;

            qDir = (qS-qNear)/((qS-qNear).norm2())*eps;

            if (qDir.norm2() < (qS-qNear).norm2()){
                qS = qNear + qDir;
            }


            // check for edge colliitons
            if(expandedBinarySearch(qS, qNear, EDGE_CHECK_EBS)){
                dx = computeDisplacement(qS);
                T.add(qS, nearestNode, dx[0], dx[1]);

                nearestNode = T.nearestNeighbor(qRobot);
                cout <<(nearestNode->q - qRobot).norm2()<< endl;


                // has the goal been reached
                if((qS - qRobot).norm2() < 0.01){
                    // goal is close end loop
                    cout <<  "Goal reached in interations N: " << N << endl;
                    break;
                }
            }
        }
    }

    cout << "checking the max iterations" << endl;
    // post path planning check
    if(N >= MAX_RRT_ITERATIONS){
        cout << "No solution found \n";
        return path;
    }


    // print out the tree in the map

    Lego* _LegoHandle = new Lego(&_state, _workcell);
    cout << "printing" << endl;

    vector< vector< double> > v = _LegoHandle->getPoses();
    cout << endl;
    T.exportTree("Tree", v);
    cout << "make the print\n";
    delete _LegoHandle;
    cout << "deleting handler"<< endl;


    // fetch the path
    Node* nearestNode= T.nearestNeighbor(qRobot);
    T.getRootPath(nearestNode, path);


    // update the newest Tree for later use
    //T.setC(nearestNode->nodeCost);
    //R = T;

    // add the goal to the path
    //path.push_back(qRobot);

    return path;
}


QPath Planning::RRTC(State astate, Q qRobot, Q qGoal, double epsilon) {
    _state = astate;

    rw::math::Math::seed(time(NULL));

    //CollisionDetector detector(_workcell, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(detector,device,_state);
    //QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, qSamples, metric, epsilon, RRTPlanner::RRTConnect);


    if (inCollision(qGoal))
        return 0;
    if (inCollision(qRobot))
        return 0;

    cout << "Planning from " << qRobot << " to \n" << qGoal << endl;
    QPath path;
    planner->query(qRobot,qGoal,path,30);
    return path;
}


// ******************************************************************
//
// Private functions
//
// ******************************************************************


VelocityScrew6D<> Planning::computeTaskError(Q qSample) {

    Frame* TaskFrame = _workcell->findFrame("TaskFrame");


    Transform3D<> wTt = TaskFrame->wTf(_state);
    Transform3D<> tTw = inverse(wTt);


    device->setQ(qSample, _state);
    Frame* EndEffector = _workcell->findFrame("EndEff");
    Transform3D<> wTe = EndEffector->wTf(_state);
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

    Transform3D<> wTt = TaskFrame->wTf(_state);
    Transform3D<> tTw = inverse(wTt);

    device->setQ(qSample, _state);
    Frame* EndEffector = _workcell->findFrame("EndEff");
    Transform3D<> wTe = EndEffector->wTf(_state);
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
        rw::proximity::CollisionDetector::QueryResult data;
        device->setQ(qs, _state);

        bool collision = detector->inCollision(_state, &data);
        if(collision)
        {
            //cout << "in collision" << endl;
            return false;
        }

        return true;
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
    rw::proximity::CollisionDetector::QueryResult data;
    device->setQ(q, _state);
    bool collision = detector->inCollision(_state, &data);
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

    // check for collision in the end
    device->setQ(EndConf, _state);
    if(detector->inCollision(_state, &data)){
        //cout << "EndConf collision status: " <<  detector->inCollision(state, &data) << endl;
        return false;
    }


    // Perform algorithm
    for(unsigned int i = 1; i <= levels; i++ ){
        steps = (int)pow(2,i-1);
        step = dqNew/steps;


        for(unsigned int j = 1; j<=steps; j++){
            qi = StartConf+(j - 0.5)*step;

            device->setQ(qi, _state);

            if((qi-StartConf).norm2() < dq.norm2()){
                if (detector->inCollision(_state, &data)){
                    // collision detected
                    return false;
                }
            }

        }

    }
    return true;
}




