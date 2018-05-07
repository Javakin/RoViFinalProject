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
    pausePlan = 0;

    // initialize trees
    _T = nullptr;
    _R = nullptr;
}

Planning::~Planning() {
    // kill the thread
    isAlive = false;

    if (_T != nullptr){
        delete _T;
    }

    if (_R != nullptr){
        delete _R;
    }

}


rw::trajectory::QPath Planning::getConstraintPath(State state, Q qGoal, Q qRobot, double eps) {
    // setup variables
    this->state = state;
    rw::trajectory::QPath path;
    VelocityScrew6D<> dx = computeDisplacement(qGoal);

    _T = new QTrees(qGoal, dx[0], dx[1]);
    Q dMax = Q(6,0.003,0.003,0.003,0.003,0.003,0.003);

    // initial conditions
    Q qRob = qRobot;
    Q qGol = qGoal;


    unsigned int N = 0;
    if(RGDNewConfig(qRob, dMax, 500,500,RGD_MIN_ERROR) && RGDNewConfig(qGol, dMax, 500,500,RGD_MIN_ERROR)){
        cout << "inside the loop\n";
        // Grow RRT tree
        Q qS;
        for(N = 0; N <= MAX_RRT_ITERATIONS; N++){


            //cout << "N: " << N;
            Q qRand = sampler(qRobot, GOAL_SAMPLE_PROB);

            Node* nearestNode= _T->nearestNeighbor(qRand);

            Q qNear = nearestNode->q;

            Q qDir = (qRand-qNear)/((qRand-qNear).norm2());

            // can the sampled point be reached
            if ((qRand-qNear).norm2() < eps){
                qS = qRand;

                // Has the goal been reached
                if((qS - qRobot).norm2() < 0.01){
                    // goal is close end loop
                    cout <<  "Goal reached in interations N: " << N << endl;
                    dx = computeDisplacement(qS);
                    _T->add(qS, nearestNode, dx[0], dx[1]);
                    break;
                }
            }else{
                qS = qNear + qDir*eps;
            }


            // constrain the point
            if(RGDNewConfig(qS, dMax, 500,500,RGD_MIN_ERROR)){

                //cout << "qS " << qS << endl;

                nearestNode = _T->nearestNeighbor(qS);

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





    cout << "checking the max iterations" << endl;
    // post path planning check
    if(N <= MAX_RRT_ITERATIONS){
        cout << "Solution found \n";
        // print out the tree in the map
        Lego* _LegoHandle = new Lego(&state, _workcell);

        vector< vector< double> > v = _LegoHandle->getPoses();
        _T->exportTree("Tree", v);
        delete _LegoHandle;


        // fetch the path
        Node* nearestNode= _T->nearestNeighbor(qRobot);
        _T->getRootPath(nearestNode, path);

        // update tree structure
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

void Planning::pausePlanner(int aStatus){
    pausePlan = aStatus;
}

void Planning::run(){
    // uptimize on current tree if active
    isAlive= true;

    cout << "lets get to it \n";
    while (isAlive){
        // Setup
        Q q1 = Q(6, 0.583, -1.073, -2.216, -1.42175, 1.57061, 1.80533);
        Q q2 = Q(6, 0.450, -2.019, -1.296, -1.4, 1.5706, 1.672);

        if(pausePlan == 0){
            // If the route is complete make a new one
            if (_RobotHandle->pathCompleted()){
                rw::trajectory::QPath aPath;
                cout << "hello world " << robotDirection << endl;
                cout << "in loop\n";
                if(robotDirection == 0){
                    aPath = getConstraintPath(state, q2, _RobotHandle->getQRobot(), 0.1);
                    if (aPath.size() != 0){
                        cout << "first" << endl;
                        robotDirection = 1;
                        _RobotHandle->setPath(aPath);
                    }
                }

                else if(robotDirection == 1){
                    aPath = getConstraintPath(state, q1, _RobotHandle->getQRobot(), 0.1);

                    if (aPath.size() != 0){
                        cout << "second" << endl;
                        robotDirection = 0;
                        _RobotHandle->setPath(aPath);
                    }
                }
                cout << aPath.size();
            }

            // Check if the path is still valid


            // Search for a better solution

        }

        // sleep for a duration of time
        usleep( 500000 );
    }

    cout << "i am dying\n";

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
        rw::proximity::CollisionDetector::QueryResult data;
        device->setQ(qs, state);

        bool collision = detector->inCollision(state, &data);
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
    device->setQ(q, state);
    bool collision = detector->inCollision(state, &data);
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




