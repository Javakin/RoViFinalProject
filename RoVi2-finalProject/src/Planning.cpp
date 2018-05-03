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
    VelocityScrew6D<> dx =  VelocityScrew6D<>(0,0,0,0,0,0);
    for (unsigned int i = 0; i<3; i++){
        dx[i] = eTt.P()[i];
        dx[3+i] = (RPY<>(eTt.R()))[i];

        dx_error[i] = C[i]*eTt.P()[i];
        dx_error[3+i] = C[3+i]*(RPY<>(eTt.R()))[i];
    }

    return dx_error;
}

rw::trajectory::QPath Planning::getConstraintPath(State _state, Q qGoal, Q qRobot, double eps) {
    // setup variables
    this->_state = _state;
    rw::trajectory::QPath path;
    QTrees T = QTrees(qGoal);
    Q dMax = Q(6,0.003,0.003,0.003,0.003,0.003,0.003);

    // initial conditions
    //Q old = qRobot;
    if(!RGDNewConfig(qRobot, dMax, 500,500,0.001))
        return path;
    //cout << (old-qRobot).norm2() <<  endl;

    if(!RGDNewConfig(qGoal, dMax, 500,500,0.001))
        return path;


    // grow RRT tree
    for(unsigned int N = 0; N< 10000; N++){
        Q qRand = sampler(qRobot, 0.2);
        Node* nearestNode= T.nearestNeighbor(qRand);
        Q qNear = nearestNode->getValue();
        Q qDir = (qRand-qNear)/((qRand-qNear).norm2());
        Q qS = qNear + qDir*eps;

        // constrain the point
        if(RGDNewConfig(qS, dMax, 500,500,0.001)){
            nearestNode = T.nearestNeighbor(qS);
            qNear = nearestNode->getValue();


            qDir = (qS-qNear)/((qS-qNear).norm2())*eps;

            if (qDir.norm2() < (qS-qNear).norm2()){
                qS = qNear + qDir;
            }

            // check for edge colliitons
            if(expandedBinarySearch(qS, qNear, 0.001)){
                T.add(qS, nearestNode);

                //nearestNode = T.nearestNeighbor(qRobot);
                //cout << N  << (nearestNode->getValue() - qRobot).norm2()<< endl;


                // has the goal been reached
                if((qS - qRobot).norm2() < 0.1){
                    // goal is close end loop
                    cout <<  "N: " << N << endl;
                    break;
                }
            }

        }

        if(N > 900){
            cout << "No solution found\n";
            return path;
        }
    }

    cout << "RRT done\n";



    // fetch the path
    Node* nearestNode= T.nearestNeighbor(qRobot);
    T.getRootPath(*nearestNode, path);

    // add the goal to the path
    //path.push_back(qRobot);

    return path;
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
    //cout << endl << "initial Q: " << qs << endl;
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


QPath Planning::createNewPath(double &outTime, double epsilon, int seed, WorkCell::Ptr wc, Device::Ptr device, State state) {
    rw::math::Math::seed(seed);


    PlannerConstraint constraint = PlannerConstraint::make(detector,device,state);

    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();

    QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, qSamples, metric, epsilon, RRTPlanner::RRTConnect);


    Q from = Q(6, 0.583, -1.073, -2.216, -1.42175, 1.57061, 1.80533);
    Q to = Q(6, 0.450, -2.019, -1.296, -1.4, 1.5706, 1.672);

    device->setQ(from, state);
    if (!inCollision(device, state, *detector, from))
        return 0;
    if (!inCollision(device, state, *detector, to))
        return 0;


    cout << "Planning from " << from << " to " << to << endl;

    QPath path;
    Timer t;
    t.resetAndResume();
    if(planner->query(from,to,path,MAXTIME)){
        cout << "A good path was found";
    }

    t.pause();
    outTime = t.getTime();
    return path;
}


rw::trajectory::QPath Planning::RRTConnect(State state, Q from, Q to, double epsilon) {

    this->_state = state;


    rw::math::Math::seed(time(NULL));

    rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(detector,device, _state);

    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    double extend = epsilon;

    rw::pathplanning::QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, qSamples, metric, extend, RRTPlanner::RRTConnect);

    // check for collision in the initial points
    if (!inCollision(device, _state, *detector, from))
        return 0;

    if (!inCollision(device, _state, *detector, to))
        return 0;


    rw::trajectory::QPath path;

    cout << "Empty path:\n";
    for(unsigned int i = 0; i < path.size(); i++)
        cout << i << ": " << path[i] << endl;

    planner->query(from,to,path,MAXTIME);

    cout << "Printing path:\n";
    for(unsigned int i = 0; i < path.size(); i++)
        cout << i << ": " << path[i] << endl;

    return path;
}

bool Planning::inCollision(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
    State testState;
    CollisionDetector::QueryResult data;
    bool colFrom;

    testState = state;
    device->setQ(q,testState);
    colFrom = detector.inCollision(testState,&data);
    if (colFrom) {
        cerr << "Configuration in collision: " << q << endl;
        cerr << "Colliding frames: " << endl;
        FramePairSet fps = data.collidingFrames;
        for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
            cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
        }
        return false;
    }
    return true;
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


