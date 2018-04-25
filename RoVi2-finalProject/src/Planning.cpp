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

    //cout << "deltaX: " << dx << endl;
    //cout << "dx_error: " << dx_error << endl;

    return dx_error;
}

rw::trajectory::QPath Planning::getConstraintPath(State _state, Q qGoal, Q qRobot, double eps) {
    this->_state = _state;

    QTrees T = QTrees(qGoal);
    Q dMax = Q(6,0.003,0.003,0.003,0.003,0.003,0.003);


    for(unsigned int N = 0; N< 100; N++){
        Q qRand = sampler(qGoal);
        Node* nearestNode= T.nearestNeighbor(qRand);
        Q qNear = nearestNode->getValue();
        Q qDir = (qRand-qNear)/((qRand-qNear).norm2());
        Q qS = qNear + qDir*eps;
        if(RGDNewConfig(qS, dMax, 500,500,0.001)){
            T.add(qS, nearestNode);
        }
    }



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
    cout << "starting\n";
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
            cout << i  << "," << j << ": " << dx_error.norm2() << endl;
            j = 0;
        }

    }
    cout << i  << "," << j << ": " << dx_error.norm2() << endl;

    cout << "ending\n";
    cout << qs << endl;


    // check that the solution is good


    if(dx_error.norm2() <= eps){
        rw::proximity::CollisionDetector::QueryResult data;
        device->setQ(qs, _state);
        bool collision = detector->inCollision(_state, &data);
        if(collision)
        {
            return false;
        }

        return true;
    }
    return false;
}

Q Planning::sampler(Q qGoal) {
    Q outPut = qGoal;

    if (((double)rand()/RAND_MAX) < GOAL_SAMPLING_PROB){
        return qSamples->sample();
    }

    return qGoal;
}

rw::trajectory::QPath Planning::pathOptimization(rw::trajectory::QPath aPath) {
    return aPath;
}
