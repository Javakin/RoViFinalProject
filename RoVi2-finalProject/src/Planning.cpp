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
    C =  VelocityScrew6D<>(0,0,1,0,1,1);

    device = this->_workcell->findDevice("UR1");
    gripper = this->_workcell->findDevice("WSG50");

    // initialize random function
    srand (time(NULL));

    // set up collision detector
    detector = new rw::proximity::CollisionDetector(_workcell, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());

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

rw::trajectory::QPath Planning::getConstraintPath(State _state, Q qGoal, Q qRobot) {
    this->_state = _state;



    Q dMax1 = Q(6,MAX_JOINT_ROTATION);
    cout << randomDisplacement(dMax1) << endl;

    Q dMax = Q(6,0.01,0.01,0.01,0.01,0.01,0.01);
    //RGDNewConfig(QGoal, dMax, 5000,500,0.001);


    // compute a path for testing if the robot
    path = rw::trajectory::QPath();

    path.push_back(qRobot);

    Q currentPos = qRobot;
    Q qDir = (qGoal-qRobot)/(qGoal-qRobot).norm2();
    while((currentPos-qGoal).norm2() < GOAL_EBS*2){
        Q qDir = (qGoal-currentPos)/(qGoal-currentPos).norm2();
        currentPos +=qDir*GOAL_EBS;
        path.push_back(currentPos);
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
