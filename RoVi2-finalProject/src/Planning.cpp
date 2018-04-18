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
    C =  VelocityScrew6D<>(0,0,1,1,1,0);

    device = this->_workcell->findDevice("UR1");
    gripper = this->_workcell->findDevice("WSG50");

}

Planning::~Planning() {


}

VelocityScrew6D<> Planning::Compute_Task_Error(Q qSample) {

    Frame* TaskFrame = _workcell->findFrame("TaskFrame");


    Transform3D<> wTt = TaskFrame->wTf(_state);
    Transform3D<> tTw = inverse(wTt);
    cout << "tTw" << tTw << endl;



    device->setQ(qSample, _state);
    //device->baseTend(_state);
    device->worldTbase(_state);
    Frame* EndEffector = _workcell->findFrame("EndEff");
    cout << EndEffector->getName() << endl;
    Transform3D<> wTe = EndEffector->wTf(_state);
    Transform3D<> eTt = tTw*wTe;

    cout << "wTe" << wTe << endl;
    cout << "eTt" << eTt << endl;



    //calculate the dx_error
    VelocityScrew6D<> dx_error =  VelocityScrew6D<>(0,0,0,0,0,0);
    VelocityScrew6D<> dx =  VelocityScrew6D<>(0,0,0,0,0,0);
    for (unsigned int i = 0; i<3; i++){
        dx[i] = eTt.P()[i];
        dx[3+i] = (RPY<>(eTt.R()))[i];

        dx_error[i] = C[i]*eTt.P()[i];
        dx_error[3+i] = C[3+i]*(RPY<>(eTt.R()))[i];
    }

    cout << "deltaX: " << dx << endl;
    cout << "dx_error: " << dx_error << endl;

    return dx_error;
}

rw::trajectory::QPath Planning::getConstraintPath(State _state, Q QGoal, Q qRobot) {
    this->_state = _state;

    Compute_Task_Error(qRobot);


    return rw::trajectory::QPath();
}
