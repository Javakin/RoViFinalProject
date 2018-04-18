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

    device = _workcell->findDevice("UR1");
    gripper = _workcell->findDevice("WSG50");

}

Planning::~Planning() {


}

VelocityScrew6D<> Planning::Compute_Task_Error(Q qSample) {

    Frame* TaskFrame = _workcell->findFrame("TaskFrame");


    Transform3D<> wTt = inverse(TaskFrame->wTf(_state));


    device->setQ(qSample, _state);
    Frame* EndEffector = _workcell->findFrame("EndEffektor");
    Transform3D<> eTw = EndEffector->wTf(_state);
    Transform3D<> eTt = wTt*eTw;

    cout << "deltaX: " << eTt << endl;

    //caclulate the dx_error

    VelocityScrew6D<> dx_error =  VelocityScrew6D<>(0,0,0,0,0,0);
    for (unsigned int i = 0; i<3; i++){
        dx_error[i] = C[i]*eTt.P()[i];
        dx_error[3+i] = C[3+i]*(RPY<>(eTt.R()))[i];
    }

    cout << "dx_error: " << dx_error << endl;

    return dx_error;
}

rw::trajectory::QPath Planning::getConstraintPath(State _state, Q QGoal, Q qRobot) {
    this->_state = _state;

    Compute_Task_Error(qRobot);


    return rw::trajectory::QPath();
}
