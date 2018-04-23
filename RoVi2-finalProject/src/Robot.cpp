//
// Created by daniel on 4/17/18.
//

#include "Robot.hpp"

Robot::Robot() {
    _workcell = NULL;
    _state = NULL;

}

Robot::Robot(State *_state, WorkCell::Ptr _workcell) {
    this->_state = _state;
    this->_workcell = _workcell;

}

Robot::~Robot() {

}

void Robot::setQ(Q qRobot) {
    Device::Ptr device;
    device = _workcell->findDevice("UR1");
    device->setQ(qRobot, *_state);

}

void Robot::setPath(rw::trajectory::QPath aPath) {
    path = aPath;
}
