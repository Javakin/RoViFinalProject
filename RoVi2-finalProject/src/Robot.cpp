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
    device = _workcell->findDevice("UR1");
    uiPathIterator = 0;

}

Robot::~Robot() {

}

void Robot::setQ(Q qRobot) {
    device->setQ(qRobot, *_state);
}

void Robot::setPath(rw::trajectory::QPath aPath) {
    path = aPath;
}

int Robot::nextState() {
    int statusSignal = 0;

    cout << "move robot\n";
    //precondition
    if (path.size() == 0)
        return -1;

    // return 0 if path is fully executed else return 1
    if(uiPathIterator < path.size() - 1){
        statusSignal = 1;
        cout << "moved robot\n";
        uiPathIterator++;
        setQ(path[uiPathIterator]);
    }

    return statusSignal;
}
