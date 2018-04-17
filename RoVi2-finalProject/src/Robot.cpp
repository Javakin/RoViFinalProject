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
