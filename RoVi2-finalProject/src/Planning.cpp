//
// Created by daniel on 4/17/18.
//

#include "Planning.hpp"

Planning::Planning() {
    _state = NULL;
    _workcell = NULL;

}

Planning::Planning(State *_state, WorkCell::Ptr _workcell) {
    this->_workcell = _workcell;
    this->_state = _state;

}

Planning::~Planning() {
    

}
