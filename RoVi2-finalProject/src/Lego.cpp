//
// Created by daniel on 4/11/18.
//

#include "Lego.hpp"
#include <string>

#define BRICK_BASE_NAME     "LegoYellow"
#define NUMBER_OF_BRICKS    10
#define NUMBER_OF_NUMBERS   2

#define X_OUT_OF_VISION     1.0
#define LEGO_SPACING        0.025

Lego::Lego() {
    // set the default state of the object
    _workcell = NULL;
    _state = NULL;
}

Lego::Lego(State *_state, WorkCell::Ptr _workcell) {
    // initialize the workspace
    this->_state = _state;
    this->_workcell = _workcell;

    // finde the legobricks
    for(unsigned int i = 1; i<=NUMBER_OF_BRICKS; i++){
        // Generate the names of the bircks
        string sBrickID = to_string(i);
        while(sBrickID.length()<NUMBER_OF_NUMBERS){
            sBrickID = "0" + sBrickID;
        }

        sBrickID = BRICK_BASE_NAME + sBrickID;

        //Store the movable frame pointer
        vLegoFrames.push_back(_workcell->findFrame<MovableFrame>(sBrickID));
    }


    // Move the marker frame a noteceble amount
    for(unsigned int ID = 0; ID<NUMBER_OF_BRICKS; ID++){
        Vector3D<> vec(X_OUT_OF_VISION + ID*LEGO_SPACING,0,0);
        RPY<> ang(0,0,0);

        vLegoFrames[ID]->setTransform(Transform3D<>(vec, ang.toRotation3D()), *_state);

    }


}

void Lego::move(int iLegoID, double dx){


}

void Lego::move(double dx) {

}

void Lego::removeFromView() {

}

void Lego::removeFromView(int iLegoID) {

}

vector<MovableFrame *> Lego::getFrames() {
    return vector<MovableFrame *>();
}

MovableFrame *Lego::getFrame(int iLegoID) {
    return nullptr;
}


