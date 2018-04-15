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
    vLegoFrames.clear();
    isTracked.clear();

    //cout << "step1\n";
    // Finde the legobricks
    for(unsigned int i = 1; i<=NUMBER_OF_BRICKS; i++){
        // Generate the names of the bircks
        string sBrickID = to_string(i);
        while(sBrickID.length()<NUMBER_OF_NUMBERS){
            sBrickID = "0" + sBrickID;
        }

        sBrickID = BRICK_BASE_NAME + sBrickID;

        //Store the movable frame pointer
        vLegoFrames.push_back(_workcell->findFrame<MovableFrame>(sBrickID));
        isTracked.push_back(0);
    }

    //cout << "step 2\n";
    // Initialize the lego bricks position out of camera sight and withoute collision
    for(unsigned int ID = 0; ID<NUMBER_OF_BRICKS; ID++){
        Vector3D<> vec(X_OUT_OF_VISION + ID*LEGO_SPACING,0,0);
        RPY<> ang(0,0,0);

        vLegoFrames[ID]->setTransform(Transform3D<>(vec, ang.toRotation3D()), *_state);
        //isTracked[ID] = 0;
    }
}

void Lego::move(int iLegoID, double dx){


}

void Lego::move(double dx) {
    // Move all active the legobricks one step of dx
    for(unsigned int ID = 0; ID<NUMBER_OF_BRICKS; ID++){
        if(isTracked[ID]) {
            // Move one step of dx
            Transform3D<> CurrentPose = vLegoFrames[ID]->getTransform(*_state);

            Vector3D<> vec(dx, 0, 0);
            RPY<> ang(0, 0, 0);

            CurrentPose = CurrentPose * Transform3D<>(vec, ang.toRotation3D());

            // Enable wraparound
            if(CurrentPose.P()[0] > X_RANGE){
                CurrentPose.P()[0] = CurrentPose.P()[0] - X_RANGE*2;
            }
            if(CurrentPose.P()[0] < -X_RANGE){
                CurrentPose.P()[0] = CurrentPose.P()[0] + X_RANGE*2;
            }

            vLegoFrames[ID]->setTransform(CurrentPose, *_state);
        }
    }


}

void Lego::removeFromView() {

}

void Lego::removeFromView(int iLegoID) {

}

vector<MovableFrame *> Lego::getFrames() {
    return vector<MovableFrame *>();
}

void Lego::trackLego(unsigned int iLegoID) {
    if(iLegoID >isTracked.size())
        isTracked[iLegoID] = 1;
}

void Lego::initializeTestSetup() {
    // set all lego bricks to be tracked
    cout << "here";
    for(unsigned int ID = 0; ID<NUMBER_OF_BRICKS; ID++){
        isTracked[ID] = 1;
    }

    // place all lego bricks so that non of them is tuching each other
    cout << "here";
}


