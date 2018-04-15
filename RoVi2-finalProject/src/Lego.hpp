//
// Created by daniel on 4/11/18.
//
#pragma once

#ifndef OBJECTAVOIDANCE_LEGO_H
#define OBJECTAVOIDANCE_LEGO_H

// ------------------- includes ----------------------------
//RobWork includes
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rw/kinematics/MovableFrame.hpp>

#include <rw/math.hpp> // Pi, Deg2Rad
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

//RobWork studio includes
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>

#include <rws/RobWorkStudio.hpp>

#include <vector>

// --------------------  namespaces ----------------------------
using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rw::math;

using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace std;
using namespace rws;


// -----------------------   defines ----------------------------
#define NUM_OF_LEGOBRICKS   10
#define X_RANGE             0.58
#define Y_RANGE             0.15






class Lego {

public:
    Lego();
    Lego(State* _state, WorkCell::Ptr _workcell);
    void initializeTestSetup();
    void trackLego(unsigned int iLegoID);
    void move(int iLegoID, double dx);
    void move(double dx);

    void removeFromView();
    void removeFromView(int iLegoID);
    vector<MovableFrame*> getFrames();



private:
    vector<MovableFrame*> vLegoFrames;
    vector<int> isTracked;

    rw::kinematics::State* _state;
    rw::models::WorkCell::Ptr _workcell;
};


#endif //OBJECTAVOIDANCE_LEGO_H
