//
// Created by daniel on 4/17/18.
//
#pragma once

#ifndef OBJECTAVOIDANCE_PLANNING_H
#define OBJECTAVOIDANCE_PLANNING_H

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


#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

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



class Planning {
public:

    Planning();
    Planning(WorkCell::Ptr _workcell);
    rw::trajectory::QPath getConstraintPath(State _state, Q QGoal, Q qRobot);
    ~Planning();


private:

    VelocityScrew6D<> computeTaskError(Q qSample);
    bool RGDNewConfig(Q &qs, Q dMax, int MaxI, int MaxJ, double eps);
    Q randomDisplacement(Q dMax);

    rw::kinematics::State _state;
    rw::models::WorkCell::Ptr _workcell;
    Device::Ptr device;
    Device::Ptr gripper;

    VelocityScrew6D<> C;
    rw::trajectory::QPath path;
    rw::proximity::CollisionDetector::Ptr detector;




};


#endif //OBJECTAVOIDANCE_PLANNING_H
