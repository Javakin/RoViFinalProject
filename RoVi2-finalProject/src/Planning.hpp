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
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>

#include <rw/math.hpp> // Pi, Deg2Rad
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/pathplanning.hpp>

//RobWork studio includes
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>

#include <rws/RobWorkStudio.hpp>


#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include "QTrees.hpp"
#include "Lego.hpp"
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

using namespace rwlibs::proximitystrategies;

using namespace std;
using namespace rws;

using namespace rw::trajectory;
using namespace rw::proximity;
using namespace rw::pathplanning;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;




// -----------------------   defines ----------------------------
#define MAX_JOINT_ROTATION  6.2
#define GOAL_EBS            0.005


#define Q0_WEIGHT           0.0086
#define Q1_WEIGHT           0.0086
#define Q2_WEIGHT           0.0162
#define Q3_WEIGHT           0.0832
#define Q4_WEIGHT           0.0955
#define Q5_WEIGHT           0.7879

#define MAX_RRT_ITERATIONS 4000





class Planning {
public:

    Planning();
    Planning(WorkCell::Ptr _workcell);
    ~Planning();

    QPath getConstraintPath(State _state, Q qGoal, Q qRobot, double eps);
    QPath RRTC(State state, Q qRobot, Q qGoal, double epsilon);
    QPath updateConstraindPath(State* _state);


private:


    VelocityScrew6D<> computeTaskError(Q qSample);
    VelocityScrew6D<> computeDisplacement(Q qSample);

    bool RGDNewConfig(Q &qs, Q dMax, int MaxI, int MaxJ, double eps);
    Q randomDisplacement(Q dMax);
    Q sampler(Q qGoal, double goalSampleProb);


    bool expandedBinarySearch(Q StartConf, Q EndConf, double eps);

    bool inCollision(const Q &q);

    rw::kinematics::State _state;
    rw::models::WorkCell::Ptr _workcell;
    Device::Ptr device;
    Device::Ptr gripper;

    VelocityScrew6D<> C;
    rw::proximity::CollisionDetector::Ptr detector;
    rw::pathplanning::QSampler::Ptr qSamples;

    QTrees T;
    QTrees R;

};


#endif //OBJECTAVOIDANCE_PLANNING_H
