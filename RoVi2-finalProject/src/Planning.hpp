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
#include "Robot.hpp"
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

#define MAX_RRT_ITERATIONS   500

#define GOAL_SAMPLE_PROB     0.1
#define RGD_MIN_ERROR        0.001
#define RGD_MAX_ERROR        0.01
#define EDGE_CHECK_EBS       0.01
#define VALIDAITON_DEPTH     0.8
#define VALIDATION_BOUND     0.25
#define RRT_EPSILON          0.1
#define IMPROVEMENT_FACTOR   0.05
#define IMPROVEMENT_FACTORCB 0.05
#define NUM_OF_NEIGHBORS     3



class Planning : public QThread {
public:

    Planning();
    Planning(WorkCell::Ptr _workcell, rw::kinematics::State::Ptr  _state, Robot* _RobotHandle);
    ~Planning();

    QPath getConstraintPath(Q qGoal, Q qRobot, double eps);
    QPath RRTC(State state, Q qRobot, Q qGoal, double epsilon);
    QPath updateConstraindPath(Q qGoal, double eps);
    QPath validate(double CheckingDebth);
    QPath repareTree();
    void printTree();

    void pausePlanner(int status);

    // for running the path planner thread
    void run();

private:
    void printTree(QTrees* _tree, rw::kinematics::State aState);

    VelocityScrew6D<> computeTaskError(Q qSample);
    VelocityScrew6D<> computeDisplacement(Q qSample);

    bool RGDNewConfig(Q &qs, Q dMax, int MaxI, int MaxJ, double eps);
    Q randomDisplacement(Q dMax);
    Q sampler(Q qGoal, double goalSampleProb);


    bool expandedBinarySearch(Q StartConf, Q EndConf, double eps);
    bool expandedBinarySearch(rw::kinematics::State::Ptr  _state, Q StartConf, Q EndConf, double eps);


    bool inCollision(const Q &q);
    bool inCollision(rw::kinematics::State::Ptr  _state, const Q &q);

    bool constrainedRRT(QTrees* _T, Q qGoal, double eps, int numOfNearestNodes);

    rw::kinematics::State state;
    rw::kinematics::State::Ptr  _state;

    rw::models::WorkCell::Ptr _workcell;
    Robot* _RobotHandle;
    Device::Ptr device;
    Device::Ptr gripper;
    bool isAlive;
    int robotDirection;
    int pausePlan;
    int InUse;

    VelocityScrew6D<> C;
    rw::proximity::CollisionDetector::Ptr detector;
    rw::pathplanning::QSampler::Ptr qSamples;

    //QTrees* _T;
    QTrees* _R;

};


#endif //OBJECTAVOIDANCE_PLANNING_H
