//
// Created by daniel on 4/17/18.
//

#ifndef OBJECTAVOIDANCE_ROBOT_H
#define OBJECTAVOIDANCE_ROBOT_H

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



class Robot {
public:
    Robot();
    Robot(State* _state, WorkCell::Ptr _workcell);
    ~Robot();
private:
    rw::kinematics::State* _state;
    rw::models::WorkCell::Ptr _workcell;
};


#endif //OBJECTAVOIDANCE_ROBOT_H
