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

//Ros includes
#include <ros/ros.h>
#include <caros_control_msgs/RobotState.h>
#include <caros/serial_device_si_proxy.h>
#include <caros_common_msgs/Q.h>
#include <caros/common_robwork.h>
#include <QThread>
#include <QObject>

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
#define SUBSCRIBER "/caros_universalrobot/caros_serial_device_service_interface/robot_state"
#define ROBOT_MAX_SPEED     0.1


class Robot: public QThread  {
Q_OBJECT

public:
    Robot();
    void setQ(Q qRobot);
    Robot(State* _state, WorkCell::Ptr _workcell);
    ~Robot();

    int update();

    void setPath(rw::trajectory::QPath aPath);
    Q getQRobot();

    void moveHome();
    void moveQ(Q q);
    bool pathCompleted();
    void pauseRobot(bool astatus);

    /// This method contains the ROS event loop. Feel free to modify
    void run();


private:
    rw::kinematics::State* _state;
    Device::Ptr device;
    rw::models::WorkCell::Ptr _workcell;
    rw::trajectory::QPath path;

    unsigned int uiPathIterator;

    /// Callback function
    void stateCallback(const caros_control_msgs::RobotState & msg);

    bool quitfromgui;

    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    caros::SerialDeviceSIProxy* _robot;
    bool isPaused;

};


#endif //OBJECTAVOIDANCE_ROBOT_H
