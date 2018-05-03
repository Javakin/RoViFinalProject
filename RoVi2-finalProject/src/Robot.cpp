//
// Created by daniel on 4/17/18.
//

#include "Robot.hpp"

#include <opencv2/highgui/highgui.hpp>

Robot::Robot() {
    _workcell = NULL;
    _state = NULL;

    // setup ROS
    // Subscribe to caros robot state
    ROS_INFO("Connected to roscore");

    _sub = _nh.subscribe(SUBSCRIBER, 1, &Robot::stateCallback, this);

    _robot = new caros::SerialDeviceSIProxy(_nh, "caros_universalrobot");

    quitfromgui = false;
}

Robot::Robot(State *_state, WorkCell::Ptr _workcell) {
    this->_state = _state;
    this->_workcell = _workcell;
    device = _workcell->findDevice("UR1");
    uiPathIterator = 0;
    path = rw::trajectory::QPath();


    // setup ROS
    // Subscribe to caros robot state
    ROS_INFO("Connected to roscore");

    _sub = _nh.subscribe(SUBSCRIBER, 1, &Robot::stateCallback, this);

    _robot = new caros::SerialDeviceSIProxy(_nh, "caros_universalrobot");

    quitfromgui = false;


    // to initialize the robot
    moveHome();

}

Robot::~Robot() {

}

void Robot::setQ(Q qRobot) {
    device->setQ(qRobot, *_state);
}

void Robot::setPath(rw::trajectory::QPath aPath) {
    path = aPath;
    uiPathIterator = 0;
}

void Robot::update(){

    nextState();
}

int Robot::nextState() {
    int statusSignal = 0;

    //precondition
    if (path.size() == 0)
        return -1;

    // return 0 if path is fully executed else return 1
    if(uiPathIterator < path.size() - 1 && (getQRobot()-path[uiPathIterator]).norm2() < 0.1){
        //cout << "Pathiterator" << uiPathIterator << endl;
        statusSignal = 1;
        uiPathIterator++;
        moveQ(path[uiPathIterator]);
    }

    return statusSignal;
}


void Robot::stateCallback(const caros_control_msgs::RobotState &msg)
{
    // Extract configuration from RobotState message
    caros_common_msgs::Q conf = msg.q;

    // Convert from ROS msg to Robwork Q
    rw::math::Q conf_rw = caros::toRw(conf);

    // update the robots configuration
    setQ(conf_rw);
}

void Robot::moveHome()
{
    //ROS_INFO("Called move home");
    float speed = 0.1;
    rw::math::Q home = rw::math::Q(6, 0, -M_PI/2.0, 0, -M_PI/2.0, 0, 0);
    _robot->moveServoQ(home, speed);
}

void Robot::moveQ(Q q)
{
    //ROS_INFO("Called move to a configuration");
    float speed = 0.1;
    _robot->moveServoQ(q, speed);

}

void Robot::run()
{
    while(ros::ok() && !quitfromgui)
    {
        ros::spinOnce();

        // Adjust the sleep to, according to how often you will check ROS for new messages
        ros::Duration(0.1).sleep();
    }
    if (!quitfromgui)
    {
        //emit rosQuits();
        ROS_INFO("ROS-Node Terminated\n");
    }
}

Q Robot::getQRobot() {
    return device->getQ(*_state);
}


bool Robot::pathCompleted(){
    //cout << uiPathIterator << "int " << path.size() - 1 << "and" <<  ((getQRobot()-path[uiPathIterator]).norm2() < 0.1) << endl;

    if (path.size() == 0)
        return 1;
    if (uiPathIterator >= path.size() - 1 && (getQRobot()-path[uiPathIterator]).norm2() < 0.1)
        return 1;
    return 0;
}