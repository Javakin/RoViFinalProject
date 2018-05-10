//
// Created by daniel on 4/17/18.
//

#ifndef OBJECTAVOIDANCE_VIS_H
#define OBJECTAVOIDANCE_VIS_H

//Ros includes
#include <ros/ros.h>
#include <caros_control_msgs/RobotState.h>
#include <caros/serial_device_si_proxy.h>
#include <caros_common_msgs/Q.h>
#include <caros/common_robwork.h>
#include <QThread>
#include <QObject>

#include <vector>
#include <std_msgs/String.h>
#include "Lego.hpp"




using namespace std;




class Vision : public QThread{
Q_OBJECT

public:
    Vision();
    Vision(Lego* _aLegoHandle);
    void savePoint();
    void run();

private:
    void cameraCallBack(const std_msgs::String::ConstPtr & msg);
    vector<vector<double> > readMsg(std::string msg, double &timeStamp);


    Lego* _LegoHandle;

    // ros
    ros::NodeHandle _nh;
    ros::Subscriber _sub;

    std_msgs::String::ConstPtr current_msg;


};
#endif //OBJECTAVOIDANCE_VIS_H
