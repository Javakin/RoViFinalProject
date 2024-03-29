//
// Created by daniel on 10/5/18.
//

#include "Vision.hpp"


// ***************************************
// public functions
//
// ***************************************
Vision::Vision(){
    _LegoHandle = nullptr;
    current_msg = nullptr;

    // setup ROS
    ROS_INFO("Connected to roscore");

    _sub = _nh.subscribe("coordinates", 100, &Vision::cameraCallBack, this);

}
Vision::Vision(Lego* _aLegoHandle){
    _LegoHandle = _aLegoHandle;
    current_msg = nullptr;

    // setup ROS
    ROS_INFO("Connected to roscore");

    _sub = _nh.subscribe("coordinates", 100, &Vision::cameraCallBack, this);
}

void Vision::run(){
    while(ros::ok())
    {
        ros::spinOnce();

        // Adjust the sleep to, according to how often you will check ROS for new messages
        ros::Duration(0.1).sleep();
    }
}

void Vision::savePoint(){
    double timestamp;
    vector<vector<double> > legoPos = readMsg(current_msg->data.c_str(), timestamp);
    _LegoHandle->cameraCalibration(legoPos);
}

void Vision::update(){
    double timestamp;
    vector<vector<double> > legoPos = readMsg(current_msg->data.c_str(), timestamp);
    _LegoHandle->placeLegos(legoPos);
}



// ***************************************
// private functions
//
// ***************************************

void Vision::cameraCallBack(const std_msgs::String::ConstPtr & msg){

    current_msg = msg;
    //ROS_INFO("I Heard [%s]", msg->data.c_str());
}



std::vector<std::vector<double> > Vision::readMsg(std::string msg, double &timeStamp)
{
    std::string temp = "";
    int index = 0;
    std::vector<std::vector<double> > results;
    std::vector<double> stateVector;

    while (msg[index] != ':') {
        temp += msg[index];
        index++;
    }

    timeStamp = std::stod(temp);

    while (msg[index] != ';') {
        if (msg[index] == '(') {
            stateVector.resize(0);
            temp = "";
        } else if (msg[index] == ',') {
            stateVector.push_back(stod(temp));
            temp = "";
        }else if (msg[index] == ')'){
            results.push_back(stateVector);
        }else{
            temp += msg[index];
        }
        index++;
    }

    return results;
}