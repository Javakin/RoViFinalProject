#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

using namespace std;
//void chatterCallback(const std_msgs::String::ConstPtr& msg)
//{
//	ROS_INFO("I heard: [%s]", msg->data.c_str());
//}

int frikadelle1 = 0;
int sem = 0;

void imageCallbackLeft(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {

        if (sem == 1)
        {
            //sem = 0;
            cv::Mat inputImage;
            inputImage = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::imshow("left", inputImage);
            std::string lort = "left" + std::to_string(frikadelle1) + ".png";
            cv::imwrite("left/"+lort, inputImage);
            cout <<  "L: " << frikadelle1 << endl;
        }

    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Coud not convert from '%s' to 'jpg'.", msg->encoding.c_str());
    }

}

void imageCallbackRight(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {

        frikadelle1++;
        cout <<  "R: " <<  frikadelle1 << endl;
        sem = 1;
        cv::waitKey(100);

        cv::Mat inputImage;
        inputImage = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::imshow("right", inputImage);
        std::string lort = "right" + std::to_string(frikadelle1) + ".png";
        cv::imwrite("right/"+lort, inputImage);



    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Coud not convert from '%s' to 'jpg'.", msg->encoding.c_str());
    }

}


void imageCallBack(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right)
{
    //do stuff with images.
    double stamp_1 = left->header.stamp.toSec();
    double stamp_2 = right->header.stamp.toSec();
    ROS_INFO("\n CallBack called \n Timestamp_left: %f \n Timestamp_right: %f\n", stamp_1, stamp_2);

    cv::Mat inputImage;
    inputImage = cv_bridge::toCvShare(left, "bgr8")->image;
    cv::imshow("left", inputImage);
    std::string lort = "left" + std::to_string(frikadelle1) + ".png";
    cv::imwrite("left/"+lort, inputImage);

    inputImage = cv_bridge::toCvShare(right, "bgr8")->image;
    lort = "right" + std::to_string(frikadelle1) + ".png";
    cv::imshow("right", inputImage);
    cv::imwrite("right/"+lort, inputImage);

    frikadelle1++;



    cv::waitKey(100);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_node");

    ros::NodeHandle n;


    // define the subscribers



    message_filters::Subscriber<sensor_msgs::Image> image1_sub(n, "camera/left/image_color",5);
    message_filters::Subscriber<sensor_msgs::Image> image2_sub(n, "camera/right/image_color",5);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub);
    //sync.setAgePenalty(1.0);
    sync.registerCallback(boost::bind(imageCallBack, _1, _2));



    /*
image_transport::ImageTransport it(n);


image_transport::Subscriber subleft = it.subscribe("camera/left/image_color",1, imageCallbackLeft, ros::VoidPtr(), image_transport::TransportHints("compressed"));
image_transport::Subscriber subright = it.subscribe("camera/right/image_color",1, imageCallbackRight, ros::VoidPtr(), image_transport::TransportHints("compressed"));
*/

    ros::spin();

    //cv::destroyWindow("lort");

    return 0;

}




