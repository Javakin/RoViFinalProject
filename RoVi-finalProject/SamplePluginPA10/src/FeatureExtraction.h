//
// Created by student on 12/10/17.
//

#ifndef FEATUREEXTRACTION_FEATUREEXTRACTION_H
#define FEATUREEXTRACTION_FEATUREEXTRACTION_H


#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"

// RobWork includes
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


//#include <rw/rw.hpp>



// RobWorkStudio includes
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>

#include <rws/RobWorkStudio.hpp>

#include "opencv2/calib3d/calib3d.hpp"

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;

class FeatureExtraction {
public:
    //FeatureExtraction();
    FeatureExtraction(int iHassanThreshhold, rw::common::LogWriter &log);

    // generate the feachures
    void setMarker(Mat mMarker);
    vector<Point2f> matchfeachures(Mat mImage);

private:
    cv::Ptr<SURF> detector;
    std::vector<KeyPoint> vKeyPointsMarker;
    Mat mDescriptorsMaker;
    Mat mMarker;
    rw::common::LogWriter &log;

};


#endif //FEATUREEXTRACTION_FEATUREEXTRACTION_H
