#ifndef IP_H
#define IP_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "opencv2/xfeatures2d.hpp"
#include <opencv2/opencv.hpp>
#include <functional>
#include <iostream>
#include <rw/loaders/ImageLoader.hpp>

class ip {

public:
     static cv::Mat toOpenCVImage(const rw::sensor::Image& img);


};


#endif //IP_H
