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
    static cv::Mat segmentateHSV(cv::Mat image, int hueMin, int hueMax, int saturationMin, int saturationMax, int valueMin, int valueMax);
	static cv::Mat opening(cv::Mat image, int elementType, int kernelSize);
	static cv::Mat closing(cv::Mat image, int elementType, int kernelSize);
	static std::vector<std::vector<cv::Point> > findContours(cv::Mat image);
	static cv::Mat drawContours(std::vector<std::vector<cv::Point> > contours, cv::Mat image);
	static std::vector<cv::Moments> getMu(std::vector<std::vector<cv::Point> > contours );
	static std::vector<cv::Point2i> getCenterPoints(std::vector<cv::Moments> moments, std::vector<std::vector<cv::Point> > contours);
	static cv::Mat drawPoints(std::vector<cv::Point2i> points, cv::Mat image, cv::Vec3b color);
	static std::vector<cv::Point2i> toRobotPoints(std::vector<cv::Point2i> points, cv::Mat image);
    static std::vector<cv::Point2i> decideOnBlueMarkers(std::vector<cv::Point2i> mcBlue, std::vector<cv::Point2i> mcRed);
    static std::vector<cv::Point2i> marker1Function(const rw::sensor::Image& image);
    static std::vector<cv::Point2i> marker3Function(const rw::sensor::Image& image, cv::Ptr<cv::xfeatures2d::SURF> _detector, cv::Mat _descriptors_object,std::vector<cv::KeyPoint> _keypoints_object,cv::Mat _img_object);
    static cv::Mat toOpenCVImage(const rw::sensor::Image& img);
};


#endif //IP_H
