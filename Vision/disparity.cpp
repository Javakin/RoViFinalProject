/*
 * Example usage of disparity functions in OpenCV, the example is partly based on the
 * example code in stereo_match.cpp
 * Compute disparity maps and reproject to 3D.
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>


#include <iostream>
#include <math.h>

using pclPoint = pcl::PointXYZRGB;
using pclCloud = pcl::PointCloud<pclPoint>;

void printMatrix(cv::Mat matrix)
{
    std::cout << "Matrix dimension: " << matrix.rows << "x" << matrix.cols << std::endl;
    std::cout << std::endl;
    std::cout << "Matrix type: " << matrix.type() << std::endl;
    std::cout << std::endl;

    for (int i = 0; i < matrix.rows; i++){
        std::cout << "( ";
        for (int j = 0; j < matrix.cols-1; j++){
            std::cout << matrix.at<float>(i,j) << ", ";
        }

        std::cout << matrix.at<float>(i, matrix.cols-1) << " )" << std::endl;
    }

    std::cout << std::endl;

}

void printPosition(cv::Point3f position)
{
	std::cout << "( " << position.x << ", " << position.y << ", " << position.z << " )" << std::endl;
}

struct KalmanFilter
{
    KalmanFilter(int length, double initialTime, cv::Mat &initialState, cv::Mat &covariances, cv::Mat &observationErrors)
    {
        currentTime = initialTime;
        N = length;
        predictedProcessCovariance = covariances.clone();
        observationErrorMatrix = observationErrors.clone();

        lastPosition = initialState.clone();
        stateVector = cv::Mat::zeros(1,N,CV_32F);

        stateVector = initialState.clone();

        H = cv::Mat::eye(N/2, N, CV_32F);
        HTransposed = cv::Mat::eye(N, N/2, CV_32F);

        A = cv::Mat::eye(N, N, CV_32F);
        ATransposed = cv::Mat::eye(N, N, CV_32F);

        inputVector = cv::Mat::zeros(1, N,CV_32F);

        velocities = cv::Mat::zeros(N/2, 5, CV_32F);

        /*
        for(int n = 0; n < N/2; n++){
            velocities.at<float>(n,4) = initialState.at<float>(0,n+N/2);
        }
         */

    }

    void update(cv::Mat input, double time)
    {

        if (time == currentTime) {
            std::cout << "STOP THIS: " << time << std::endl;
            return;
        }

        for (int i = 0; i < N/2; i++) {
            ATransposed.at<float>(i,i+N/2) = time - currentTime;
            A.at<float>(i+N/2,i) = time - currentTime;
        }

        for (int n = 0; n < N/2; n++)
        {
            inputVector.at<float>(0,n) = input.at<float>(0,n);
            inputVector.at<float>(0,N/2+n) = (input.at<float>(0,n) - lastPosition.at<float>(0,n)) / (time - currentTime);
        }

        updateVelocityEstimate();
        if (velocityCount < 5){
            stateVector = inputVector.clone();
            lastPosition = inputVector.clone();
            currentTime = time;
            return;
        }
        projectAhead();
        updateKalmanGain();
        updatePrediction();
        updateErrorCovariance();
        currentTime = time;
        lastPosition = inputVector.clone();


    }

    void printPrediction()
    {
        printVector(stateVector);
    }


    void printVector(cv::Mat vector)
    {
        std::cout << "( " << vector.at<float>(0,0) << ", " << vector.at<float>(0,1) << ", " << vector.at<float>(0,2) << " ) ( " << vector.at<float>(0,3) << ", " << vector.at<float>(0,4) << ", " << vector.at<float>(0,5) << " )" << std::endl;
    }

    cv::Point3f getPosition(double time)
    {
        return cv::Point3f(stateVector.at<float>(0,0) + stateVector.at<float>(0,3) * (time - currentTime),
                           stateVector.at<float>(0,1) + stateVector.at<float>(0,4) * (time - currentTime),
                           stateVector.at<float>(0,2) + stateVector.at<float>(0,5) * (time - currentTime));
    }
    cv::Point3f getLastPosition()
    {
        return cv::Point3f(lastPosition.at<float>(0,0),
                           lastPosition.at<float>(0,1),
                           lastPosition.at<float>(0,2));
    }

    double getTime()
    {
        return currentTime;
    }

    bool active = true;
    int velocityCount = 0;
    double currentTime;
    cv::Mat stateVector;
    cv::Mat predictedProcessCovariance;

    std::string getState()
    {
        std::string result = std::to_string(currentTime);

        for (int n = 0; n < 6; n++){
            result += "," + std::to_string(stateVector.at<float>(0,n));
        }
        for (int n = 0; n < 6; n++){
            result += "," + std::to_string(predictedProcessCovariance.at<float>(n,n));
        }

        return result;
    }


private:

    void updateVelocityEstimate()
    {
        for (int n = 0; n < 4; n++) {
            velocities.col(n + 1).copyTo(velocities.col(n));
        }
        for (int n = 0; n < N/2; n++){
            velocities.at<float>(n,4) = inputVector.at<float>(0,N/2+n);
        }
        cv::Mat velocitySum;
        cv::reduce(velocities, velocitySum, 1, CV_REDUCE_AVG);

        for (int n = 0; n < N/2; n++){
            inputVector.at<float>(0,N/2+n) = velocitySum.at<float>(n,0);
        }
        velocityCount++;

    }

    void projectAhead()
    {
        stateVector = stateVector*A;
        predictedProcessCovariance = A*predictedProcessCovariance*ATransposed + cv::Mat::eye(N,N,CV_32F)*0.05;

    }

    void updateKalmanGain()
    {
        cv::Mat temp = predictedProcessCovariance + observationErrorMatrix;
        kalmanGain = predictedProcessCovariance * temp.inv();
        for (int i = 0; i < N; i++){
            for (int j = 0; j < N; j++){
                if (i != j)
                    kalmanGain.at<float>(i,j) = 0;
            }
        }

    }

    void updatePrediction()
    {
        cv::Mat temp;
        cv::transpose((inputVector-stateVector), temp);
        cv::transpose(kalmanGain * temp, temp);
        stateVector += temp.clone();
    }

    void updateErrorCovariance()
    {
        predictedProcessCovariance = (cv::Mat::eye(cv::Size(6,6), CV_32F) - kalmanGain)* predictedProcessCovariance;
        predictedProcessCovariance = predictedProcessCovariance.clone();
        for (int i = 0; i < N; i++){
            for (int j = 0; j < N; j++){
                if (i != j)
                    predictedProcessCovariance.at<float>(i,j) = 0;
            }
        }
    }

    int N;
    cv::Mat velocities;
    cv::Mat lastPosition;
    cv::Mat inputVector;
    cv::Mat observationErrorMatrix;
    cv::Mat kalmanGain;
    cv::Mat A;
    cv::Mat ATransposed;
    cv::Mat H;
    cv::Mat HTransposed;
};

void printMatrixDouble(cv::Mat matrix)
{
	std::cout << "Matrix dimension: " << matrix.rows << "x" << matrix.cols << std::endl;
	std::cout << std::endl;
	std::cout << "Matrix type: " << matrix.type() << std::endl;
	std::cout << std::endl;

	for (int i = 0; i < matrix.rows; i++){
		std::cout << "( ";
		for (int j = 0; j < matrix.cols-1; j++){
			std::cout << matrix.at<double>(i,j) << ", ";
		}

		std::cout << matrix.at<double>(i, matrix.cols-1) << " )" << std::endl;
	}

	std::cout << std::endl;

}

cv::Mat combineMatricesHorizontally(cv::Mat left, cv::Mat right)
{
	cv::Mat result = cv::Mat::zeros(left.rows, left.cols + right.cols, CV_64F);

	for (int i = 0; i < left.rows; i++){
		for (int j = 0; j < left.cols; j++){
			result.at<double>(i,j) = left.at<double>(i,j);
		}
	}

	for (int i = 0; i < right.rows; i++){
		for (int j = 0; j < right.cols; j++){
			result.at<double>(i,j + left.cols) = right.at<double>(i,j);
		}
	}

	return result;
}

void loadImages(int number, cv::Mat &left, cv::Mat &right, std::string path)
{
	left = cv::imread(path + "/left/left" + std::to_string(number) + ".png");
	right = cv::imread(path + "/right/right" + std::to_string(number) + ".png");
}

void combineImages(cv::Mat &left, cv::Mat &right, cv::Mat &result)
{
	std::vector<cv::Mat> channels;
	cv::Mat empty = cv::Mat::zeros(left.rows, left.cols, CV_8U);


	channels.push_back(left);
	channels.push_back(right);
	channels.push_back(empty);

	cv::merge(channels, result);


}

void opening(cv::Mat &img, int elementType, int kernelSize)
{
	cv::Mat element = cv::getStructuringElement(elementType, cv::Size(2*kernelSize+1, 2*kernelSize+1), cv::Point(kernelSize, kernelSize));

	cv::erode(img, img, element);
	cv::dilate(img, img, element);
}

void closing(cv::Mat &img, int elementType, int kernelSize)
{
	cv::Mat element = cv::getStructuringElement(elementType, cv::Size(2*kernelSize+1, 2*kernelSize+1), cv::Point(kernelSize, kernelSize));

	cv::dilate(img, img, element);
	cv::erode(img, img, element);
}

void erodeImg(cv::Mat &img, int elementType, int kernelSize)
{
	cv::Mat element = cv::getStructuringElement(elementType, cv::Size(2*kernelSize+1, 2*kernelSize+1), cv::Point(kernelSize, kernelSize));

	cv::erode(img, img, element);
}

void dilateImg(cv::Mat &img, int elementType, int kernelSize)
{
	cv::Mat element = cv::getStructuringElement(elementType, cv::Size(2*kernelSize+1, 2*kernelSize+1), cv::Point(kernelSize, kernelSize));

	cv::dilate(img, img, element);
}

std::vector<cv::Point2f> getPoints(cv::Mat &range, float minDist)
{
	cv::Mat cannyOut;
	std::vector<std::vector<cv::Point> > contours;

	///https://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/moments/moments.html
	/// Detect edges using canny
	std::vector<cv::Vec4i> hierarchy;
	cv::Canny( range, cannyOut, 100, 100*2, 3 );
	/// Find contours
	findContours( cannyOut, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    /// break if no contours
    if (contours.size() == 0) {
        std::vector<cv::Point2f> temp;
        return temp;
    }

    /*
    for (int j = 0; j < contours.size(); j++) {
        std::cout << cv::contourArea(contours[j]) << std::endl;
    }
     */
    
	/// Get the moments
	std::vector<cv::Moments> mu(contours.size() );
	for( int i = 0; i < contours.size(); i++ ) {
        if (cv::contourArea(contours[i]) > 700) {
            mu[i] = moments(contours[i], false);
        }
    }
    
	///  Get the mass centers:
	std::vector<cv::Point2f> mc( mu.size() );
	for( int i = 0; i < mu.size(); i++ )
	{
		mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );

	}

	for (int n = 0; n < mc.size()-1; n++) {
		for (int m = n + 1; m < mc.size(); m++) {
			if (abs(mc[n].x - mc[m].x) + abs(mc[n].y - mc[m].y) < minDist) {
				mc.erase(mc.begin() + m);
			}
		}
	}

    for (int n = 0; n < mc.size(); n++){
        if (mc[n].y > 700){
            mc.erase(mc.begin() + n);
            n--;
        }
    }

	for (int n = 0; n < mc.size(); n++){
		if (mc[n].x != mc[n].x){
			mc.erase(mc.begin() + n);
			n--;
		}
	}

	return mc;
}

void printDimensions(cv::Mat matrix)
{
	std::cout << matrix.rows << ", " << matrix.cols << std::endl;
}

void filterByHSV(cv::Mat &img, cv::Mat &range, cv::Scalar minHSV, cv::Scalar maxHSV, bool replace = false)
{

	cv::cvtColor(img, img, cv::COLOR_BGRA2BGR);
	cv::cvtColor(img, img, cv::COLOR_BGR2HSV);
	cv::inRange(img, minHSV, maxHSV, range);
	cv::cvtColor(img, img, cv::COLOR_HSV2BGR);

    //cv::imshow("raw", range);

	//opening(range, CV_8U, 5);

	dilateImg(range, CV_8U, 1);
	erodeImg(range, CV_8U, 1);
	erodeImg(range, CV_8U, 1);
	dilateImg(range, CV_8U, 1);
	dilateImg(range, CV_8U, 1);
	dilateImg(range, CV_8U, 1);
	erodeImg(range, CV_8U, 1);
	erodeImg(range, CV_8U, 1);
	erodeImg(range, CV_8U, 1);
	erodeImg(range, CV_8U, 1);
	erodeImg(range, CV_8U, 1);
	dilateImg(range, CV_8U, 1);

	//cv::imshow("hej", range);
	//opening(range, CV_8U, 5);
	//opening(range, CV_8U, 5);
	//closing(range, CV_8U, 5);
	//closing(range, CV_8U, 5);

	if (replace) {
		for (int x = 0; x < img.cols; x++) {
			for (int y = 0; y < img.rows; y++) {
				if (range.at<uchar>(y, x) == 0) {
					img.at<cv::Vec3b>(y, x)[0] = 0;
					img.at<cv::Vec3b>(y, x)[1] = 0;
					img.at<cv::Vec3b>(y, x)[2] = 0;
				}
			}
		}
	}

}

/// initializations for kalmanfilters
cv::Mat covariancesGlobal, observationErrorsGlobal;
float vx = 0, vy = 0, vz = 0;

/// in 3d
void updateKalmans(std::vector<KalmanFilter> &filters, cv::Point3f &position, double time, float minDistance, std::vector<cv::Point3f> &start, std::vector<cv::Point3f> &end)
{

	cv::Point3f prediction;
	cv::Mat state = cv::Mat::zeros(1, 3, CV_32F);
	cv::Mat fullState = cv::Mat::zeros(1, 6, CV_32F);
	bool found = false;
	int bestIndex;
	for (int n = 0; n < filters.size(); n++){
		if (filters[n].active) {
			continue;
		}
		prediction = filters[n].getPosition(time);
		state.at<float>(0,0) = position.x;
		state.at<float>(0,1) = position.y;
		state.at<float>(0,2) = position.z;



		if (abs(100*position.x - 100*prediction.x) + abs(100*position.y - 100*prediction.y) + abs(100*position.z - 100*prediction.z) < minDistance){
            bestIndex = n;
			minDistance = abs(100*position.x - 100*prediction.x) + abs(100*position.y - 100*prediction.y) + abs(100*position.z - 100*prediction.z);
			found = true;
		}
	}
	if (found){
		start.push_back(position);
		end.push_back(filters[bestIndex].getPosition(time));
		filters[bestIndex].update(state, time);
		filters[bestIndex].active = true;
	}else{
		fullState.at<float>(0,0) = position.x;
		fullState.at<float>(0,1) = position.y;
		fullState.at<float>(0,2) = position.z;
		fullState.at<float>(0,3) = vx;
		fullState.at<float>(0,4) = vy;
		fullState.at<float>(0,5) = vz;
		filters.push_back(KalmanFilter(6, time, fullState, covariancesGlobal, observationErrorsGlobal));

	}

}

/// in 3d
void createKalmans(std::vector<KalmanFilter> &filters, std::vector<cv::Point3f> &positions, double time)
{
	cv::Mat fullState = cv::Mat::zeros(1, 6, CV_32F);
	for (int n = 0; n < positions.size(); n++){
		fullState.at<float>(0,0) = positions[n].x;
		fullState.at<float>(0,1) = positions[n].y;
		fullState.at<float>(0,2) = positions[n].z;
		fullState.at<float>(0,3) = vx;
		fullState.at<float>(0,4) = vy;
		fullState.at<float>(0,5) = vz;

		filters.push_back(KalmanFilter(6, time, fullState, covariancesGlobal, observationErrorsGlobal));
	}
}

std::string makeMsg(double timestamp, std::vector<KalmanFilter> list)
{
    std::string msg = std::to_string(timestamp) + ":";
    cv::Mat state;

    for (int n = 0; n < list.size(); n++){
        state = list[n].stateVector;
        msg += "(";
        msg += std::to_string(state.at<float>(0,0)) + ",";
        msg += std::to_string(state.at<float>(0,1)) + ",";
        msg += std::to_string(state.at<float>(0,2)) + ",";
        msg += std::to_string(state.at<float>(0,3)) + ",";
        msg += std::to_string(state.at<float>(0,4)) + ",";
        msg += std::to_string(state.at<float>(0,5)) + ",";
        msg += ")";
    }

    return msg + ";";
}

void imageCallBack(const sensor_msgs::ImageConstPtr& leftImage, const sensor_msgs::ImageConstPtr& rightImage)
{
	//do stuff with images.
	double stamp_left = leftImage->header.stamp.toSec();
	double stamp_right = rightImage->header.stamp.toSec();

	double time = stamp_left;

	//ROS_INFO("\n CallBack called \n Timestamp %f \n ", time);


    //init stuff for publisher
    static ros::NodeHandle n; //handle for coordinates msgs

    static ros::Publisher pub = n.advertise<std_msgs::String>("coordinates", 1000);

    //static ros::Rate loop_rate(100);



    static bool doOnce = true;

	cv::Mat left, right;
	left = cv_bridge::toCvShare(leftImage, "bgr8")->image;
	//cv::imshow("left", inputImage);

	right = cv_bridge::toCvShare(rightImage, "bgr8")->image;
	//cv::imshow("right", inputImage);

    static int width = 1024, height = 768;

    static cv::Mat projectionLeft = cv::Mat::eye(3, 4, CV_64F), projectionRight = cv::Mat::eye(3, 4, CV_64F);
    static cv::Mat distCoeffsLeft = cv::Mat::zeros(1, 5, CV_64F), distCoeffsRight = cv::Mat::zeros(1, 5, CV_64F);
    static cv::Mat rotLeft = cv::Mat::zeros(1, 3, CV_64F);
    static cv::Mat transLeft = cv::Mat::zeros(1,3 , CV_64F);
    static cv::Mat camMatrixLeft = cv::Mat::eye(3, 3, CV_64F);


    // HSV constants
    static cv::Scalar yellowMin = cv::Scalar(0, 116, 28);
    static cv::Scalar yellowMax = cv::Scalar(52, 255, 255);

    //initilize variables

    static cv::Mat leftRange, rightRange, merged, disp, dispGray, leftGray, rightGray;
    static std::vector<cv::Point2f> pointsLeft, pointsRight;
    static cv::Mat triangulationPoints;
    static std::vector<cv::Point3f> points3d;
    static cv::Point3f point3d;

    static std::vector<KalmanFilter> filters;
    static std::vector<cv::Point3f> lineStarts, lineEnds;

    ////////// init do once ///////////
    if(doOnce){
        doOnce = false;

        covariancesGlobal = cv::Mat::eye(6, 6, CV_32F) * 0.01;
        observationErrorsGlobal = cv::Mat::eye(6, 6, CV_32F) * 0.5;

        projectionLeft.at<double>(0, 0) = 1322.97;
        projectionLeft.at<double>(1, 1) = 1325.18;
        projectionLeft.at<double>(0, 2) = 519.66;
        projectionLeft.at<double>(1, 2) = 411.72;

        //printMatrixDouble(projectionLeft);

        projectionRight.at<double>(0, 0) = 1321.39;
        projectionRight.at<double>(0, 1) = -5.52706;
        projectionRight.at<double>(0, 2) = 528.325;
        projectionRight.at<double>(0, 3) = -158.547;

        projectionRight.at<double>(1, 0) = -10.5154;
        projectionRight.at<double>(1, 1) = 1325.23;
        projectionRight.at<double>(1, 2) = 408.596;
        projectionRight.at<double>(1, 3) = 0.27565;

        projectionRight.at<double>(2, 0) = -0.01714;
        projectionRight.at<double>(2, 1) = -0.01786;
        projectionRight.at<double>(2, 2) = 0.99969;
        projectionRight.at<double>(2, 3) = 0.00257;

        //printMatrixDouble(projectionRight);


        distCoeffsLeft.at<double>(0, 0) = -0.48778;
        distCoeffsLeft.at<double>(0, 1) = 0.48369;
        distCoeffsLeft.at<double>(0, 2) = -0.00133;
        distCoeffsLeft.at<double>(0, 3) = 0.00022;

        distCoeffsRight.at<double>(0, 0) = -0.47735;
        distCoeffsRight.at<double>(0, 1) = 0.38745;
        distCoeffsRight.at<double>(0, 2) = 0.00041;
        distCoeffsRight.at<double>(0, 3) = 0.00180;


        camMatrixLeft.at<double>(0,0) = 1322.97;
        camMatrixLeft.at<double>(0,2) = 519.661;
        camMatrixLeft.at<double>(1,1) = 1325.18;
        camMatrixLeft.at<double>(1,2) = 411.717;

        //std::cout << "Finished initializing" << std::endl;
    }
    //////////////////////////

    //loadImages(time, left, right, "../few bricks");

    filterByHSV(left, leftRange, yellowMin, yellowMax);
    filterByHSV(right, rightRange, yellowMin, yellowMax);

    pointsLeft = getPoints(leftRange, 100);
    pointsRight = getPoints(rightRange, 100);


    if (pointsLeft.size() != 0)
    for (int n = 0; n < pointsLeft.size(); n++){
        cv::circle(left, pointsLeft[n], 5, cv::Scalar(255,0,0), 3);
    }

    if (pointsRight.size() != 0)
    for (int n = 0; n < pointsRight.size(); n++){
        cv::circle(right, pointsRight[n], 5, cv::Scalar(255,0,0), 3);
    }



	//cv::waitKey(0);

    if  (pointsLeft.size() == pointsRight.size() && pointsLeft.size() != 0) {
		cv::triangulatePoints(projectionLeft, projectionRight, pointsLeft, pointsRight, triangulationPoints);


		points3d.resize(0);

		//std::cout << "coordinates:" << std::endl;


		for (int n = 0; n < triangulationPoints.cols; n++) {
			point3d.x = triangulationPoints.at<float>(0, n) / triangulationPoints.at<float>(3, n);
			point3d.y = triangulationPoints.at<float>(1, n) / triangulationPoints.at<float>(3, n);
			point3d.z = triangulationPoints.at<float>(2, n) / triangulationPoints.at<float>(3, n);

			//std::cout << point3d.x << ", " << point3d.y << ", " << point3d.z << std::endl;

			points3d.push_back(point3d);
		}

		if (filters.size() > 0) {
			for (int n = 0; n < points3d.size(); n++) {
				updateKalmans(filters, points3d[n], time, 10, lineStarts, lineEnds);
			}
		} else {
			createKalmans(filters, points3d, time);
            std::cout << "CREATING KALMANS" << std::endl;
		}


		//std::cout << "Finished updating Kalmans:" << std::endl;

		//std::cout << "Number of Kalmans: " << filters.size() << std::endl;


	}

	for (int n = 0; n < filters.size(); n++) {
		if (time - filters[n].getTime() > 5) {
            std::cout << time - filters[n].getTime() << std::endl;
            std::cout << "Kalman died old" << std::endl;
            filters.erase(filters.begin() + n);
        } else if (!filters[n].active && filters[n].velocityCount < 5){
            std::cout << "Kalman died young" << std::endl;
            filters.erase(filters.begin() + n);
        } else {
            filters[n].active = false;
        }
		//else
			//std::cout << "time interval: " << time - filters[n].getTime() << std::endl;
	}

    //std::cout << "Finished filtering" << std::endl;

    /// Triangulation by http://answers.opencv.org/question/117141/triangulate-3d-points-from-a-stereo-camera-and-chessboard/


    std_msgs::String msg;
    std::string string = makeMsg(time,filters);
/*
    std::string string = std::to_string(0) + ":" +
            "(" + std::to_string(points3d[0].x) + "," + std::to_string(points3d[0].y) + "," + std::to_string(points3d[0].z) + ",0,0,0,)" +
            "(" + std::to_string(points3d[1].x) + "," + std::to_string(points3d[1].y) + "," + std::to_string(points3d[1].z) + ",0,0,0,)" +
            "(" + std::to_string(points3d[2].x) + "," + std::to_string(points3d[2].y) + "," + std::to_string(points3d[2].z) + ",0,0,0,)" +
            + " ;";
*/

     //std::cout << "lort: " << string << std::endl;
    msg.data = string;
    pub.publish(msg);
    //ros::spinOnce();

    //loop_rate.sleep();

    std::vector<cv::Point3f> filterPos;
    filterPos.resize(0);


    if (filters.size() > 0){
        for (int i = 0; i < filters.size(); i++) {
            filterPos.push_back(filters[i].getPosition(time));
        }

        std::vector<cv::Point2f> projectedPoints;
        projectedPoints.resize(0);
        if (filterPos.size() != 0) {
            cv::projectPoints(filterPos, rotLeft, transLeft, camMatrixLeft, distCoeffsLeft, projectedPoints);
            for (int n = 0; n < projectedPoints.size(); n++) {
                cv::circle(left, projectedPoints[n], 5, cv::Scalar(0, 255, 0), 3);
            }
        }

    }


    //std::cout << "left: " << pointsLeft.size() << std::endl;

    //std::cout << "right: " << pointsRight.size() << std::endl;

    cv::imshow("right range", rightRange);
    cv::imshow("left range", leftRange);
    cv::imshow("right", right);
    cv::imshow("left", left);

    cv::waitKey(1);

}



int main(int argc, char** argv) {

	ros::init(argc, argv, "vision_node");

	ros::NodeHandle n; //handle for raw pictures

	// define the subscribers

	message_filters::Subscriber<sensor_msgs::Image> image1_sub(n, "camera/left/image_color",5);
	message_filters::Subscriber<sensor_msgs::Image> image2_sub(n, "camera/right/image_color",5);

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub);


//start the callback
    sync.registerCallback(boost::bind(imageCallBack, _1, _2));

    ros::spin();



	/*int time = 0;

	while (time < 40) {

		loadImages(time, left, right, "../few bricks");

		filterByHSV(left, leftRange, yellowMin, yellowMax);
		filterByHSV(right, rightRange, yellowMin, yellowMax);

		pointsLeft = getPoints(leftRange, 25);
		pointsRight = getPoints(rightRange, 25);

		for (int n = 0; n < pointsLeft.size(); n++){
			cv::circle(left, pointsLeft[n], 5, cv::Scalar(255,0,0), 3);
		}

		for (int n = 0; n < pointsRight.size(); n++){
			cv::circle(right, pointsRight[n], 5, cv::Scalar(255,0,0), 3);
		}

		cv::imshow("left", left);
		cv::imshow("right", right);
		cv::imshow("left range", leftRange);
		cv::imshow("right range", rightRange);

		if  (pointsLeft.size() == pointsRight.size()){
			cv::triangulatePoints(projectionLeft, projectionRight, pointsLeft, pointsRight, triangulationPoints);
		}

		points3d.resize(0);

		std::cout << "coordinates:" << std::endl;


		for (int n = 0; n < triangulationPoints.cols; n++){
			point3d.x = triangulationPoints.at<float>(0,n) / triangulationPoints.at<float>(3,n);
			point3d.y = triangulationPoints.at<float>(1,n) / triangulationPoints.at<float>(3,n);
			point3d.z = triangulationPoints.at<float>(2,n) / triangulationPoints.at<float>(3,n);

			//std::cout << point3d.x << ", " << point3d.y << ", " << point3d.z << std::endl;

			points3d.push_back(point3d);
		}

		if (filters.size() > 0)	{
			for(int n = 0; n < points3d.size(); n++){
				updateKalmans(filters, points3d[n], time, 25, lineStarts, lineEnds);
			}
		}else{
			createKalmans(filters, points3d, time);
		}

		for (int n = 0; n < filters.size(); n++){
			filters[n].active = false;
		}

		std::cout << "Finished updating Kalmans" << std::endl;

		std::cout << filters.size() << std::endl;

		printPosition(filters[0].getPosition(time));
		printPosition(filters[0].getPosition(time+1));

		cv::imshow("left", left);

		std::cout << "Finished filtering" << std::endl;

		/// Triangulation by http://answers.opencv.org/question/117141/triangulate-3d-points-from-a-stereo-camera-and-chessboard/

        std::cout << time << std::endl;

		cv::waitKey(50);
		time++;
	}*/

	return 1;
}
































