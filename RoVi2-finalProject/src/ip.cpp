#include "ip.h"


cv::Mat ip::segmentateHSV(cv::Mat image, int hueMin, int hueMax, int saturationMin, int saturationMax, int valueMin, int valueMax){

    //convert to HSV
    cv::Mat HSV(image.rows, image.cols, CV_8UC3);
    cv::cvtColor(image, HSV, CV_BGR2HSV);

    cv::Mat dst;

    inRange(HSV, cv::Scalar(hueMin,saturationMin,valueMin), cv::Scalar(hueMax,saturationMax,valueMax), dst);

    return dst;
}

cv::Mat ip::opening(cv::Mat image, int elementType, int kernelSize){
    //elementType 0 = MORPTH_RECT, 1 = MORPH_CROSS, 2 = MORPH_ELLIPSE
    cv::Mat dst;
    cv::Mat element = cv::getStructuringElement(elementType,
                      cv::Size(2*kernelSize+1, 2*kernelSize+1),
                      cv::Point(kernelSize,kernelSize));

    cv::erode(image,dst,element);
    cv::dilate(dst, dst, element);


    return dst;
}

cv::Mat ip::closing(cv::Mat image, int elementType, int kernelSize){
    //elementType 0 = MORPTH_RECT, 1 = MORPH_CROSS, 2 = MORPH_ELLIPSE
    cv::Mat dst;
    cv::Mat element = cv::getStructuringElement(elementType,
                      cv::Size(2*kernelSize+1, 2*kernelSize+1),
                      cv::Point(kernelSize,kernelSize));

    cv::dilate(image,dst,element);
    cv::erode(dst,dst,element);


    return dst;
}

std::vector<std::vector<cv::Point> > ip::findContours(cv::Mat image){

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(image,contours,cv::RETR_TREE, cv::CHAIN_APPROX_NONE );

    return contours;
}


cv::Mat ip::drawContours(std::vector<std::vector<cv::Point> > contours, cv::Mat image){

    cv::Mat contourImg(image.size(), CV_8UC3, cv::Scalar(0,0,0));
    cv::Scalar colors[3];
    colors[0] = cv::Scalar(255,0,0);
    colors[1] = cv::Scalar(0,255,0);
    colors[2] = cv::Scalar(0,0,255);

    for (size_t i = 0; i < contours.size(); i++) {
        cv::drawContours(contourImg, contours, i, colors[i % 3]);
    }

    return contourImg;
}

std::vector<cv::Moments> ip::getMu(std::vector<std::vector<cv::Point> > contours ){
    std::vector<cv::Moments> mu(contours.size() );
    for (int i = 0; i < contours.size(); i++) {
        mu[i] = cv::moments(contours[i],false);
    }
    return mu;
}

std::vector<cv::Point2i> ip::getCenterPoints(std::vector<cv::Moments> moments, std::vector<std::vector<cv::Point> > contours){
    std::vector<cv::Point2i> centerPoints(contours.size());
    for (int i = 0; i < contours.size(); i++) {
        centerPoints[i] = cv::Point2f(moments[i].m10/moments[i].m00, moments[i].m01/moments[i].m00);
    }
    return centerPoints;
}


cv::Mat ip::drawPoints(std::vector<cv::Point2i> points, cv::Mat image, cv::Vec3b color){

    cv::Mat dst = image.clone();

    for (int i = 0; i < points.size(); i++) {
        dst.at<cv::Vec3b>(points[i]) = color;
    }

    return dst;
}

std::vector<cv::Point2i> ip::toRobotPoints(std::vector<cv::Point2i> points, cv::Mat image){

    cv::Point2i UV((image.cols)/2,(image.rows)/2);
    std::vector<cv::Point2i> newPoints = points;
    //std::cout << "x: " << UV.x << " y: " << UV.y << std::endl;
    for (int i = 0; i < points.size() ; i++) {
       // std::cout << "x: " << (UV-points[i]).x << " y: " << (UV-points[i]).y << std::endl;
        newPoints[i] = points[i]-UV;
        //newPoints[i].x = -newPoints[i].x;
        //newPoints[i].y = -newPoints[i].y;
    }
    return newPoints;
}

std::vector<cv::Point2i> ip::decideOnBlueMarkers(std::vector<cv::Point2i> mcBlue, std::vector<cv::Point2i> mcRed) {

    if(mcRed.size() < 1 || mcBlue.size() < 3)
        return mcRed;

    cv::Vec2i vector1 = {mcBlue[0].x - mcRed[0].x,mcBlue[0].y - mcRed[0].y};
    cv::Vec2i vector2 = {mcBlue[1].x - mcRed[0].x,mcBlue[1].y - mcRed[0].y};
    cv::Vec2i vector3 = {mcBlue[2].x - mcRed[0].x,mcBlue[2].y - mcRed[0].y};

    std::vector<cv::Point2i> points;

    std::vector<int> dotProducts = {vector1.dot(vector2), vector1.dot(vector3),vector2.dot(vector3)};
    int idx = std::min_element(dotProducts.begin(),dotProducts.end())-dotProducts.begin();
    if(idx == 0){

        int crossProduct = vector1[0]*vector2[1] - vector1[1]*vector2[0];;
        if(crossProduct < 0){
            points = {mcBlue[0],mcBlue[1],mcBlue[2]};
        }
        else{
            points = {mcBlue[1],mcBlue[0],mcBlue[2]};
        }
    }
    else if(idx == 1){
        int crossProduct = vector1[0]*vector3[1] - vector1[1]*vector3[0];
        if(crossProduct < 0){
            points = {mcBlue[0],mcBlue[2],mcBlue[1]};
        }
        else{
            points = {mcBlue[2],mcBlue[0],mcBlue[1]};
        }
    }
    else if(idx == 2){
        int crossProduct = vector2[0]*vector3[1] - vector2[1]*vector3[0];
        if(crossProduct < 0){
            points = {mcBlue[1],mcBlue[2],mcBlue[0]};
        }
        else{
            points = {mcBlue[2],mcBlue[1],mcBlue[0]};
        }
    }
    return points;
}

cv::Mat ip::toOpenCVImage(const rw::sensor::Image& img) {
    cv::Mat res(img.getHeight(),img.getWidth(), CV_8UC3);
    res.data = (uchar*)img.getImageData();
    return res;
}

std::vector<cv::Point2i> ip::marker1Function(const rw::sensor::Image& image) {
// Convert to OpenCV image
    cv::Mat im = toOpenCVImage(image);
    cv::Mat imflip;
    cv::flip(im, imflip, 0);
    //imflip = im;
    cv::cvtColor(imflip, imflip, CV_RGB2BGR);

    cv::Mat segmentedBlue, segmentedRed;

//FIND BLUE
    segmentedBlue = segmentateHSV(imflip, 120, 121, 250, 256, 100, 256); //blue circles
    segmentedBlue = opening(segmentedBlue, 0, 3);
    segmentedBlue = closing(segmentedBlue, 0, 6);
    segmentedBlue = opening(segmentedBlue, 0, 14);

    std::vector<std::vector<cv::Point> > contoursBlue = findContours(segmentedBlue);

    //segmentedBlue = ip::drawContours(contoursBlue, segmentedBlue);

//Get the moments
    std::vector<cv::Moments> muBlue = getMu(contoursBlue);

//get the mass centers
    std::vector<cv::Point2i> mcBlue = getCenterPoints(muBlue, contoursBlue);

//FIND RED
    segmentedRed = segmentateHSV(imflip, 0, 1, 250, 256, 100, 256); //blue circles
    segmentedRed = opening(segmentedRed, 0, 3);
    segmentedRed = closing(segmentedRed, 0, 6);
    segmentedRed = opening(segmentedRed, 0, 14);

    std::vector<std::vector<cv::Point> > contoursRed = findContours(segmentedRed);

    //segmentedRed = ip::drawContours(contoursRed, segmentedRed);

//Get the moments
    std::vector<cv::Moments> muRed = getMu(contoursRed);

//get the mass centers
    std::vector<cv::Point2i> mcRed = getCenterPoints(muRed, contoursRed);

//segmentedRed = ip::drawPoints(mcRed, segmentedRed,cv::Vec3b(0,0,255));

//DO STUFF WITH RED AND BLUE
    //cv::Mat result = segmentedBlue + segmentedRed;

    std::vector<cv::Point2i> bluePoints = decideOnBlueMarkers(mcBlue, mcRed);

    if (bluePoints.size() < 3 || mcRed.size() < 1) {
        std::cout << "Did not find enough markers" << std::endl;
        std::vector<cv::Point> null;
        cv::Point2i p = {0,0};
        null.push_back(p);
        return null;
    }

    std::vector<cv::Point2i> allPoints;

    for (int i = 0; i < mcRed.size(); i++) {
        //cv::circle(result, mcRed[i], 10, cv::Vec3b(255, 0, 0), 4);
        allPoints.push_back(mcRed[i]);
    }
    for (int i = 0; i < bluePoints.size(); i++) {
        allPoints.push_back(bluePoints[i]);
        //cv::circle(result, bluePoints[i], 10, cv::Vec3b(0, 0, 255), 4);
    }

    allPoints = toRobotPoints(allPoints, imflip);

/* Show on QLabel
    QImage img(result.data, result.cols, result.rows, result.step, QImage::Format_RGB888);
    QPixmap p = QPixmap::fromImage(img);
    unsigned int maxW = 400;
    unsigned int maxH = 800;
    _processedPicture->setPixmap(p.scaled(maxW, maxH, Qt::KeepAspectRatio));
*/
    return allPoints;
}

std::vector<cv::Point2i> ip::marker3Function(const rw::sensor::Image& image, cv::Ptr<cv::xfeatures2d::SURF> _detector, cv::Mat _descriptors_object,std::vector<cv::KeyPoint> _keypoints_object,cv::Mat _img_object) {
    cv::Mat im = toOpenCVImage(image);
    cv::Mat imflip;
    cv::flip(im, imflip, 0);
    cv::cvtColor(imflip, imflip, CV_RGB2GRAY);

    cv::Mat img_scene = imflip;

    //cv::Mat img_object = _img_object.clone();

    if( !_img_object.data || !img_scene.data )
    { std::cout<< " --(!) Error reading images " << std::endl; return std::vector<cv::Point2i>(); }

    //-- Step 1: Detect the keypoints using SURF Detector

    std::vector<cv::KeyPoint> keypoints_scene;

    //-- Step 2: Calculate descriptors (feature vectors)
    cv::Mat descriptors_scene;

    _detector->detectAndCompute( img_scene, cv::Mat(), keypoints_scene, descriptors_scene );

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    cv::FlannBasedMatcher matcher;
    std::vector< cv::DMatch > matches;
    matcher.match( _descriptors_object, descriptors_scene, matches);

    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < _descriptors_object.rows; i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist )
            min_dist = dist;
        if( dist > max_dist )
            max_dist = dist;
    }
    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector< cv::DMatch > good_matches;

    for( int i = 0; i < _descriptors_object.rows; i++ )
    {
        if( matches[i].distance < 3*min_dist)
            good_matches.push_back( matches[i]);
    }

    //-- Localize the object
    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;
    for( int i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        obj.push_back( _keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }
    cv::Mat H = findHomography( obj, scene, cv::RANSAC );
    if(H.empty())
    {
        rw::common::Log::log().info() << "No good matches, no new points calculated" << std::endl;
        std::vector<cv::Point> null;
        cv::Point2i p = {0,0};
        null.push_back(p);
        return null;
    }
    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<cv::Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0);
    obj_corners[1] = cvPoint( _img_object.cols, 0 );
    obj_corners[2] = cvPoint( _img_object.cols, _img_object.rows );
    obj_corners[3] = cvPoint( 0, _img_object.rows );
    std::vector<cv::Point2f> scene_corners(4);

    perspectiveTransform( obj_corners, scene_corners, H);

    std::vector<cv::Point2i> points;
    for (int i = 0; i < scene_corners.size(); i++) {
        points.push_back(scene_corners[i]);
    }
    points = toRobotPoints(points, imflip);
/*
    for(int i = 0; i < points.size(); i++){
        rw::common::Log::log().info() << "Point: " << i << " " << points[i];
    }
    rw::common::Log::log().info() << std::endl;


    for (int i = 0; i < points.size(); i++) {
        cv::circle(img_scene,points[i],10,0,4);
    }

    // Show on QLabel
    QImage img(img_scene.data, img_scene.cols, img_scene.rows, img_scene.step, QImage::Format_Grayscale8);
    QPixmap p = QPixmap::fromImage(img);
    unsigned int maxW = 400;
    unsigned int maxH = 800;
    _processedPicture->setPixmap(p.scaled(maxW, maxH, Qt::KeepAspectRatio));
*/
    return points;
}