//
// Created by student on 12/10/17.
//

#include <opencv2/imgproc.hpp>
#include "FeatureExtraction.h"


/*FeatureExtraction::FeatureExtraction() {

    // Constrct the SURF detector
    int minHessian = 400;
    detector = SURF::create();
    detector->setHessianThreshold(minHessian);
}*/

FeatureExtraction::FeatureExtraction(int iHassanThreshhold,rw::common::LogWriter &log):log(log) {

    // Constrct the SURF detector
    detector = SURF::create();
    detector->setHessianThreshold(iHassanThreshhold);
}

void FeatureExtraction::setMarker(Mat aMarker) {
    detector->detectAndCompute( aMarker, Mat(), vKeyPointsMarker, mDescriptorsMaker);
    aMarker.copyTo(mMarker);
}

vector<Point2f> FeatureExtraction::matchfeachures(Mat mImage) {
    // solution based on source: https://docs.opencv.org/master/d5/d6f/tutorial_feature_flann_matcher.html

    //-- Setep 1 set up descriptors and keypoints for the target image
    vector<KeyPoint> vKeyPointImage;
    Mat mDescriptorImage;
    detector->detectAndCompute( mImage, Mat(), vKeyPointImage, mDescriptorImage);

    //-- Step 2: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match( mDescriptorsMaker, mDescriptorImage, matches );


    // draw matches - for debugging
    /*Mat img_matches5;
    drawMatches( mMarker, vKeyPointsMarker, mImage, vKeyPointImage, matches, img_matches5 );
    imshow("All matches", img_matches5);
    cv::waitKey();*/


    double max_dist = 0; double min_dist = 100;
    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < mDescriptorsMaker.rows; i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }
    /*printf("-- Max dist : %f \n", max_dist );
    printf("-- Min dist : %f \n", min_dist );*/
    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
    //-- or a small arbitary value ( 0.2 ) in the event that min_dist is very
    //-- small)
    //-- PS.- radiusMatch can also be used here.
    std::vector< DMatch > good_matches;
    for( int i = 0; i < mDescriptorsMaker.rows; i++ )
    { if( matches[i].distance <= max(2*min_dist, 0.15) )
        { good_matches.push_back( matches[i]); }
    }


    //-- Draw only "good" matches
    //Mat img_matches;
    /*drawMatches( mMarker, vKeyPointsMarker, mImage, vKeyPointImage,
                 good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );


    //-- Show detected matches
    imshow( "Good Matches", img_matches );
    waitKey(0);*/

    /*
    for( int i = 0; i < (int)matches.size(); i++ )
    {
        log << "-- Good Match [" << i <<  "] Keypoint 1: " << matches[i].queryIdx <<  "-- Keypoint 2: " << matches[i].trainIdx << " -- distance: " << matches[i].distance << endl;
    }

    cout << "**********************good matches************************" << endl;

    for( int i = 0; i < (int)good_matches.size(); i++ )
    {
        log << "-- Good Match [" << i <<  "] Keypoint 1: " << good_matches[i].queryIdx <<  "-- Keypoint 2: " << good_matches[i].trainIdx << " -- distance: " << good_matches[i].distance << endl;
    }*/


    // use the good matches to finde the homography
    // solution based on source: https://docs.opencv.org/2.4/doc/tutorials/features2d/feature_homography/feature_homography.html
    //-- Localize the object
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;

    //-- Get the keypoints from the good matches
    for( unsigned int i = 0; i < good_matches.size(); i++ )
    {
        obj.push_back( vKeyPointsMarker[ good_matches[i].queryIdx ].pt );
        scene.push_back( vKeyPointImage[ good_matches[i].trainIdx ].pt );
    }

    if (good_matches.size() < 10) return std::vector<Point2f>();
    Mat H = findHomography( obj, scene, CV_RANSAC );

    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0);
    obj_corners[1] = cvPoint( mMarker.cols, 0 );
    obj_corners[2] = cvPoint( mMarker.cols, mMarker.rows );
    obj_corners[3] = cvPoint( 0, mMarker.rows );
    std::vector<Point2f> scene_corners(4);

    perspectiveTransform( obj_corners, scene_corners, H);

    //-- Draw lines between the corners (the mapped object in the scene - image_2
    /*line( img_matches, scene_corners[0] + Point2f( mMarker.cols, 0), scene_corners[1] + Point2f( mMarker.cols, 0), Scalar(0, 255, 0), 4 );
    line( img_matches, scene_corners[1] + Point2f( mMarker.cols, 0), scene_corners[2] + Point2f( mMarker.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[2] + Point2f( mMarker.cols, 0), scene_corners[3] + Point2f( mMarker.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[3] + Point2f( mMarker.cols, 0), scene_corners[0] + Point2f( mMarker.cols, 0), Scalar( 0, 255, 0), 4 );

    //-- Show detected matches
    imshow( "Good Matches & Object detection", img_matches );
    waitKey(0);*/

    return scene_corners;

}
