/*
 * RoVi1
 * Example application to load and display an image
 */

// v1.0-4-gdfe246a

#include <opencv2/opencv.hpp>
#include <iostream>
#include "Markers.h"

// sift feachures

#include "opencv2/xfeatures2d.hpp"

using namespace cv;
using namespace std;


int main()
{
    Mat img;
    Markers NewMarker;

    // Display all the markers
    /*
    for(int i = 1; i<=52; i++) {
        NewMarker.getMarker(img, MARKER3, HARD, i);

        // Show the image
        cv::imshow("Image" + to_string(i), img);
        cv::waitKey();
    }*/

    // Get the marker true marker
    Mat Marker;
    NewMarker.getReference(Marker, MARKER3);

    // Show the image
    cv::imshow("Marker3", Marker);
    cv::waitKey();


    // extract the feachures of the marker
    // Load images
    std::string filename1 = parser.get<std::string>("@image1");
    std::string filename2 = parser.get<std::string>("@image2");
    cv::Mat img1 = cv::imread(filename1);
    cv::Mat img2 = cv::imread(filename2);

    if (img1.empty()) {
        std::cout << "Input image 1 not found at '" << filename1 << "'\n";
        return 1;
    }

    if (img2.empty()) {
        std::cout << "Input image 2 not found at '" << filename2 << "'\n";
        return 1;
    }

    // Construct detector
    cv::Ptr<cv::Feature2D> detector;
    std::string features_type = parser.get<std::string>("features");

    if (features_type == "sift") {
        detector = cv::xfeatures2d::SIFT::create();
    } else if (features_type == "surf") {
        detector = cv::xfeatures2d::SURF::create();
    } else {
        std::cout << "Unknown feature type '" << features_type << "'\n";
        return 1;
    }

    std::cout << "Feature type: " << features_type << std::endl;

    // 1. / 2. Detect keypoints and compute descriptors
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;
    detector->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
    detector->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

    // Draw and show keypoints
    cv::Mat img_keypoints1;
    cv::Mat img_keypoints2;
    cv::drawKeypoints(img1, keypoints1, img_keypoints1);
    cv::drawKeypoints(img2, keypoints2, img_keypoints2);
    cv::imshow("Keypoints 1", img_keypoints1);
    cv::imshow("Keypoints 2", img_keypoints2);

    // Construct matcher
    cv::Ptr<cv::DescriptorMatcher> matcher;
    std::string matcher_type = parser.get<std::string>("matcher");

    if (matcher_type == "bruteforce") {
        matcher = cv::BFMatcher::create();
    } else if (matcher_type == "flannbased") {
        matcher = cv::FlannBasedMatcher::create();
    } else {
        std::cout << "Unknown matcher type '" << matcher_type << "'\n";
        return 1;
    }

    std::cout << "Matcher type: " << matcher_type << std::endl;

    // 3. / 4. Match descriptor vectors
    std::vector<cv::DMatch> matches;
    matcher->match(descriptors1, descriptors2, matches);
    std::cout << "Number of matches: " << matches.size() << std::endl;

    // Draw and show matches
    cv::Mat img_matches;
    cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, img_matches);
    cv::imshow("Matches", img_matches);

    while (cv::waitKey() != 27)
        ;

    // extract the feachures of the image marker

    return 0;
}
