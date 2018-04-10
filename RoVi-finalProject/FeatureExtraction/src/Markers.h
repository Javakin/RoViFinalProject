//
// Created by tera on 12/9/17.
//

#ifndef FEATUREEXTRACTION_MARKERS_H
#define FEATUREEXTRACTION_MARKERS_H

#define MARKER1     0
#define MARKER2A    1
#define MARKER2B    2
#define MARKER3     3

#define EASY        0
#define HARD        1


// includes
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

// namespaces
using namespace std;
using namespace cv;



class Markers {
public:
    Markers();
    void getMarker(cv::Mat& mOutput, int iMarkerType, int iDif, int iNumber);
    void getReference(Mat& mOutput, int iMarkerType);

};


#endif //FEATUREEXTRACTION_MARKERS_H
