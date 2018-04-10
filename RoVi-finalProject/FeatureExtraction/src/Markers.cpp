//
// Created by tera on 12/9/17.
//

#include "Markers.h"
#include <string>

Markers::Markers() {

}

void Markers::getMarker(cv::Mat &mOutput, int iMarkerType, int iDif, int iNumber) {
    // Select the propper marker type
    string file = "../../Markers/";
    switch(iMarkerType){
        case(MARKER1):
            file += "marker_color/marker_color_";
            break;
        case(MARKER2A):
            file += "marker_thinline/marker_thinline_";
            break;
        case(MARKER2B):
            file += "marker_thikline/marker_thikline_";
            break;
        case(MARKER3):
            file += "marker_corny/marker_corny_";
            break;
        default:
            cout << "Invalid MarkerType value:" << iMarkerType;
            file += "marker_color";
            break;
    }

    // select the difficulty
    if(iDif == HARD) file += "hard_";

    // select the number
    if(iNumber <10) file +="0";
    file += std::to_string(iNumber);

    // add the file extention
    file += ".png";

    // Load image
    mOutput = cv::imread(file);

    if (mOutput.empty()) {
        std::cout << "Input image not found at '" << file << "'\n";
        return;
    }

    return;


}

void Markers::getReference(Mat& mOutput, int iMarkerType) {
    string file = "../../Markers/markers/";
    switch(iMarkerType){
        case(MARKER1):
            file += "Marker1.ppm";
            break;
        case(MARKER2A):
            file += "Marker2a.ppm";
            break;
        case(MARKER2B):
            file += "Marker2b.ppm";
            break;
        case(MARKER3):
            file += "Marker3.ppm";
            break;
        default:
            cout << "Invalid MarkerType value:" << iMarkerType << "returns marker 1";
            file += "Marker1.ppm";
            break;
    }

    mOutput = cv::imread(file);
    cout << "test";
    if (mOutput.empty()) {
        std::cout << "Input image not found at '" << file << "'\n";
        return;
    }

    return;

}
