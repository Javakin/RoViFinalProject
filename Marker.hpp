


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



// OpenCV 3
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#ifndef MARKER
#define MARKER

#define MARKER1     "SamplePluginPA10/markers/Marker1.ppm"

#define MARKER2A    "SamplePluginPA10/markers/Marker2a.ppm"
#define MARKER2B    "SamplePluginPA10/markers/Marker2b.ppm"
#define MARKER3     "SamplePluginPA10/markers/Marker3.ppm"

#define SLOWSEQ     "SamplePluginPA10/motions/MarkerMotionSlow.txt"
#define MEDISEQ     "SamplePluginPA10/motions/MarkerMotionMedium.txt"
#define FASTSEQ     "SamplePluginPA10/motions/MarkerMotionFast.txt"



using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rw::math;

using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace std;
using namespace rws;
using namespace cv;

class Marker{
public:
    Marker(rw::common::LogWriter &lout, double f):log(lout),focal(f){}
    Marker(rwlibs::opengl::RenderImage *_textureRender,rw::common::LogWriter &lout,double f):log(lout),focal(f){
        //setup texture render
        this->_textureRender = _textureRender;
        //lout << "I am here" << endl;
    }
    void setImage(string sFilePath){
        // load and place marker image
        Image::Ptr image;
        image = ImageLoader::Factory::load(sFilePath);
        _textureRender->setImage(*image);

    }
    void setMarker(rw::models::WorkCell::Ptr _wc){
        MarkerFrame = _wc->findFrame<MovableFrame>("Marker");
        this->_wc = _wc;

    }
    void setState(State* worldState){
        _state = worldState;
    }


    bool moveMarker(){
        // move the market to the position given the position number
        if (MarkerPath.size()>iPosNum)
            MarkerFrame->setTransform(MarkerPath[iPosNum++], *_state);
        else {
            iPosNum = 0;
            return 0;
        }
        return 1;

    }
    void moveMarker(unsigned int iPosNum){
        // move the market to the position given the position number
        if (MarkerPath.size()>iPosNum)
        {
            MarkerFrame->setTransform(MarkerPath[iPosNum], *_state);
            this->iPosNum = iPosNum;
        }
    }

    void importPath(string aFilePath){
        // get the path form selected filepath and place it in the MarkerPath variable
        float num[6];
        ifstream myfile(aFilePath);
        if (myfile.is_open())
        {
            while (!myfile.eof()) {
                for (int i = 0; i < 6; i++) {
                    myfile >> num[i];
                    //log << num[i] << "\t";
                }
                //log << endl;
                Vector3D<> vec(num[0],num[1],num[2]);
                RPY<> ang(num[3],num[4],num[5]);
                MarkerPath.push_back(Transform3D<>(vec, ang.toRotation3D()));
            }
            myfile.close();
        }

        log << "importing path finished." << endl;
    }

    VelocityScrew6D<> getMarkerPoints(int points = 0){
        // return three points from the marker frame seen from the camera
        Frame* cam = _wc->findFrame("Camera");
        //Transform3D<> MarkerTCam = MarkerFrame->fTf(cam,*_state);
        Transform3D<> CamTMarker = cam->fTf(MarkerFrame,*_state);
        //Transform3D<> CamT = cam->wTf(*_state);
        //Transform3D<> MarkerT = MarkerFrame->wTf(*_state);



        //log << "Marker transform: " <<  MarkerT << endl << MarkerT.P() << endl;

        // generate 3 points in the marker frame
        Vector3D<> point0(0,0,0);
        Vector3D<> point1(0.15,0.15,0);
        Vector3D<> point2(-0.15,0.15,0);
        Vector3D<> point3(0.15,-0.15,0);

        //log << "test stufff: " <<  MarkerTCam*point0 << endl;

        // transform to camera frame
        if(points != 3){
            point0 = CamTMarker*point0;
        }else{
            point1 = CamTMarker*point1;
            point2 = CamTMarker*point2;
            point3 = CamTMarker*point3;

        }

        // transform to camera frame
        VelocityScrew6D<> imgPoints;
        if(points != 3){
            imgPoints[0] = focal*point0[0]/point0[2];
            imgPoints[1] = focal*point0[1]/point0[2];
        }else{
            imgPoints[0] = focal*point1[0]/point1[2];
            imgPoints[1] = focal*point1[1]/point1[2];
            imgPoints[2] = focal*point2[0]/point2[2];
            imgPoints[3] = focal*point2[1]/point2[2];
            imgPoints[4] = focal*point3[0]/point3[2];
            imgPoints[5] = focal*point3[1]/point3[2];

        }
        return imgPoints;

    }

    Transform3D<> getPosition(){
        // return the imeage transform
        return MarkerPath[iPosNum];
    }


private:
    rwlibs::opengl::RenderImage *_textureRender;
    vector<Transform3D<> > MarkerPath;
    MovableFrame* MarkerFrame;
    rw::kinematics::State* _state;
    unsigned int iPosNum;
    rw::common::LogWriter &log;
    rw::models::WorkCell::Ptr _wc;
    double focal;

};

#endif // MARKER













