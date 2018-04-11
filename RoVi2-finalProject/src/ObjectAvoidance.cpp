#include "ObjectAvoidance.hpp"

#include <boost/filesystem.hpp>
#include <boost/bind.hpp>

#include <QDialog>
#include <QtWidgets/QFileDialog>
#include <rw/kinematics.hpp>

#include "ip.h"



ObjectAvoidance::ObjectAvoidance():
        RobWorkStudioPlugin("ObjectAvoidance", QIcon(":/pa_icon.png"))
{
    QScrollArea *widg = new QScrollArea(this);
    widg->setWidgetResizable(true);
    QWidget *dockWidgetContent = new QWidget(this);
    QVBoxLayout *verticalLayout = new QVBoxLayout(dockWidgetContent);


    //create stuff we want in the plug in here
    _markerButtons = createModeButtons();
    _cams = createCamSetup();
    _initButton = new QPushButton("Init");


    //add them to the layout here
    verticalLayout->addWidget(_initButton);
    verticalLayout->addWidget(_markerButtons);
    verticalLayout->addWidget(_cams);
    verticalLayout->addStretch(0);

    //setup the layouts.
    dockWidgetContent->setLayout(verticalLayout);
    widg->setWidget(dockWidgetContent);
    this->setWidget(widg);

    connect(_initButton, SIGNAL(pressed()), this, SLOT(init()));

}

ObjectAvoidance::~ObjectAvoidance(){

}

void ObjectAvoidance::initialize(){
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&ObjectAvoidance::stateChangedListener, this, _1), this);
}

void ObjectAvoidance::open(rw::models::WorkCell* workcell) {
    _workcell = workcell;
    _state = _workcell->getDefaultState();
}

void ObjectAvoidance::close(){

    // Stop the timer
    //_timer->stop();

    // Delete the old framegrabber
    if (_framegrabberLeft != NULL && _framegrabberRigth != NULL) {
        delete _framegrabberLeft;
        delete _framegrabberRigth;
    }
    _framegrabberLeft = NULL;
    _framegrabberRigth = NULL;
    _workcell = NULL;
}

void ObjectAvoidance::capture() {

    if (_framegrabberLeft != NULL) {
        // Get the image as a RW image
        rw::kinematics::Frame* cameraFrame = _workcell->findFrame("CameraSimLeft");
        _framegrabberLeft->grab(cameraFrame, _state);
        const rw::sensor::Image& image = _framegrabberLeft->getImage();

        // Convert to OpenCV image
        cv::Mat im = ip::toOpenCVImage(image);
        cv::Mat imflip;
        cv::flip(im, imflip, 0);

        // Show on QLabel
        QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
        QPixmap p = QPixmap::fromImage(img);
        unsigned int maxW = 200;
        unsigned int maxH = 400;
        _leftCam->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
    }

    if (_framegrabberRigth != NULL) {
        // Get the image as a RW image
        rw::kinematics::Frame* cameraFrame = _workcell->findFrame("CameraSimRigth");
        _framegrabberRigth->grab(cameraFrame, _state);
        const rw::sensor::Image& image = _framegrabberRigth->getImage();

        // Convert to OpenCV image
        cv::Mat im = ip::toOpenCVImage(image);
        cv::Mat imflip;
        cv::flip(im, imflip, 0);

        // Show on QLabel
        QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
        QPixmap p = QPixmap::fromImage(img);
        unsigned int maxW = 200;
        unsigned int maxH = 400;
        _rightCam->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
    }
}

void ObjectAvoidance::stateChangedListener(const rw::kinematics::State &state){
    _state = state;
    capture();
}

QWidget* ObjectAvoidance::createModeButtons(){

    QScrollArea *widg = new QScrollArea();
    widg->setFrameShape(QFrame::NoFrame);
    QWidget* markerButtons = new QWidget();
    QGridLayout *layout = new QGridLayout(markerButtons);


    QToolButton *btns[3];

    btns[0] = new QToolButton();
    btns[0]->setIcon(QIcon(":icons/mode1.png"));
    btns[0]->setIconSize(QSize(64,64));

    btns[1] = new QToolButton();
    btns[1]->setIcon(QIcon(":icons/mode2.png"));
    btns[1]->setIconSize(QSize(64,64));

    btns[2] = new QToolButton();
    btns[2]->setIcon(QIcon(":icons/mode3.png"));
    btns[2]->setIconSize(QSize(64,64));



    layout->addWidget(btns[0],0,0);
    layout->addWidget(btns[1],0,1);
    layout->addWidget(btns[2],0,2);


    markerButtons->setLayout(layout);
    widg->setWidget(markerButtons);
    widg->setSizePolicy(QSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed));
    return widg;
}

QWidget* ObjectAvoidance::createCamSetup() {

    QWidget* cams = new QWidget();
    QGridLayout *layout = new QGridLayout(cams);

    _leftCam = new QLabel();
    _rightCam = new QLabel();

    layout->addWidget(_leftCam,0,0);
    layout->addWidget(_rightCam,0,1);

    cams->setLayout(layout);

    return cams;
}

void ObjectAvoidance::init() {

    _framegrabberLeft = NULL;
    _framegrabberRigth = NULL;

    if (_workcell != NULL) {
        // Create a GLFrameGrabber if there is a camera frame with a Camera property set
        rw::kinematics::Frame* cameraFrameLeft = _workcell->findFrame("CameraSimLeft");
        rw::kinematics::Frame* cameraFrameRight = _workcell->findFrame("CameraSimRigth");
        if (cameraFrameLeft != NULL) {
            if (cameraFrameLeft->getPropertyMap().has("Camera")) {
                // Read the dimensions and field of view
                double fovy;
                int width,height;
                std::string camParam = cameraFrameLeft->getPropertyMap().get<std::string>("Camera");
                std::istringstream iss (camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabberLeft = new rwlibs::simulation::GLFrameGrabber(width,height,fovy);
                rw::graphics::SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
                _framegrabberLeft->init(gldrawer);
            }
        }

        if (cameraFrameRight != NULL) {
            if (cameraFrameRight->getPropertyMap().has("Camera")) {
                // Read the dimensions and field of view
                double fovy;
                int width,height;
                std::string camParam = cameraFrameRight->getPropertyMap().get<std::string>("Camera");
                std::istringstream iss (camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabberRigth = new rwlibs::simulation::GLFrameGrabber(width,height,fovy);
                rw::graphics::SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
                _framegrabberRigth->init(gldrawer);
            }
        }

    }

    capture();
}



