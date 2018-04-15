#pragma once

#ifndef OBJECTAVOIDANCE_HPP
#define OBJECTAVOIDANCE_HPP_HPP

// RobWork includes
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

// RobWorkStudio includes
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>
#include <rws/RobWorkStudio.hpp>

//OpenCV
#include <opencv2/opencv.hpp>


//QT
#include <QTimer>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QHBoxLayout>

// content from the cpp file

#include <boost/filesystem.hpp>
#include <boost/bind.hpp>

#include <QDialog>

#include <QtWidgets/QFileDialog>
#include <rw/kinematics.hpp>

//Constructed files
#include "ip.h"
#include "Lego.hpp"



class ObjectAvoidance: public rws::RobWorkStudioPlugin
{

Q_OBJECT
    Q_INTERFACES( rws::RobWorkStudioPlugin )
#if RWS_USE_QT5
    Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "ObjectAvoidance.json")
#endif

public:
    ObjectAvoidance();
    virtual ~ObjectAvoidance();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();



private:
    QWidget* createModeButtons();
    QWidget* createCamSetup();

    rw::kinematics::State _state;
    rw::models::WorkCell* _workcell;

    QTimer* _timer;
    QWidget* _markerButtons;
    QWidget* _cams;
    QLabel* _leftCam;
    QLabel* _rightCam;
    QPushButton* _initButton;
    QPushButton* _runButton;
    rwlibs::simulation::GLFrameGrabber* _framegrabberLeft;
    rwlibs::simulation::GLFrameGrabber* _framegrabberRigth;
    //Lego* LegoHandle;



private slots:
    void stateChangedListener(const rw::kinematics::State& state);
    void capture();
    void init();
    //void run();
    //void update();
};

#endif