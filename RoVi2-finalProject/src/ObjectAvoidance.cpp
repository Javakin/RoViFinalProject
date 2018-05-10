#include "ObjectAvoidance.hpp"



#define DELTA_T_SIM    50     // time in miliseconds
#define WORKCELL_PATH "/home/daniel/catkin_ws/src/RoViFinalProject/WorkStation_3/WC3_Scene.wc.xml"

ObjectAvoidance::ObjectAvoidance():
        RobWorkStudioPlugin("ObjectAvoidance", QIcon(":/pa_icon.png"))
{
    // initialize ros to start without running roslaunch or rosrun
    char** argv = NULL;
    int argc = 0;
    ros::init(argc, argv,"object_avoidance");

    // setpu GUI
    QScrollArea *widg = new QScrollArea(this);
    widg->setWidgetResizable(true);
    QWidget *dockWidgetContent = new QWidget(this);
    QVBoxLayout *verticalLayout = new QVBoxLayout(dockWidgetContent);


    //create stuff we want in the plug in here
    _markerButtons = createModeButtons();
    _cams = createCamSetup();
    _initButton = new QPushButton("Init");
    _runButton = new QPushButton("Run");

    // setting up the timer
    _timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(update()));



    //add them to the layout here
    verticalLayout->addWidget(_initButton);
    verticalLayout->addWidget(_markerButtons);
    verticalLayout->addWidget(_runButton);
    verticalLayout->addWidget(_cams);
    verticalLayout->addStretch(0);

    //setup the layouts.
    dockWidgetContent->setLayout(verticalLayout);
    widg->setWidget(dockWidgetContent);
    this->setWidget(widg);

    connect(_initButton, SIGNAL(pressed()), this, SLOT(init()));
    connect(_runButton, SIGNAL(pressed()), this, SLOT(run()));

}

ObjectAvoidance::~ObjectAvoidance(){
    delete LegoHandle;
}

void ObjectAvoidance::initialize(){
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&ObjectAvoidance::stateChangedListener, this, _1), this);
    _framegrabberLeft = NULL;
    _framegrabberRigth = NULL;
    LegoHandle = NULL;
    PlannerHandle = NULL;
    RobotHandle = NULL;
    VisionHandle = NULL;

    // Auto load workcell
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(WORKCELL_PATH);
    std::cout << "tried to load workcell" << std::endl;
    getRobWorkStudio()->setWorkCell(wc);

}

void ObjectAvoidance::open(rw::models::WorkCell* workcell) {
    _workcell = workcell;
    _state = _workcell->getDefaultState();

}

void ObjectAvoidance::close(){

    // Stop the timer
    //if(_timer->isActive())
    //    _timer->stop();


    // Delete the old framegrabber
    if (_framegrabberLeft != NULL && _framegrabberRigth != NULL) {
        delete _framegrabberLeft;
        delete _framegrabberRigth;
    }
    _framegrabberLeft = NULL;
    _framegrabberRigth = NULL;



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
    btns[0]->setToolTip("Mode 1");
    btns[0]->setIconSize(QSize(64,64));

    btns[1] = new QToolButton();
    btns[1]->setIcon(QIcon(":icons/mode2.png"));
    btns[1]->setToolTip("Mode 2");
    btns[1]->setIconSize(QSize(64,64));

    btns[2] = new QToolButton();
    btns[2]->setIcon(QIcon(":icons/mode3.png"));
    btns[2]->setToolTip("Mode 3");
    btns[2]->setIconSize(QSize(64,64));



    layout->addWidget(btns[0],0,0);
    layout->addWidget(btns[1],0,1);
    layout->addWidget(btns[2],0,2);


    connect(btns[0], SIGNAL(pressed()), this, SLOT(simpleMazeRunner()));
    connect(btns[1], SIGNAL(pressed()), this, SLOT(printConfig()));
    connect(btns[2], SIGNAL(pressed()), this, SLOT(moveHome()));

    markerButtons->setLayout(layout);
    widg->setWidget(markerButtons);
    widg->setSizePolicy(QSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed));
    return widg;
}

QWidget* ObjectAvoidance::createCamSetup() {

    QWidget* cams = new QWidget();
    QHBoxLayout *layout = new QHBoxLayout(cams);

    _leftCam = new QLabel();
    _rightCam = new QLabel();

    layout->addWidget(_leftCam);
    layout->addWidget(_rightCam);

    cams->setLayout(layout);
    cams->setSizePolicy(QSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed));

    return cams;
}



void ObjectAvoidance::init() {

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

        // Setting up legoHandler object
        LegoHandle = new Lego(&_state, _workcell);
        getRobWorkStudio()->setState(_state);


        //for simulation make a lego brick configuration
        LegoHandle->initializeTestSetup();
        getRobWorkStudio()->setState(_state);


        // Setting up the robotHandler
        RobotHandle = new Robot(&_state, _workcell);


        // Setting up the path planner
        PlannerHandle  = new Planning(_workcell, &_state, RobotHandle);


        // move robot to start configuration
        Q qGoal = Q(6,0.584, -1.065,-2.145, -1.504, 1.571, 1.805);
        Q qRobot =  RobotHandle->getQRobot();

        rw::trajectory::QPath aPath = PlannerHandle->RRTC(_state, qRobot, qGoal, 0.01);

        RobotHandle->setPath(aPath);

        RobotHandle->start();

        while (!RobotHandle->update()){
            getRobWorkStudio()->setState(_state);
        }


        PlannerHandle->start();


        // setting up vision
        VisionHandle = new Vision(LegoHandle);
        VisionHandle->start();

        getRobWorkStudio()->setState(_state);

    }


    capture();

}



void ObjectAvoidance::run(){
    // this button is for tuggling the timer
    if(_timer->isActive()){
        cout << "stopping the timer\n";
        _timer->stop();
        PlannerHandle->pausePlanner(1);

    }else{
        cout << "starting the timer\n";
        _timer->start(DELTA_T_SIM);
        PlannerHandle->pausePlanner(0);
    }

}

void ObjectAvoidance::update(){

    // update workspace
    //LegoHandle->move(0.003);

    //RobotHandle->update();

    // check for errors in the tree
    //QPath robotpath = PlannerHandle->validate(VALIDAITON_DEPTH);

    //PlannerHandle->printTree(_state);

    /*if(robotpath.size() != 0){
        // collision detected
        RobotHandle->setPath(robotpath);
        cout << "Collision detected\n";

        robotpath = PlannerHandle->repareTree();
        RobotHandle->setPath(robotpath);
    }*/

    // Update the workcell with the new state
    getRobWorkStudio()->setState(_state);
}



void ObjectAvoidance::simpleMazeRunner() {
    cout << "no code for simple maze runner function\n";

}

void ObjectAvoidance::printConfig() {
    // do not use the robot clase since this isnt nesesary up to date
    Device::Ptr device = _workcell->findDevice("UR1");
    cout << device->getQ(_state) << endl;
}

void ObjectAvoidance::moveHome() {
    RobotHandle->moveHome();

}
