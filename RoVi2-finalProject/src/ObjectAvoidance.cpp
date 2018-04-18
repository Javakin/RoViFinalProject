#include "ObjectAvoidance.hpp"



#define DELTA_T_SIM    50     // time in miliseconds

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

    std::cout << "Test" << std::endl;

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
        cout << "Setting up LegoHandler\n";
        LegoHandle = new Lego(&_state, _workcell);
        getRobWorkStudio()->setState(_state);

        cout << "initialize\n";
        //for simulation make a lego brick configuration
        LegoHandle->initializeTestSetup();
        getRobWorkStudio()->setState(_state);

        // Setting up the path planner
        PlannerHandle  = new Planning(&_state, _workcell);

        // Setting up the robotHandler
        RobotHandle = new Robot(&_state, _workcell);
    }


    capture();



}

void ObjectAvoidance::run(){
    // start the timer

    // Stop the timer
    if(_timer->isActive()){
        cout << "stopping the timer\n";
        _timer->stop();

    }else{
        cout << "stopping the timer\n";
        _timer->start(DELTA_T_SIM);

    }
    //_timer->start(DELTA_T_SIM);
    //_timer->stop();

}


void ObjectAvoidance::update(){
    LegoHandle->move(0.0015);
    getRobWorkStudio()->setState(_state);

    // the desired q valuse
    //Q q1 =  Q(6,3.291,-1.161,-4.915, 4.503, 4.712,4.513);
    //Q q2 =  Q(6,3.059,-2.037,-4.063, 4.529, 4.712,4.281);

}

void ObjectAvoidance::simpleMazeRunner() {
    // Initialize stuff for the run mode



}
