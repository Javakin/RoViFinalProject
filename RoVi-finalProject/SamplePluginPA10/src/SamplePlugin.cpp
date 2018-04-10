#include "SamplePlugin.hpp"

#include <rws/RobWorkStudio.hpp>

#include <QPushButton>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/kinematics/MovableFrame.hpp>

#include <functional>
#include <string>




#define DELTA_T_MAR  1000       // period in ms
#define DELTA_T_BOT  1000         // period in ms
#define DELTA_T_SIM  0
#define DELTA_T_CAMPROCESSING   850
#define Z_COORDINAT  0.5        // the debth of the image in meters
#define FOCALLENGTH  823        // focal lehgth of the camera
#define POINTS       1
#define ENABLE_VI_SER   0
#define SEQUENCE     FASTSEQ
string PATH_TO_PROJECT = "/home/daniel/Desktop/RoViFinalProject/RoVi-finalProject/";




using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace rws;

using namespace cv;

using namespace std::placeholders;

SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png")),SURFObj(400,log().info())
{
	setupUi(this);

	_timer = new QTimer(this);
    _markerMover = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));
    connect(_markerMover, SIGNAL(timeout()), this, SLOT(markerTimer()));

	// now connect stuff from the ui component
	connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );


	Image textureImage(300,300,Image::GRAY,Image::Depth8U);
	_textureRender = new RenderImage(textureImage);

    // setup markerhandle
    myMarker = new Marker(_textureRender,log().info(),(double)FOCALLENGTH);
    myMarker->importPath(PATH_TO_PROJECT + SEQUENCE);


	Image bgImage(0,0,Image::GRAY,Image::Depth8U);
	_bgRender = new RenderImage(bgImage,2.5/1000.0);
	_framegrabber = NULL;
}

SamplePlugin::~SamplePlugin()
{
    delete _textureRender;
    delete _bgRender;
    delete myMarker;
	delete myViscServ;
}

void SamplePlugin::initialize() {
	log().info() << "INITALIZE" << "\n";

	getRobWorkStudio()->stateChangedEvent().add(std::bind(&SamplePlugin::stateChangedListener, this, _1), this);

	// Auto load workcell
    WorkCell::Ptr wc = WorkCellLoader::Factory::load(PATH_TO_PROJECT + "PA10WorkCell/ScenePA10RoVi1.wc.xml");
	getRobWorkStudio()->setWorkCell(wc);

	// Load Lena image
	Mat im, image;
    im = imread(PATH_TO_PROJECT + "SamplePluginPA10/src/lena.bmp", CV_LOAD_IMAGE_COLOR); // Read the file
	cvtColor(im, image, CV_BGR2RGB); // Switch the red and blue color channels
	if(! image.data ) {
		RW_THROW("Could not open or find the image: please modify the file path in the source code!");
	}
	QImage img(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888); // Create QImage from the OpenCV image
	_label->setPixmap(QPixmap::fromImage(img)); // Show the image at the label in the plugin
}

void SamplePlugin::open(WorkCell* workcell)
{
    log().info() << "OPEN" << "\n";
    _wc = workcell;
    _state = _wc->getDefaultState();
	myViscServ = new VisualServoing(log().info(), _wc, _state);


    log().info() << workcell->getFilename() << "\n";

    if (_wc != NULL) {
        // Add the texture render to this workcell if there is a frame for texture
        Frame* textureFrame = _wc->findFrame("MarkerTexture");
        myMarker->setMarker(_wc);
        myMarker->setState(&_state);
        if (textureFrame != NULL) {
            getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
        }


        // Add the background render to this workcell if there is a frame for texture
        Frame* bgFrame = _wc->findFrame("Background");
        if (bgFrame != NULL) {
            getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
        }

        // Create a GLFrameGrabber if there is a camera frame with a Camera property set
        Frame* cameraFrame = _wc->findFrame("CameraSim");
        if (cameraFrame != NULL) {
            if (cameraFrame->getPropertyMap().has("Camera")) {
                // Read the dimensions and field of view
                double fovy;
                int width,height;
                std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
                std::istringstream iss (camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabber = new GLFrameGrabber(width,height,fovy);
                SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
                _framegrabber->init(gldrawer);
            }
        }
    }

    // Load image
    Mat CvMarker = cv::imread(PATH_TO_PROJECT + MARKER3);

    if (CvMarker.empty()) {
        log().info() << "Input image not found at '" << PATH_TO_PROJECT + MARKER3 << "'\n";
        return;
    }

    // initialize vision part
    SURFObj.setMarker(CvMarker);
}

bool SamplePlugin::getPoints(Mat im){
    // return the three points to follow
    if(ENABLE_VI_SER == 0) {
        DetectedPoints = myMarker->getMarkerPoints(POINTS);
    }
    else {
        // get points from feachure extraction
        vector <Point2f> test = SURFObj.matchfeachures(im);
        if (test.size() == 4){
            for (int i = 0; i < 3; i++) {
                // Marker found - update Detected points
                DetectedPoints[i * 2] = test[i].x - im.cols/2;
                DetectedPoints[i * 2 + 1] = test[i].y - im.rows/2;
            }
        }else {
            // protocol for handling failed reads - do not move
            DetectedPoints = TargetPoints;
            return 0;
        }

    }
    return 1;
}

void SamplePlugin::close() {
    log().info() << "CLOSE" << "\n";

    // Stop the timer
    _timer->stop();
    // Remove the texture render
	Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
	}
	// Remove the background render
	Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
	}
	// Delete the old framegrabber
	if (_framegrabber != NULL) {
		delete _framegrabber;
	}
	_framegrabber = NULL;
	_wc = NULL;
}

Mat SamplePlugin::toOpenCVImage(const Image& img) {
	Mat res(img.getHeight(),img.getWidth(), CV_8UC3);
	res.data = (uchar*)img.getImageData();
	return res;
}

Mat SamplePlugin::takePicture() {
    Frame* cameraFrame = _wc->findFrame("CameraSim");
    _framegrabber->grab(cameraFrame, _state);
    const Image& image = _framegrabber->getImage();

    // Convert to OpenCV image
    Mat im = toOpenCVImage(image);
    Mat imflip;
    cv::flip(im, imflip, 0);
    imflip.copyTo(im);
    cvtColor(im, im, CV_RGB2BGR);


    // get the detected marker points
    getPoints( im );
    VelocityScrew6D<> imgPoints = DetectedPoints;


    for(unsigned int i = 0;i <POINTS; i++){
        cv::circle(imflip,Point(imgPoints[i*2] + imflip.cols/2,imgPoints[i*2+1] + imflip.rows/2),5,Scalar(0,0,255),3);
        cv::circle(imflip,Point(TargetPoints[i*2] + imflip.cols/2,TargetPoints[i*2+1] + imflip.rows/2),8,Scalar(255,0,255),3);

        //log().info() << "changed: " << imgPoints[i*2] + imflip.cols/2 << " " << imgPoints[i*2+1] + imflip.rows/2 << endl;

    }



    /*imshow("imflip: ", imflip);
    waitKey();*/
    // Show in QLabel
    QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
    QPixmap p = QPixmap::fromImage(img);
    unsigned int maxW = 400;
    unsigned int maxH = 800;
    _label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));

    // mark the targetpoints in green

    // mark the actual points in red

    return im;
}
void SamplePlugin::reset() {
    Image::Ptr image;


    myMarker->setImage(PATH_TO_PROJECT + MARKER3);

    image = ImageLoader::Factory::load(PATH_TO_PROJECT + "SamplePluginPA10/backgrounds/color2.ppm");
    _bgRender->setImage(*image);
    getRobWorkStudio()->updateAndRepaint();

    Device::Ptr device;
    device = _wc->findDevice("PA10");

    if (device == NULL) {
        log().info() << "read of device failed\n";
    }
    myMarker->moveMarker(0);
    device->setQ(Q(7,0,-0.65,0,1.80,0,0.42,0), _state);
    getRobWorkStudio()->setState(_state);

    getPoints(takePicture());
    VelocityScrew6D<> imgPoints = DetectedPoints;

    // get the detected marker points
    /*if(ENABLE_VI_SER == 0)   imgPoints = myMarker->getMarkerPoints(POINTS);
    else {
        // get points from feachure extraction
        vector<Point2f> test = SURFObj.matchfeachures(takePicture());

        for(int i = 0;i < 3; i++){
            imgPoints[i*2] = test[i].x;
            imgPoints[i*2+1] = test[i].y;
        }
    }*/


    // calculate the image jacobian
    if (POINTS == 3)    myViscServ->setImageJacobian(FOCALLENGTH, Z_COORDINAT, imgPoints);
    else myViscServ->setImageJacobian1(FOCALLENGTH, Z_COORDINAT, imgPoints);
    TargetPoints = imgPoints;
    takePicture();


}

void SamplePlugin::run() {
    /*if (!_markerMover->isActive())
        _markerMover->start(DELTA_T_MAR);
    else
        _markerMover->stop();
*/
    if (!_timer->isActive())
        _timer->start(DELTA_T_SIM);
    else
        _timer->stop();


}

void SamplePlugin::btnPressed() {
    QObject *obj = sender();

	if(obj==_btn0){
		log().info() << "Button 0\n";
		// Set a new texture (one pixel = 1 mm)
		reset();

	} else if(obj==_btn1){
        log().info() << "Button 1\n";
		// Toggle the timer on and offs
        run();
	}
}

void SamplePlugin::timer() {

    if(!myMarker->moveMarker()){
        //_markerMover->stop();
        //_timer->stop();
        dT-=50;
        if(dT-DELTA_T_CAMPROCESSING> 0){
            reset();
            //run();
            log().info() << endl << endl<< dT << endl;
        }else{
            dT = 1000;
            _timer->stop();
            log().info() << endl <<"I AM DONE" << endl;


        }


    }

    if (_framegrabber != NULL) {
        // Get the image as a RW image
        Mat img = takePicture();

        // Update the detected points
        getPoints(img);
        VelocityScrew6D<> imgPoints = DetectedPoints;

        /*if(ENABLE_VI_SER == 0)   imgPoints = myMarker->getMarkerPoints(POINTS);
        else {
            // get points from feachure extraction
            vector<Point2f> test = SURFObj.matchfeachures(img);

            for(int i = 0;i < 3; i++){
                imgPoints[i*2] = test[i].x;
                imgPoints[i*2+1] = test[i].y;
            }
        }*/

        Q next;
        if(POINTS ==3) next= myViscServ->nextQ(imgPoints, dT-DELTA_T_CAMPROCESSING);
        else next= myViscServ->nextQ1(imgPoints, dT-DELTA_T_CAMPROCESSING);

        // calculate eucleadian distance
        //VelocityScrew6D<> error_dist= (TargetPoints-imgPoints);

        //log().info() << error_dist.norm2() << ", ";
        // setup devise

        Device::Ptr device;
        device = _wc->findDevice("PA10");

        if (device == NULL) {
            log().info() << "read of device failed\n";
        }

        device->setQ(next, _state);
        getRobWorkStudio()->setState(_state);

        log().info() << "" << next[0];
        for (unsigned int i = 1; i< 7; i++){
            log().info() << ", "<< next[i];
        }
        Frame *cameraFrame = _wc->findFrame("Camera");
        Transform3D<> baseToTool = cameraFrame->wTf(_state);

                //device->baseTframe(cameraFrame, _state);
        VelocityScrew6D<> toolPose(baseToTool);
        for (unsigned int i = 0; i< 6; i++){
            log().info() << ", "<< toolPose[i];
        }


        log().info() << ";" << endl;

        /*getPoints(takePicture());
        VelocityScrew6D<> error_dist= (TargetPoints-DetectedPoints);
        log().info() << error_dist.norm2() << ", ";*/
    }





}

void SamplePlugin::markerTimer(){

    getRobWorkStudio()->setState(_state);

    //log().info() << myMarker->getPosition().P() << endl;
}

void SamplePlugin::stateChangedListener(const State& state) {
  _state = state;
}
