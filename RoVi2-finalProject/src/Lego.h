//
// Created by daniel on 4/11/18.
//

#ifndef OBJECTAVOIDANCE_LEGO_H
#define OBJECTAVOIDANCE_LEGO_H



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



class Lego {

public:
    Lego(rw::common::LogWriter &lout, double f):log(lout),focal(f){}
    Lego(rwlibs::opengl::RenderImage *_textureRender,rw::common::LogWriter &lout,double f):log(lout),focal(f){
        //setup texture render
        this->_textureRender = _textureRender;
        //lout << "I am here" << endl;
    }

private:

};


#endif //OBJECTAVOIDANCE_LEGO_H
