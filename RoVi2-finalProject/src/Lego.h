//
// Created by daniel on 4/11/18.
//

#ifndef OBJECTAVOIDANCE_LEGO_H
#define OBJECTAVOIDANCE_LEGO_H

// ------------------- includes ----------------------------
#include <rw/rw.hpp>
#include <rws/RobWorkStudio.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <vector>

// --------------------  namespaces ----------------------------
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


// -----------------------   defines ----------------------------
#define NUM_OF_LEGOBRICKS   10
#define X_RANGE             0.58
#define Y_RANGE             0.15






class Lego {

public:
    Lego();
    void move(int iLegoID, double dx);
    void move(double dx);

    void removeFromView();
    void removeFromView(int iLegoID);
    vector<MovableFrame*> getFrames();
    MovableFrame* getFrame(int iLegoID);



private:
    vector<MovableFrame*> vLegoFrames;
};


#endif //OBJECTAVOIDANCE_LEGO_H
