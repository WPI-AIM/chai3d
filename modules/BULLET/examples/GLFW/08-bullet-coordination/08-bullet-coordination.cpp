//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.2.0 $Rev: 1869 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//---------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------
#include "CBullet.h"
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// GENERAL SETTINGS
//---------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled 
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//---------------------------------------------------------------------------
// BULLET MODULE VARIABLES
//---------------------------------------------------------------------------

// bullet world
cBulletWorld* bulletWorld;

// bullet objects
cBulletMesh* bulletMesh1;
cBulletMesh* bulletMesh2;
cBulletMesh* bulletTool;
cBulletMesh* bulletCylinder;
cBulletMultiMesh* bulletGear;
cBulletMultiMesh* bulletTorus;
cBulletMultiMesh* tool;

// bullet static walls and ground
cBulletStaticPlane* bulletInvisibleWall1;
cBulletStaticPlane* bulletInvisibleWall2;
cBulletStaticPlane* bulletInvisibleWall3;
cBulletStaticPlane* bulletInvisibleWall4;
cBulletStaticPlane* bulletInvisibleWall5;
cBulletStaticPlane* bulletGround;

// stiffness of virtual spring
double linGain = 0.05;
double angGain = 0.03;
double linG;
double angG;
double linStiffness = 1000;
double angStiffness = 30;

cVector3d camPos(0,0,0);
cVector3d dev_vel;
cMatrix3d cam_rot_last, dev_rot_last, dev_rot_cur;


//---------------------------------------------------------------------------
// CHAI3D VARIABLES
//---------------------------------------------------------------------------


// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cSpotLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
shared_ptr<cGenericHapticDevice> hapticDevice;

// workspace scale factor
double workspaceScaleFactor = 30.0;

// a label to display the rates [Hz] at which the simulation is running
cLabel* labelRates;


//---------------------------------------------------------------------------
// GENERAL VARIABLES
//---------------------------------------------------------------------------

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

// root resource path
string resourceRoot;


//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------

// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())

//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);

//===========================================================================
/*
    DEMO:    04-bullet-tool.cpp

    This example illustrates the use of the Bullet framework for simulating
    haptic interaction with dynamic bodies. In this scene we create 4
    cubic meshes that we individually attach to ODE bodies. One of the blocks
    is attached to the haptic device through a virtual spring.
 */
//===========================================================================

int main(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Demo: 04-bullet-tool" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[h] - Display help menu" << endl;
    cout << "[1] - Enable gravity" << endl;
    cout << "[2] - Disable gravity" << endl << endl;
    cout << "[3] - decrease linear haptic gain" << endl;
    cout << "[4] - increase linear haptic gain" << endl;
    cout << "[5] - decrease angular haptic gain" << endl;
    cout << "[6] - increase angular haptic gain" << endl  << endl;
    cout << "[7] - decrease linear stiffness" << endl;
    cout << "[8] - increase linear stiffness" << endl;
    cout << "[9] - decrease angular stiffness" << endl;
    cout << "[0] - increase angular stiffness" << endl << endl;
    cout << "[q] - Exit application\n" << endl;
    cout << endl << endl;


    //-----------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //-----------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 0.8 * mode->height;
    int h = 0.5 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // set active stereo mode
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    }
    else
    {
        glfwWindowHint(GLFW_STEREO, GL_FALSE);
    }

    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window, &width, &height);

    // set position of window
    glfwSetWindowPos(window, x, y);

    // set key callback
    glfwSetKeyCallback(window, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);

    // initialize GLEW library
#ifdef GLEW_VERSION
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif


    //-----------------------------------------------------------------------
    // 3D - SCENEGRAPH
    //-----------------------------------------------------------------------

    // create a dynamic world.
    bulletWorld = new cBulletWorld();

    // set the background color of the environment
    bulletWorld->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    camera = new cCamera(bulletWorld);
    bulletWorld->addChild(camera);

    // position and orient the camera
    camera->set(cVector3d(2.5, 0.0, 0.3),    // camera position (eye)
                cVector3d(0.0, 0.0,-0.5),    // lookat position (target)
                cVector3d(0.0, 0.0, 1.0));   // direction of the "up" vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.02);
    camera->setStereoFocalLength(2.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a light source
    light = new cSpotLight(bulletWorld);

    // attach light to camera
    bulletWorld->addChild(light);

    // enable light source
    light->setEnabled(true);

    // position the light source
    light->setLocalPos( 0, 0, 1.2);

    // define the direction of the light beam
    light->setDir(0,0,-1.0);

    // set uniform concentration level of light 
    light->setSpotExponent(0.0);

    // enable this light source to generate shadows
    light->setShadowMapEnabled(true);

    // set the resolution of the shadow map
    light->m_shadowMap->setQualityLow();
    //light->m_shadowMap->setQualityMedium();

    // set light cone half angle
    light->setCutOffAngleDeg(45);


    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFontPtr font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    labelRates->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelRates);


    //////////////////////////////////////////////////////////////////////////
    // BULLET WORLD
    //////////////////////////////////////////////////////////////////////////
    // stiffness properties
    double maxStiffness	= hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

    // clamp the force output gain to the max device stiffness
    linGain = cMin(linGain, maxStiffness / linStiffness);

    // set some gravity
    bulletWorld->setGravity(cVector3d(0.0, 0.0, -9.8));


    //////////////////////////////////////////////////////////////////////////
    // 3 BULLET BLOCKS
    //////////////////////////////////////////////////////////////////////////
    double size = 0.40;

    bulletGear = new cBulletMultiMesh(bulletWorld);
    bulletGear->loadFromFile(RESOURCE_PATH("../resources/models/gear/gear.3ds"));
    bulletGear->scale(0.0014);
    bulletWorld->addChild(bulletGear);
    bulletGear->buildContactTriangles(0.001);
    bulletGear->setMass(0.3);
    bulletGear->estimateInertia();
    bulletGear->buildDynamicModel();

    bulletTorus = new cBulletMultiMesh(bulletWorld);
    bulletTorus->loadFromFile(RESOURCE_PATH("../resources/models/gear/torus.3ds"));
    bulletTorus->scale(0.2);
    bulletTorus->setLocalPos(cVector3d(0.3,0,0));
    bulletWorld->addChild(bulletTorus);
    bulletTorus->buildContactTriangles(0.001);
    bulletTorus->setMass(0.8);
    bulletTorus->estimateInertia();
    bulletTorus->buildDynamicModel();


    //////////////////////////////////////////////////////////////////////////
    // INVISIBLE WALLS
    //////////////////////////////////////////////////////////////////////////

    // we create 5 static walls to contain the dynamic objects within a limited workspace
    double planeWidth = 1.0;
    bulletInvisibleWall1 = new cBulletStaticPlane(bulletWorld, cVector3d(0.0, 0.0, -1.0), -2.0 * planeWidth);
    bulletInvisibleWall2 = new cBulletStaticPlane(bulletWorld, cVector3d(0.0, -1.0, 0.0), -1.5*planeWidth);
    bulletInvisibleWall3 = new cBulletStaticPlane(bulletWorld, cVector3d(0.0, 1.0, 0.0), -1.5*planeWidth);
    bulletInvisibleWall4 = new cBulletStaticPlane(bulletWorld, cVector3d(-1.0, 0.0, 0.0), -planeWidth);
    bulletInvisibleWall5 = new cBulletStaticPlane(bulletWorld, cVector3d(1.0, 0.0, 0.0), -0.8 * planeWidth);


    //////////////////////////////////////////////////////////////////////////
    // GROUND
    //////////////////////////////////////////////////////////////////////////

    // create ground plane
    bulletGround = new cBulletStaticPlane(bulletWorld, cVector3d(0.0, 0.0, 1.0), -planeWidth);

    // add plane to world as we will want to make it visibe
    bulletWorld->addChild(bulletGround);

    // create a mesh plane where the static plane is located
    cCreatePlane(bulletGround, 3.0, 3.0, bulletGround->getPlaneConstant() * bulletGround->getPlaneNormal());

    // define some material properties and apply to mesh
    cMaterial matGround;
    matGround.setBlueDodger();
    matGround.m_emission.setGrayLevel(0.3);
    bulletGround->setMaterial(matGround);


    //////////////////////////////////////////////////////////////////////////
    // TOOL
    //////////////////////////////////////////////////////////////////////////

    // create three objects that are added to the world
    //bulletTool = new cBulletCylinder(bulletWorld, 2.0 * size, 0.2 * size);
    cMatrix3d rot;
    // assign linear and angular damping

    tool = new cBulletMultiMesh(bulletWorld);
    tool->loadFromFile(RESOURCE_PATH("../resources/models/gear/hook.3ds"));
    tool->scale(1);
    rot.setAxisAngleRotationDeg(0,1,0,-90);
    bulletWorld->addChild(tool);

    // define some material properties for the axes
    cMaterial matAxis;
//    matAxis.setStiffness(0.5 * maxStiffness);
//    matAxis.setDynamicFriction(0.7);
//    matAxis.setStaticFriction(0.5);
    matAxis.setRedDarkSalmon();

    tool->setMaterial(matAxis, true);

    tool->setMass(0.03);
    tool->buildContactTriangles(0.01);
    tool->estimateInertia();
    tool->buildDynamicModel();
    tool->setDamping(1.0,1.0);



    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}

//---------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width = a_width;
    height = a_height;
}

//---------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//---------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // option - toggle fullscreen
    else if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M)
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }

    // option - help menu
    else if (a_key == GLFW_KEY_H)
    {
        cout << "Keyboard Options:" << endl << endl;
        cout << "[h] - Display help menu" << endl;
        cout << "[1] - Enable gravity" << endl;
        cout << "[2] - Disable gravity" << endl << endl;
        cout << "[3] - decrease linear haptic gain" << endl;
        cout << "[4] - increase linear haptic gain" << endl;
        cout << "[5] - decrease angular haptic gain" << endl;
        cout << "[6] - increase angular haptic gain" << endl << endl;
        cout << "[7] - decrease linear stiffness" << endl;
        cout << "[8] - increase linear stiffness" << endl;
        cout << "[9] - decrease angular stiffness" << endl;
        cout << "[0] - increase angular stiffness" << endl << endl;
        cout << "[q] - Exit application\n" << endl;
        cout << endl << endl;
    }

    // option - enable gravity
    else if (a_key == GLFW_KEY_1)
    {
        // enable gravity
        bulletWorld->setGravity(cVector3d(0.0, 0.0, -9.8));
        printf("gravity ON:\n");
    }

    // option - disable gravity
    else if (a_key == GLFW_KEY_2)
    {
        // disable gravity
        bulletWorld->setGravity(cVector3d(0.0, 0.0, 0.0));
        printf("gravity OFF:\n");
    }

    // option - decrease linear haptic gain
    else if (a_key == GLFW_KEY_3)
    {
        linGain = linGain - 0.05;
        if (linGain < 0)
            linGain = 0;
        printf("linear haptic gain:  %f\n", linGain);
    }

    // option - increase linear haptic gain
    else if (a_key == GLFW_KEY_4)
    {
        linGain = linGain + 0.05;
        printf("linear haptic gain:  %f\n", linGain);
    }

    // option - decrease angular haptic gain
    else if (a_key == GLFW_KEY_5)
    {
        angGain = angGain - 0.005;
        if (angGain < 0)
            angGain = 0;
        printf("angular haptic gain:  %f\n", angGain);
    }

    // option - increase angular haptic gain
    else if (a_key == GLFW_KEY_6)
    {
        angGain = angGain + 0.005;
        printf("angular haptic gain:  %f\n", angGain);
    }

    // option - decrease linear stiffness
    else if (a_key == GLFW_KEY_7)
    {
        linStiffness = linStiffness - 50;
        if (linStiffness < 0)
            linStiffness = 0;
        printf("linear stiffness:  %f\n", linStiffness);
    }

    // option - increase linear stiffness
    else if (a_key == GLFW_KEY_8)
    {
        linStiffness = linStiffness + 50;
        printf("linear stiffness:  %f\n", linStiffness);
    }

    // option - decrease angular stiffness
    else if (a_key == GLFW_KEY_9)
    {
        angStiffness = angStiffness - 1;
        if (angStiffness < 0)
            angStiffness = 0;
        printf("angular stiffness:  %f\n", angStiffness);
    }

    // option - increase angular stiffness
    else if (a_key == GLFW_KEY_0)
    {
        angStiffness = angStiffness + 1;
        printf("angular stiffness:  %f\n", angStiffness);
    }
}

//---------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    hapticDevice->close();

    // delete resources
    delete hapticsThread;
    delete bulletWorld;
    delete handler;
}

//---------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);
    bool _pressed;
    hapticDevice->getUserSwitch(1, _pressed);
    if(_pressed){
        double scale = 0.3;
        hapticDevice->getLinearVelocity(dev_vel);
        hapticDevice->getRotation(dev_rot_cur);
        camera->setLocalPos(camera->getLocalPos() + cMul(scale, cMul(camera->getGlobalRot(),dev_vel)));
        camera->setLocalRot(cMul(cam_rot_last, cMul(cTranspose(dev_rot_last), dev_rot_cur)));
    }
    if(!_pressed){
        cam_rot_last = camera->getGlobalRot();
        hapticDevice->getRotation(dev_rot_last);
    }

    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    bulletWorld->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(width, height);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{
    // simulation in now running
    simulationRunning = true;
    simulationFinished = false;

    // start haptic device
    hapticDevice->open();

    // simulation clock
    cPrecisionClock simClock;
    simClock.start(true);

    cMatrix3d prevRotTool;
    prevRotTool.identity();

    // update position and orientation of tool
    cVector3d posDevice, posSim, posSimLast, posDeviceClutched;
    cMatrix3d rotDevice, rotSim, rotSimLast, rotDeviceClutched;
    bool _firstClutchPress = false;
    hapticDevice->getRotation(rotDevice);
    rotDeviceClutched.identity();
    rotSimLast = rotDevice;

    // main haptic simulation loop
    while(simulationRunning)
    {
        // signal frequency counter
        freqCounterHaptics.signal(1);

        // retrieve simulation time and compute next interval
        double time = simClock.getCurrentTimeSeconds();
        double nextSimInterval = 0.0005;//cClamp(time, 0.00001, 0.0002);

        // reset clock
        simClock.reset();
        simClock.start();

        // compute global reference frames for each object
        bulletWorld->computeGlobalPositions(true);

        hapticDevice->getPosition(posDevice);
        hapticDevice->getRotation(rotDevice);

        // send forces to device
        bool _pressed;
        hapticDevice->getUserSwitch(1,_pressed);
        if(_pressed){
            if(_firstClutchPress){
                _firstClutchPress = false;
                posSimLast = posSim / workspaceScaleFactor;
                rotSimLast = rotSim;
            }
            posDeviceClutched = posDevice;
            rotDeviceClutched = rotDevice;
        }
        else{_firstClutchPress = true;}

        posSim = cAdd(posSimLast, cMul(camera->getLocalRot(), cSub(posDevice, posDeviceClutched)));
        rotSim = rotSimLast * camera->getLocalRot() * cTranspose(rotDeviceClutched) * rotDevice * cTranspose(camera->getLocalRot());
        posSim.mul(workspaceScaleFactor);

        // read position of tool
        cVector3d posTool = tool->getLocalPos();
        cMatrix3d rotTool = tool->getLocalRot();

        // compute position and angular error between tool and haptic device
        cVector3d deltaPos = (posSim - posTool);
        cMatrix3d deltaRot = cMul(cTranspose(rotTool), rotSim);
        double angle;
        cVector3d axis;
        deltaRot.toAxisAngle(axis, angle);

        // compute force and torque to apply to tool
        cVector3d force, torque;
        force = linStiffness * deltaPos;
        tool->addExternalForce(force);

        torque = cMul((angStiffness * angle), axis);
        rotTool.mul(torque);
        tool->addExternalTorque(torque);

        // compute force and torque to apply to haptic device
        force = -linG * force;
        torque = -angG * torque;

        // send forces to device
        hapticDevice->setForceAndTorqueAndGripperForce(force, torque, 0.0);

        if (linG < linGain)
        {
            linG = linG + 0.1 * time * linGain;
        }
        else
        {
            linG = linGain;
        }

        if (angG < angGain)
        {
            angG = angG + 0.1 * time * angGain;
        }
        else
        {
            angG = angGain;
        }

        // update simulation
        bulletWorld->updateDynamics(nextSimInterval);
    }

    // exit haptics thread
    simulationFinished = true;
}

