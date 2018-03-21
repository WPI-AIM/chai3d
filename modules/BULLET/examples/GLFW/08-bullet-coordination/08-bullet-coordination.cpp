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

    \author    <http://www.aimlab.wpi.edu>
    \author    Adnan Munawar
    \version   3.2.1 $Rev: 1869 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------
#include <GLFW/glfw3.h>
#include <boost/program_options.hpp>
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
cBulletWorld* g_bulletWorld;

// bullet objects
cBulletMesh* g_bulletMesh1;
cBulletMesh* g_bulletMesh2;
cBulletMesh* g_bulletCylinder;
cBulletMultiMesh* g_bulletGear;
cBulletMultiMesh* g_bulletTorus;
cBulletMultiMesh* g_bulletBase;

// bullet static walls and ground
cBulletStaticPlane* g_bulletGround;

cBulletStaticPlane* g_bulletBoxWall[5];

cVector3d g_camPos(0,0,0);
cVector3d g_dev_vel;
cMatrix3d g_cam_rot_last, g_dev_rot_last, g_dev_rot_cur;
double g_dt_fixed = 0;
bool g_force_enable = true;
// Default switch index for clutches


//---------------------------------------------------------------------------
// CHAI3D VARIABLES
//---------------------------------------------------------------------------


// a camera to render the world in the window display
cCamera* g_camera;

// a light source to illuminate the objects in the world
cSpotLight *g_light;

// a label to display the rates [Hz] at which the simulation is running
cLabel* g_labelRates;
cLabel* g_labelDevRates[10];
cLabel* g_labelTimes;
cLabel* g_labelModes;
cLabel* g_labelBtnAction;
std::string g_btn_action_str = "";
bool g_cam_btn_pressed = false;
bool g_clutch_btn_pressed = false;
cPrecisionClock g_clockWorld;


//---------------------------------------------------------------------------
// GENERAL VARIABLES
//---------------------------------------------------------------------------

// flag to indicate if the haptic simulation currently running
bool g_simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool g_simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter g_freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter g_freqCounterHaptics;

// haptic thread
cThread* g_hapticsThreads[10];
// bullet simulation thread
cThread* g_bulletSimThread;

// a handle to window display context
GLFWwindow* g_window = NULL;

// current width of window
int g_width = 0;

// current height of window
int g_height = 0;

// swap interval for the display context (vertical synchronization)
int g_swapInterval = 1;

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
void updateHaptics(void*);

//this function contains the main Bullet Simulation loop
void updateBulletSim(void);

// this function closes the application
void close(void);

const int MAX_DEVICES = 10;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
class DataExchange{
public:
    virtual cVector3d measured_pos(){}
    virtual cMatrix3d measured_rot(){}
    virtual void update_measured_pose(){}
    virtual cVector3d measured_lin_vel(){}
    virtual bool is_button_pressed(int button_index){}
    virtual double measured_gripper_angle(){}
    virtual void apply_wrench(cVector3d force, cVector3d torque){}
    virtual void apply_force(cVector3d force){}
    virtual void apply_torque(cVector3d torque){}
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* This class encapsulates each haptic device in isolation and provides methods to get/set device
 * state/commands, button's state and grippers state if present */
class Device: public DataExchange{
public:
    Device(){}
    ~Device(){}
    virtual cVector3d measured_pos();
    virtual cMatrix3d measured_rot();
    virtual void update_measured_pose();
    virtual cVector3d measured_lin_vel();
    virtual cVector3d mearured_ang_vel();
    virtual double measured_gripper_angle();
    virtual void apply_wrench(cVector3d a_force, cVector3d a_torque);
    virtual bool is_button_pressed(int button_index);
    virtual bool is_button_press_rising_edge(int button_index);
    virtual bool is_button_press_falling_edge(int button_index);
    void enable_force_feedback(bool enable){m_dev_force_enabled = enable;}
    cShapeSphere* create_cursor(cBulletWorld* a_world);
    cBulletSphere* create_af_cursor(cBulletWorld* a_world, std::string a_name);
    cGenericHapticDevicePtr m_hDevice;
    cHapticDeviceInfo m_hInfo;
    cVector3d m_posDevice, m_posDeviceClutched, m_velDevice, m_avelDevice;
    cMatrix3d m_rotDevice, m_rotDeviceClutched;
    double m_workspace_scale_factor;
    cShapeSphere* m_cursor = NULL;
    cBulletSphere* m_af_cursor = NULL;
    bool m_btn_prev_state_rising[10] = {false};
    bool m_btn_prev_state_falling[10] = {false};
    cFrequencyCounter m_freq_ctr;

private:
    boost::mutex m_mutex;
    void update_cursor_pose();
    bool m_dev_force_enabled = true;
};

cShapeSphere* Device::create_cursor(cBulletWorld* a_world){
    m_cursor = new cShapeSphere(0.05);
    m_cursor->setShowEnabled(true);
    m_cursor->setShowFrame(true);
    m_cursor->setFrameSize(0.1);
    cMaterial mat;
    mat.setGreenLightSea();
    m_cursor->setMaterial(mat);
    a_world->addChild(m_cursor);
    return m_cursor;
}

cBulletSphere* Device::create_af_cursor(cBulletWorld *a_world, string a_name){
    m_af_cursor = new cBulletSphere(a_world, 0.05, a_name);
    m_af_cursor->setShowEnabled(true);
    m_af_cursor->setShowFrame(true);
    m_af_cursor->setFrameSize(0.1);
    cMaterial mat;
    mat.setGreenLightSea();
    m_af_cursor->setMaterial(mat);
    m_af_cursor->buildDynamicModel();
    a_world->addChild(m_af_cursor);
    return m_af_cursor;
}

cVector3d Device::measured_pos(){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    m_hDevice->getPosition(m_posDevice);
    update_cursor_pose();
    return m_posDevice;
}

cMatrix3d Device::measured_rot(){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    m_hDevice->getRotation(m_rotDevice);
    return m_rotDevice;
}

void Device::update_measured_pose(){
    update_cursor_pose();
}

void Device::update_cursor_pose(){
    if(m_cursor){
        m_cursor->setLocalPos(m_posDevice * m_workspace_scale_factor);
        m_cursor->setLocalRot(m_rotDevice);
    }
    if(m_af_cursor){
        m_af_cursor->setLocalPos(m_posDevice * m_workspace_scale_factor);
        m_af_cursor->setLocalRot(m_rotDevice);
    }
}

cVector3d Device::measured_lin_vel(){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    m_hDevice->getLinearVelocity(m_velDevice);
    return m_velDevice;
}

cVector3d Device::mearured_ang_vel(){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    m_hDevice->getAngularVelocity(m_avelDevice);
    return m_avelDevice;
}

double Device::measured_gripper_angle(){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    double angle;
    m_hDevice->getGripperAngleRad(angle);
    return angle;
}

bool Device::is_button_pressed(int button_index){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    bool status;
    m_hDevice->getUserSwitch(button_index, status);
    return status;
}

bool Device::is_button_press_rising_edge(int button_index){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    bool status;
    m_hDevice->getUserSwitch(button_index, status);
    if (m_btn_prev_state_rising[button_index] ^ status){
        if (!m_btn_prev_state_rising[button_index]){
            m_btn_prev_state_rising[button_index] = true;
            return true;
        }
        else{
            m_btn_prev_state_rising[button_index] = false;
        }
    }
    return false;
}

bool Device::is_button_press_falling_edge(int button_index){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    bool status;
    m_hDevice->getUserSwitch(button_index, status);
    if (m_btn_prev_state_falling[button_index] ^ status){
        if (m_btn_prev_state_falling[button_index]){
            m_btn_prev_state_falling[button_index] = false;
            return true;
        }
        else{
            m_btn_prev_state_falling[button_index] = true;
        }
    }
    return false;
}

void Device::apply_wrench(cVector3d force, cVector3d torque){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    force = force * m_dev_force_enabled;
    torque = torque * m_dev_force_enabled;
    m_hDevice->setForceAndTorqueAndGripperForce(force, torque, 0.0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* This class encapsulates Simulation Parameters that deal with the interaction between a single haptic device
 *  and the related Gripper simulated in Bullet. These Parameters include mapping the device buttons to
 * action/mode buttons, capturing button triggers in addition to presses, mapping the workspace scale factors
 * for a device and so on. */
class Sim{
public:
    Sim(){
        m_workspaceScaleFactor = 30.0;
        K_lh = 0.02;
        K_lh_ramp = 0.0;
        K_ah_ramp = 0.0;
        K_ah = 0.03;
        K_lc = 200;
        K_ac = 30;
        B_lc = 5.0;
        B_ac = 3.0;
        act_1_btn   = 0;
        act_2_btn   = 1;
        mode_next_btn = 2;
        mode_prev_btn= 3;
        btn_cam_rising_edge = false;
        btn_clutch_rising_edge = false;
        m_posRefLast.set(0.0,0.0,0.0);
        m_rotRefLast.identity();
        m_loop_exec_flag = false;
    }
    void set_sim_params(cHapticDeviceInfo &a_hInfo, Device* a_dev);
    inline void set_loop_exec_flag(){m_loop_exec_flag=true;}
    inline void clear_loop_exec_flag(){m_loop_exec_flag = false;}
    inline bool is_loop_exec(){return m_loop_exec_flag;}
    inline double get_workspace_scale_factor(){return m_workspaceScaleFactor;}
    cVector3d m_posRef, m_posRefLast;
    cMatrix3d m_rotRef, m_rotRefLast;
    double m_workspaceScaleFactor;
    double K_lh;                    //Linear Haptic Stiffness Gain
    double K_ah;                    //Angular Haptic Stiffness Gain
    double K_lh_ramp;               //Linear Haptic Stiffness Gain Ramped
    double K_ah_ramp;               //Angular Haptic Stiffness Gain Ramped
    double K_lc;                    //Linear Controller Stiffness Gain
    double K_ac;                    //Angular Controller Stiffness Gain
    double B_lc;                    //Linear Controller Damping Gain
    double B_ac;                    //Angular Controller Damping Gain
    int act_1_btn;
    int act_2_btn;
    int mode_next_btn;
    int mode_prev_btn;
    int m_gripper_pinch_btn = -1;
    bool btn_cam_rising_edge;
    bool btn_clutch_rising_edge;
    bool m_loop_exec_flag;
};

void Sim::set_sim_params(cHapticDeviceInfo &a_hInfo, Device* a_dev){
    double maxStiffness	= a_hInfo.m_maxLinearStiffness / m_workspaceScaleFactor;

    // clamp the force output gain to the max device stiffness
    K_lh = cMin(K_lh, maxStiffness / K_lc);
    if (strcmp(a_hInfo.m_modelName.c_str(), "MTM-R") == 0 || strcmp(a_hInfo.m_modelName.c_str(), "MTMR") == 0 ||
        strcmp(a_hInfo.m_modelName.c_str(), "MTM-L") == 0 || strcmp(a_hInfo.m_modelName.c_str(), "MTML") == 0)
    {
        std::cout << "Device " << a_hInfo.m_modelName << " DETECTED, CHANGING BUTTON AND WORKSPACE MAPPING" << std::endl;
        m_workspaceScaleFactor = 10.0;
        K_lh = K_lh/3;
        act_1_btn     =  1;
        act_2_btn     =  2;
        mode_next_btn =  3;
        mode_prev_btn =  4;
        K_lh = 0.04;
        K_ah = 0.0;
        m_gripper_pinch_btn = 0;
        a_dev->enable_force_feedback(false);
    }

    if (strcmp(a_hInfo.m_modelName.c_str(), "Falcon") == 0)
    {
        std::cout << "Device " << a_hInfo.m_modelName << " DETECTED, CHANGING BUTTON AND WORKSPACE MAPPING" << std::endl;
        act_1_btn     = 0;
        act_2_btn     = 2;
        mode_next_btn = 3;
        mode_prev_btn = 1;
        K_lh = 0.05;
        K_ah - 0.0;
    }

    if (strcmp(a_hInfo.m_modelName.c_str(), "PHANTOM Omni") == 0)
    {
        std::cout << "Device " << a_hInfo.m_modelName << " DETECTED, CHANGING BUTTON AND WORKSPACE MAPPING" << std::endl;
        K_lh = 0.01;
        K_ah = 0.0;
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* This class encapsulates a single Gripper, simulated in Bullet and provides methods to get/set state/commands
 * for interface with the haptics device*/
class ToolGripper: public Sim, public DataExchange{
public:
    ToolGripper(){m_gripper_angle = 3.0;}
    ~ToolGripper(){}
    virtual cVector3d measured_pos();
    virtual cMatrix3d measured_rot();
    virtual void update_measured_pose();
    virtual inline void apply_force(cVector3d force){if (!gripper->m_af_pos_ctrl_active) gripper->addExternalForce(force);}
    virtual inline void apply_torque(cVector3d torque){if (!gripper->m_af_pos_ctrl_active) gripper->addExternalTorque(torque);}
    bool is_wrench_set();
    void clear_wrench();
    void offset_gripper_angle(double offset);
    void set_gripper_angle(double angle);
    cBulletGripper* gripper;
    cVector3d m_posGripper;
    cMatrix3d m_rotGripper;
    double m_gripper_angle;

    boost::mutex m_mutex;
};

cVector3d ToolGripper::measured_pos(){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    return gripper->getLocalPos();
}

cMatrix3d ToolGripper::measured_rot(){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    return gripper->getLocalRot();
}

void ToolGripper::update_measured_pose(){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    m_posGripper  = gripper->getLocalPos();
    m_rotGripper = gripper->getLocalRot();
}

void ToolGripper::set_gripper_angle(double angle){
    if(!gripper->m_af_pos_ctrl_active) gripper->set_gripper_angle(angle);
}

void ToolGripper::offset_gripper_angle(double offset){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    m_gripper_angle += offset;
    m_gripper_angle = cClamp(m_gripper_angle, 0.0, 1.0);
    gripper->set_gripper_angle(m_gripper_angle);
}

bool ToolGripper::is_wrench_set(){
    btVector3 f = gripper->m_bulletRigidBody->getTotalForce();
    btVector3 n = gripper->m_bulletRigidBody->getTotalTorque();
    if (f.isZero()) return false;
    else return true;
}

void ToolGripper::clear_wrench(){
    gripper->m_bulletRigidBody->clearForces();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* These are the currently availble modes for each device. */

enum MODES{ CAM_CLUTCH_CONTROL,
            GRIPPER_JAW_CONTROL,
            CHANGE_CONT_LIN_GAIN,
            CHANGE_CONT_ANG_GAIN,
            CHANGE_CONT_LIN_DAMP,
            CHANGE_CONT_ANG_DAMP,
            CHANGE_DEV_LIN_GAIN,
            CHANGE_DEV_ANG_GAIN
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* This is a higher level class that queries the number of haptics devices available on the sytem
 * and on the Network for dVRK devices and creates a single Bullet Gripper and a Device Handle for
   each device. */
class Coordination{
    public:
    Coordination(cBulletWorld* a_bullet_world, int a_max_load_devs = MAX_DEVICES);
    bool retrieve_device_handle(uint dev_num);
    void create_bullet_gripper(uint dev_num);
    void open_devices();
    void close_devices();

    double increment_K_lh(double a_offset);
    double increment_K_ah(double a_offset);
    double increment_K_lc(double a_offset);
    double increment_K_ac(double a_offset);
    double increment_B_lc(double a_offset);
    double increment_B_ac(double a_offset);
    bool are_all_haptics_loop_exec();
    int num_of_haptics_loop_execd();
    void clear_all_haptics_loop_exec_flags();

    void next_mode();
    void prev_mode();

    cHapticDeviceHandler *m_deviceHandler;
    ToolGripper m_bulletTools[MAX_DEVICES];
    Device m_hapticDevices[MAX_DEVICES];
    uint m_num_devices;
    cBulletWorld* m_bulletWorld;

    // bool to enable the rotation of tool be in camera frame. i.e. Orienting the camera
    // re-orients the tool.
    bool m_use_cam_frame_rot;
    MODES m_simModes;
    std::string m_mode_str;
    std::vector<MODES> m_modes_enum_vec {MODES::CAM_CLUTCH_CONTROL,
                                         MODES::GRIPPER_JAW_CONTROL,
                                         MODES::CHANGE_CONT_LIN_GAIN,
                                         MODES::CHANGE_CONT_ANG_GAIN,
                                         MODES::CHANGE_CONT_LIN_DAMP,
                                         MODES::CHANGE_CONT_ANG_DAMP,
                                         MODES::CHANGE_DEV_LIN_GAIN,
                                         MODES::CHANGE_DEV_ANG_GAIN};

    std::vector<std::string> m_modes_enum_str {"CAM_CLUTCH_CONTROL  ",
                                               "GRIPPER_JAW_CONTROL ",
                                               "CHANGE_CONT_LIN_GAIN",
                                               "CHANGE_CONT_ANG_GAIN",
                                               "CHANGE_CONT_LIN_DAMP",
                                               "CHANGE_CONT_ANG_DAMP",
                                               "CHANGE_DEV_LIN_GAIN ",
                                               "CHANGE_DEV_ANG_GAIN "};
    int m_mode_idx;
};

Coordination::Coordination(cBulletWorld* a_bullet_world, int a_max_load_devs){
    m_bulletWorld = NULL;
    m_bulletWorld = a_bullet_world;
    m_deviceHandler = new cHapticDeviceHandler();
    m_num_devices = m_deviceHandler->getNumDevices();
    std::cerr << "Num of devices " << m_num_devices << std::endl;
    if (a_max_load_devs < m_num_devices) m_num_devices = a_max_load_devs;
    for (uint i = 0; i < m_num_devices; i++){
        retrieve_device_handle(i);
        create_bullet_gripper(i);
    }
    m_use_cam_frame_rot = true;
    m_simModes = CAM_CLUTCH_CONTROL;
    m_mode_str = "CAM_CLUTCH_CONTROL";
    m_mode_idx = 0;
}

void Coordination::next_mode(){
    m_mode_idx = (m_mode_idx + 1) % m_modes_enum_vec.size();
    m_simModes = m_modes_enum_vec[m_mode_idx];
    m_mode_str = m_modes_enum_str[m_mode_idx];
    g_btn_action_str = "";
    g_cam_btn_pressed = false;
    g_clutch_btn_pressed = false;
    std::cout << m_mode_str << std::endl;
}


void Coordination::prev_mode(){
    m_mode_idx = (m_mode_idx - 1) % m_modes_enum_vec.size();
    m_simModes = m_modes_enum_vec[m_mode_idx];
    m_mode_str = m_modes_enum_str[m_mode_idx];
    g_btn_action_str = "";
    g_cam_btn_pressed = false;
    g_clutch_btn_pressed = false;
    std::cout << m_mode_str << std::endl;
}

bool Coordination::retrieve_device_handle(uint dev_num){
    m_deviceHandler->getDeviceSpecifications(m_hapticDevices[dev_num].m_hInfo, dev_num);
    return m_deviceHandler->getDevice(m_hapticDevices[dev_num].m_hDevice, dev_num);
}

void Coordination::create_bullet_gripper(uint dev_num){
    std::ostringstream dev_str;
    dev_str << (dev_num + 1);
    std::string gripper_name = "Gripper" + dev_str.str();
    m_bulletTools[dev_num].gripper = new cBulletGripper(m_bulletWorld, gripper_name);
    m_bulletTools[dev_num].set_sim_params(m_hapticDevices[dev_num].m_hInfo, & m_hapticDevices[dev_num]);
    m_bulletTools[dev_num].gripper->build();
    m_bulletWorld->addChild(m_bulletTools[dev_num].gripper);
    m_hapticDevices[dev_num].m_workspace_scale_factor = m_bulletTools[dev_num].get_workspace_scale_factor();
}

void Coordination::open_devices(){
    for (int i = 0 ; i < m_num_devices ; i++){
        m_hapticDevices[i].m_hDevice->open();
        std::string name = "Device" + std::to_string(i+1);
        m_hapticDevices[i].create_cursor(m_bulletWorld);
    }
}

void Coordination::close_devices(){
    for (int i = 0 ; i < m_num_devices ; i++){
        m_hapticDevices[i].m_hDevice->close();
    }
}

int Coordination::num_of_haptics_loop_execd(){
    int num_devs_loop_execd = 0;
    for (int i = 0 ; i < m_num_devices ; i++){
        if (m_bulletTools[i].is_loop_exec()) num_devs_loop_execd++;
    }
    return num_devs_loop_execd;
}

bool Coordination::are_all_haptics_loop_exec(){
    bool flag = true;
    for (int i = 0 ; i < m_num_devices ; i++){
        flag &= m_bulletTools[i].is_loop_exec();
    }
    return flag;
}

void Coordination::clear_all_haptics_loop_exec_flags(){
    for (int i = 0 ; i < m_num_devices ; i++){
        m_bulletTools[i].clear_loop_exec_flag();
    }
}

double Coordination::increment_K_lh(double a_offset){
    for (int i = 0 ; i < m_num_devices ; i++){
        if (m_bulletTools[i].K_lh + a_offset <= 0)
        {
            m_bulletTools[i].K_lh = 0.0;
        }
        else{
            m_bulletTools[i].K_lh += a_offset;
        }
    }
    //Set the return value to the gain of the last device
    if(m_num_devices > 0){
        a_offset = m_bulletTools[m_num_devices-1].K_lh;
        g_btn_action_str = "K_lh = " + cStr(a_offset, 4);
    }
    return a_offset;
}

double Coordination::increment_K_ah(double a_offset){
    for (int i = 0 ; i < m_num_devices ; i++){
        if (m_bulletTools[i].K_ah + a_offset <=0){
            m_bulletTools[i].K_ah = 0.0;
        }
        else{
            m_bulletTools[i].K_ah += a_offset;
        }
    }
    //Set the return value to the gain of the last device
    if(m_num_devices > 0){
        a_offset = m_bulletTools[m_num_devices-1].K_ah;
        g_btn_action_str = "K_ah = " + cStr(a_offset, 4);
    }
    return a_offset;
}

double Coordination::increment_K_lc(double a_offset){
    for (int i = 0 ; i < m_num_devices ; i++){
        if (m_bulletTools[i].K_lc + a_offset <=0){
            m_bulletTools[i].K_lc = 0.0;
        }
        else{
            m_bulletTools[i].K_lc += a_offset;
        }
    }
    //Set the return value to the stiffness of the last device
    if(m_num_devices > 0){
        a_offset = m_bulletTools[m_num_devices-1].K_lc;
        g_btn_action_str = "K_lc = " + cStr(a_offset, 4);
    }
    return a_offset;
}

double Coordination::increment_K_ac(double a_offset){
    for (int i = 0 ; i < m_num_devices ; i++){
        if (m_bulletTools[i].K_ac + a_offset <=0){
            m_bulletTools[i].K_ac = 0.0;
        }
        else{
            m_bulletTools[i].K_ac += a_offset;
        }
    }
    //Set the return value to the stiffness of the last device
    if(m_num_devices > 0){
        a_offset = m_bulletTools[m_num_devices-1].K_ac;
        g_btn_action_str = "K_ac = " + cStr(a_offset, 4);
    }
    return a_offset;
}

double Coordination::increment_B_lc(double a_offset){
    for (int i = 0 ; i < m_num_devices ; i++){
        if (m_bulletTools[i].B_lc + a_offset <=0){
            m_bulletTools[i].B_lc = 0.0;
        }
        else{
            m_bulletTools[i].B_lc += a_offset;
        }
    }
    //Set the return value to the stiffness of the last device
    if(m_num_devices > 0){
        a_offset = m_bulletTools[m_num_devices-1].B_lc;
        g_btn_action_str = "B_lc = " + cStr(a_offset, 4);
    }
    return a_offset;
}

double Coordination::increment_B_ac(double a_offset){
    for (int i = 0 ; i < m_num_devices ; i++){
        if (m_bulletTools[i].B_ac + a_offset <=0){
            m_bulletTools[i].B_ac = 0.0;
        }
        else{
            m_bulletTools[i].B_ac += a_offset;
        }
    }
    //Set the return value to the stiffness of the last device
    if(m_num_devices > 0){
        a_offset = m_bulletTools[m_num_devices-1].B_ac;
        g_btn_action_str = "B_ac = " + cStr(a_offset, 4);
    }
    return a_offset;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* This is an implementation of Sleep function that tries to adjust sleep between each cycle to maintain
 * the desired loop frequency. This class has been inspired from ROS Rate Sleep written by Eitan Marder-Eppstein */
class RateSleep{
public:
    RateSleep(int a_freq){
        m_cycle_time = 1.0 / double(a_freq);
        m_rateClock.start();
        m_next_expected_time = m_rateClock.getCurrentTimeSeconds() + m_cycle_time;
    }
  bool sleep(){
      double cur_time = m_rateClock.getCurrentTimeSeconds();
      if (cur_time >= m_next_expected_time){
          m_next_expected_time = cur_time + m_cycle_time;
          return true;
      }
      while(m_rateClock.getCurrentTimeSeconds() <= m_next_expected_time){

      }
      m_next_expected_time = m_rateClock.getCurrentTimeSeconds() + m_cycle_time;
      return true;
  }
private:
  double m_next_expected_time;
  double m_cycle_time;
  cPrecisionClock m_rateClock;
};


std::shared_ptr<Coordination> g_coordApp;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//===========================================================================
/*
    Application:    08-bullet-coordination.cpp

    This Application allows multi-manual tasks using several haptics devices.
    Each device can perturb or control the dynamic bodies in the simulation
    environment. The objects in the simulation are exposed via Asynchoronous
    Framework (AF) to allow query and control via external applications.
 */
//===========================================================================
int main(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------
    namespace p_opt = boost::program_options;

    p_opt::options_description cmd_opts("Coordination Application Usage");
    cmd_opts.add_options()
            ("help,h", "Show help")
            ("ndevs,n", p_opt::value<int>(), "Number of Haptic Devices to Load")
            ("timestep,t", p_opt::value<double>(), "Value in secs for fixed Simulation time step(dt)")
            ("enableforces,f", p_opt::value<bool>(), "Enable Force Feedback on Devices");
    p_opt::variables_map var_map;
    p_opt::store(p_opt::command_line_parser(argc, argv).options(cmd_opts).run(), var_map);
    p_opt::notify(var_map);

    int num_devices_to_load = MAX_DEVICES;
    if(var_map.count("help")){ std::cout<< cmd_opts << std::endl; return 0;}
    if(var_map.count("ndevs")){ num_devices_to_load = var_map["ndevs"].as<int>();}
    if (var_map.count("timestep")){ g_dt_fixed = var_map["timestep"].as<double>();}
    if (var_map.count("enableforces")){ g_force_enable = var_map["enableforces"].as<bool>();}

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Application: 08-bullet-coordination" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
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
    g_window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!g_window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(g_window, &g_width, &g_height);

    // set position of window
    glfwSetWindowPos(g_window, x, y);

    // set key callback
    glfwSetKeyCallback(g_window, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(g_window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(g_window);

    // sets the swap interval for the current display context
    glfwSwapInterval(g_swapInterval);

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
    g_bulletWorld = new cBulletWorld("World");

    // set the background color of the environment
    g_bulletWorld->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    g_camera = new cCamera(g_bulletWorld);
    g_bulletWorld->addChild(g_camera);

    // position and orient the camera
    g_camera->set(cVector3d(3.0, 0.0, 0.3),    // camera position (eye)
                cVector3d(0.0, 0.0,-0.5),    // lookat position (target)
                cVector3d(0.0, 0.0, 1.0));   // direction of the "up" vector

    // set the near and far clipping planes of the camera
    g_camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    g_camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    g_camera->setStereoEyeSeparation(0.02);
    g_camera->setStereoFocalLength(2.0);

    // set vertical mirrored display mode
    g_camera->setMirrorVertical(mirroredDisplay);

    // create a light source
    g_light = new cSpotLight(g_bulletWorld);

    // attach light to camera
    g_bulletWorld->addChild(g_light);

    // enable light source
    g_light->setEnabled(true);

    // position the light source
    g_light->setLocalPos( 0, 0, 1.2);

    // define the direction of the light beam
    g_light->setDir(0,0,-1.0);

    // set uniform concentration level of light
    g_light->setSpotExponent(0.0);

    // enable this light source to generate shadows
    g_light->setShadowMapEnabled(true);

    // set the resolution of the shadow map
    g_light->m_shadowMap->setQualityLow();
    //light->m_shadowMap->setQualityMedium();

    // set light cone half angle
    g_light->setCutOffAngleDeg(45);


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFontPtr font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic and graphic rate of the simulation
    g_labelRates = new cLabel(font);
    g_labelTimes = new cLabel(font);
    g_labelModes = new cLabel(font);
    g_labelBtnAction = new cLabel(font);
    g_labelRates->m_fontColor.setBlack();
    g_labelTimes->m_fontColor.setBlack();
    g_labelModes->m_fontColor.setBlack();
    g_labelBtnAction->m_fontColor.setBlack();
    g_camera->m_frontLayer->addChild(g_labelRates);
    g_camera->m_frontLayer->addChild(g_labelTimes);
    g_camera->m_frontLayer->addChild(g_labelModes);
    g_camera->m_frontLayer->addChild(g_labelBtnAction);

    //////////////////////////////////////////////////////////////////////////
    // BULLET WORLD
    //////////////////////////////////////////////////////////////////////////
    // set some gravity
    g_bulletWorld->setGravity(cVector3d(0.0, 0.0, -9.8));


    //////////////////////////////////////////////////////////////////////////
    // 3 BULLET BLOCKS
    //////////////////////////////////////////////////////////////////////////
    double size = 0.40;
    cMaterial meshMat;

    g_bulletGear = new cBulletMultiMesh(g_bulletWorld, "Gear");
    g_bulletGear->loadFromFile(RESOURCE_PATH("../resources/models/gear/gear.3ds"));
    g_bulletGear->scale(0.0014);
    g_bulletWorld->addChild(g_bulletGear);
    g_bulletGear->buildContactTriangles(0.001);
    g_bulletGear->setMass(0.3);
    g_bulletGear->estimateInertia();
    g_bulletGear->buildDynamicModel();
    meshMat.setPinkDeep();
    g_bulletGear->setMaterial(meshMat);
    g_bulletGear->m_bulletRigidBody->setFriction(1);

    g_bulletTorus = new cBulletMultiMesh(g_bulletWorld, "Torus");
    g_bulletTorus->loadFromFile(RESOURCE_PATH("../resources/models/gear/torus.3ds"));
    g_bulletTorus->scale(0.2);
    g_bulletTorus->setLocalPos(cVector3d(0.3,0,0));
    g_bulletWorld->addChild(g_bulletTorus);
    g_bulletTorus->buildContactTriangles(0.001);
    g_bulletTorus->setMass(0.8);
    g_bulletTorus->estimateInertia();
    g_bulletTorus->buildDynamicModel();
    meshMat.setOrangeTomato();
    g_bulletTorus->setMaterial(meshMat);

    g_bulletBase = new cBulletMultiMesh(g_bulletWorld, "Base");
    g_bulletBase->loadFromFile(RESOURCE_PATH("../resources/models/gear/base.3ds"));
    g_bulletBase->scale(0.3);
    g_bulletBase->setLocalPos(cVector3d(-0.3,0,0));
    g_bulletWorld->addChild(g_bulletBase);
    g_bulletBase->buildContactTriangles(0.001);
    g_bulletBase->setMass(10);
    g_bulletBase->estimateInertia();
    g_bulletBase->buildDynamicModel();
    meshMat.setBlueNavy();
    g_bulletBase->setMaterial(meshMat);
    g_bulletBase->m_bulletRigidBody->setFriction(1);

    g_coordApp = std::make_shared<Coordination>(g_bulletWorld, num_devices_to_load);

    //////////////////////////////////////////////////////////////////////////
    // INVISIBLE WALLS
    //////////////////////////////////////////////////////////////////////////

    // we create 5 static walls to contain the dynamic objects within a limited workspace
    double planeWidth = 1.0;
    g_bulletBoxWall[0] = new cBulletStaticPlane(g_bulletWorld, cVector3d(0.0, 0.0, -1.0), -2.0 * planeWidth);
    g_bulletBoxWall[1] = new cBulletStaticPlane(g_bulletWorld, cVector3d(0.0, -1.0, 0.0), -1.5*planeWidth);
    g_bulletBoxWall[2] = new cBulletStaticPlane(g_bulletWorld, cVector3d(0.0, 1.0, 0.0), -1.5*planeWidth);
    g_bulletBoxWall[3] = new cBulletStaticPlane(g_bulletWorld, cVector3d(-1.0, 0.0, 0.0), -planeWidth);
    g_bulletBoxWall[4] = new cBulletStaticPlane(g_bulletWorld, cVector3d(1.0, 0.0, 0.0), -0.8 * planeWidth);

    for (int i = 0 ; i < 5 ; i++){
        cVector3d worldZ;
        worldZ.set(0,0,1);
        cVector3d planeOri = cCross(g_bulletBoxWall[i]->getPlaneNormal(), worldZ);
        cMatrix3d planeRot;
        planeRot.setAxisAngleRotationDeg(planeOri, 90);
        worldZ.set(0,0,1);
        g_bulletWorld->addChild(g_bulletBoxWall[i]);
        cCreatePlane(g_bulletBoxWall[i], 2.0, 3.0,
                     g_bulletBoxWall[i]->getPlaneConstant() * g_bulletBoxWall[i]->getPlaneNormal(),
                     planeRot);
        cMaterial matPlane;
        matPlane.setBlueSky();
        g_bulletBoxWall[i]->setMaterial(matPlane);
        g_bulletBoxWall[i]->setTransparencyLevel(0.5, true, true);
    }


    //////////////////////////////////////////////////////////////////////////
    // GROUND
    //////////////////////////////////////////////////////////////////////////

    // create ground plane
    g_bulletGround = new cBulletStaticPlane(g_bulletWorld, cVector3d(0.0, 0.0, 1.0), -planeWidth);

    // add plane to world as we will want to make it visibe
    g_bulletWorld->addChild(g_bulletGround);

    // create a mesh plane where the static plane is located
    cCreatePlane(g_bulletGround, 3.0, 3.0, g_bulletGround->getPlaneConstant() * g_bulletGround->getPlaneNormal());

    // define some material properties and apply to mesh
    cMaterial matGround;
    matGround.setGreenChartreuse();
    matGround.m_emission.setGrayLevel(0.3);
    g_bulletGround->setMaterial(matGround);
    g_bulletGround->m_bulletRigidBody->setFriction(1);

    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------
    g_coordApp->open_devices();

    // create a thread which starts the main haptics rendering loop
    int dev_num[10] = {0,1,2,3,4,5,6,7,8,9};
    for (int i = 0 ; i < g_coordApp->m_num_devices ; i++){
        g_hapticsThreads[i] = new cThread();
        g_hapticsThreads[i]->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS, &dev_num[i]);
    }
    //create a thread which starts the Bullet Simulation loop
    g_bulletSimThread = new cThread();
    g_bulletSimThread->start(updateBulletSim, CTHREAD_PRIORITY_HAPTICS);

    for (int i = 0 ; i < g_coordApp->m_num_devices ; i++){
        g_labelDevRates[i] = new cLabel(font);
        g_labelDevRates[i]->m_fontColor.setBlack();
        g_labelDevRates[i]->setFontScale(0.8);
        g_camera->m_frontLayer->addChild(g_labelDevRates[i]);
    }

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(g_window, g_width, g_height);

    // main graphic loop
    while (!glfwWindowShouldClose(g_window))
    {
        // get width and height of window
        glfwGetWindowSize(g_window, &g_width, &g_height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(g_window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        g_freqCounterGraphics.signal(1);
    }

    // close window
    glfwDestroyWindow(g_window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}

//---------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    g_width = a_width;
    g_height = a_height;
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
            glfwSetWindowMonitor(g_window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(g_swapInterval);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(g_window, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(g_swapInterval);
        }
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M)
    {
        mirroredDisplay = !mirroredDisplay;
        g_camera->setMirrorVertical(mirroredDisplay);
    }

    // option - help menu
    else if (a_key == GLFW_KEY_H)
    {
        cout << "Keyboard Options:" << endl << endl;    g_coordApp->close_devices();
        cout << "[h] - Display help menu" << endl;
        cout << "[1] - Enable gravity" << endl;
        cout << "[2] - Disable gravity" << endl << endl;    g_coordApp->close_devices();
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
        g_bulletWorld->setGravity(cVector3d(0.0, 0.0, -9.8));
        printf("gravity ON:\n");
    }

    // option - disable gravity
    else if (a_key == GLFW_KEY_2)
    {
        // disable gravity
        g_bulletWorld->setGravity(cVector3d(0.0, 0.0, 0.0));
        printf("gravity OFF:\n");
    }

    // option - decrease linear haptic gain
    else if (a_key == GLFW_KEY_3)
    {
        printf("linear haptic gain:  %f\n", g_coordApp->increment_K_lh(-0.05));
    }

    // option - increase linear haptic gain
    else if (a_key == GLFW_KEY_4)
    {
        printf("linear haptic gain:  %f\n", g_coordApp->increment_K_lh(0.05));
    }

    // option - decrease angular haptic gain
    else if (a_key == GLFW_KEY_5)
    {
        printf("angular haptic gain:  %f\n", g_coordApp->increment_K_ah(-0.05));
    }

    // option - increase angular haptic gain
    else if (a_key == GLFW_KEY_6)
    {
        printf("angular haptic gain:  %f\n", g_coordApp->increment_K_ah(0.05));
    }

    // option - decrease linear stiffness
    else if (a_key == GLFW_KEY_7)
    {
        printf("linear stiffness:  %f\n", g_coordApp->increment_K_lc(-50));
    }

    // option - increase linear stiffness
    else if (a_key == GLFW_KEY_8)
    {
        printf("linear stiffness:  %f\n", g_coordApp->increment_K_lc(50));
    }

    // option - decrease angular stiffness
    else if (a_key == GLFW_KEY_9)
    {
        printf("angular stiffness:  %f\n", g_coordApp->increment_K_ac(-1));
    }

    // option - increase angular stiffness
    else if (a_key == GLFW_KEY_0)
    {
        printf("angular stiffness:  %f\n", g_coordApp->increment_K_ac(1));
    }
    else if (a_key == GLFW_KEY_C){
        g_coordApp->m_use_cam_frame_rot = true;
        printf("Gripper Rotation w.r.t Camera Frame:\n");
    }
    else if (a_key == GLFW_KEY_W){
        g_coordApp->m_use_cam_frame_rot = false;
        printf("Gripper Rotation w.r.t World Frame:\n");
    }
//    // option - open gripper
//    else if (a_key == GLFW_KEY_S)
//    {
//        grip_angle -= 0.01;
//        printf("gripper angle:  %f\n", grip_angle);
//    }
//    // option - open close gripper
//    else if (a_key == GLFW_KEY_D)
//    {
//        grip_angle += 0.01;
//        printf("gripper angle:  %f\n", grip_angle);
//    }
}

//---------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    g_simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!g_simulationFinished) { cSleepMs(100); }

    // delete resources
    g_coordApp->close_devices();
    for(int i = 0 ; i < g_coordApp->m_num_devices ; i ++){delete g_hapticsThreads[i];}
    delete g_bulletWorld;
    delete g_coordApp->m_deviceHandler;
}

//---------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic and graphic rate data
    g_labelTimes->setText("Wall Time: " + cStr(g_clockWorld.getCurrentTimeSeconds(),2) + " s" +
                        + " / "+" Simulation Time: " + cStr(g_bulletWorld->getSimulationTime(),2) + " s");
    g_labelRates->setText(cStr(g_freqCounterGraphics.getFrequency(), 0) + " Hz / " + cStr(g_freqCounterHaptics.getFrequency(), 0) + " Hz");
    g_labelModes->setText("MODE: " + g_coordApp->m_mode_str);
    g_labelBtnAction->setText(" : " + g_btn_action_str);

    for (int i = 0 ; i < g_coordApp->m_num_devices ; i++){
        g_labelDevRates[i]->setText(g_coordApp->m_hapticDevices[i].m_hInfo.m_modelName + ": " + cStr(g_coordApp->m_hapticDevices[i].m_freq_ctr.getFrequency(), 0) + " Hz");
        g_labelDevRates[i]->setLocalPos(10, (int)(g_height - (i+1)*20));
    }

    // update position of label
    g_labelTimes->setLocalPos((int)(0.5 * (g_width - g_labelTimes->getWidth())), 30);
    g_labelRates->setLocalPos((int)(0.5 * (g_width - g_labelRates->getWidth())), 10);
    g_labelModes->setLocalPos((int)(0.5 * (g_width - g_labelModes->getWidth())), 50);
    g_labelBtnAction->setLocalPos((int)(0.5 * (g_width - g_labelModes->getWidth()) + g_labelModes->getWidth()), 50);
    bool _pressed;
    if (g_coordApp->m_num_devices > 0){
        g_coordApp->m_hapticDevices[0].m_hDevice->getUserSwitch(g_coordApp->m_bulletTools[0].act_2_btn, _pressed);
        if(_pressed && g_coordApp->m_simModes == MODES::CAM_CLUTCH_CONTROL){
            double scale = 0.3;
            g_dev_vel = g_coordApp->m_hapticDevices[0].measured_lin_vel();
            g_coordApp->m_hapticDevices[0].m_hDevice->getRotation(g_dev_rot_cur);
            g_camera->setLocalPos(g_camera->getLocalPos() + cMul(scale, cMul(g_camera->getGlobalRot(),g_dev_vel)));
            g_camera->setLocalRot(cMul(g_cam_rot_last, cMul(cTranspose(g_dev_rot_last), g_dev_rot_cur)));
        }
        if(!_pressed){
            g_cam_rot_last = g_camera->getGlobalRot();
            g_coordApp->m_hapticDevices[0].m_hDevice->getRotation(g_dev_rot_last);
        }
    }

    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    g_bulletWorld->updateShadowMaps(false, mirroredDisplay);

    // render world
    g_camera->renderView(g_width, g_height);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));
}

// Function to fix time dilation

double compute_dt(bool adjust_int_steps = false){
    double dt = g_clockWorld.getCurrentTimeSeconds() - g_bulletWorld->getSimulationTime();
    int min_steps = 2;
    int max_steps = 10;
    if (adjust_int_steps){
        if (dt >= g_bulletWorld->getIntegrationTimeStep() * min_steps){
            int int_steps_max =  dt / g_bulletWorld->getIntegrationTimeStep();
            if (int_steps_max > max_steps){
                int_steps_max = max_steps;
            }
            g_bulletWorld->setIntegrationMaxIterations(int_steps_max + min_steps);        }
    }
    return dt;
}

//---------------------------------------------------------------------------

void updateBulletSim(){
    g_simulationRunning = true;
    g_simulationFinished = false;

    // start haptic device
    g_clockWorld.start(true);
    // main Bullet simulation loop
    int n = g_coordApp->m_num_devices;
    cVector3d dpos[n], ddpos[n], dposLast[n];
    cMatrix3d drot[n], ddrot[n], drotLast[n];

    for(int i = 0 ; i < n; i ++){
        dpos[i].set(0,0,0); ddpos[i].set(0,0,0); dposLast[i].set(0,0,0);
        drot[i].identity(); ddrot[i].identity(); drotLast[i].identity();
    }
    RateSleep rateSleep(1000);
    while(g_simulationRunning)
    {
        // signal frequency counter
        g_freqCounterHaptics.signal(1);
        double dt;
        if (g_dt_fixed > 0.0) dt = g_dt_fixed;
        else dt = compute_dt(true);
        for (int i = 0 ; i<g_coordApp->m_num_devices ; i++){
            // update position of tool
            g_coordApp->m_bulletTools[i].update_measured_pose();

            dposLast[i] = dpos[i];
            dpos[i] = g_coordApp->m_bulletTools[i].m_posRef - g_coordApp->m_bulletTools[i].m_posGripper;
            ddpos[i] = (dpos[i] - dposLast[i]) / dt;

            drotLast[i] = drot[i];
            drot[i] = cTranspose(g_coordApp->m_bulletTools[i].m_rotGripper) * g_coordApp->m_bulletTools[i].m_rotRef;
            ddrot[i] = (cTranspose(drot[i]) * drotLast[i]);

            double angle, dangle;
            cVector3d axis, daxis;
            drot[i].toAxisAngle(axis, angle);
            ddrot[i].toAxisAngle(daxis, dangle);

            cVector3d force, torque;

            force = g_coordApp->m_bulletTools[i].K_lc * dpos[i] +
                    (g_coordApp->m_bulletTools[i].B_lc) * ddpos[i];
            torque = (g_coordApp->m_bulletTools[i].K_ac * angle) * axis;
            g_coordApp->m_bulletTools[i].m_rotGripper.mul(torque);

            g_coordApp->m_bulletTools[i].apply_force(force);
            g_coordApp->m_bulletTools[i].apply_torque(torque);
        }
        g_bulletWorld->updateDynamics(dt, g_clockWorld.getCurrentTimeSeconds(), g_freqCounterHaptics.getFrequency(), g_coordApp->m_num_devices);
        g_coordApp->clear_all_haptics_loop_exec_flags();
        rateSleep.sleep();
    }
    g_simulationFinished = true;
}


void updateHaptics(void* a_arg){
    int i = *(int*) a_arg;
    // simulation in now running
    g_simulationRunning = true;
    g_simulationFinished = false;

    // update position and orientation of tool
    Device *hDev = & g_coordApp->m_hapticDevices[i];
    ToolGripper* bGripper = & g_coordApp->m_bulletTools[i];
    hDev->m_posDeviceClutched.set(0.0,0.0,0.0);
    hDev->measured_rot();
    hDev->m_rotDeviceClutched.identity();
    bGripper->m_rotRefLast = hDev->m_rotDevice;

    cVector3d dpos, ddpos, dposLast;
    cMatrix3d drot, ddrot, drotLast;
    dpos.set(0,0,0); ddpos.set(0,0,0); dposLast.set(0,0,0);
    drot.identity(); ddrot.identity(); drotLast.identity();

    double K_lc_offset = 10;
    double K_ac_offset = 1;
    double B_lc_offset = 1;
    double B_ac_offset = 1;
    double K_lh_offset = 5;
    double K_ah_offset = 1;

    // main haptic simulation loop
    while(g_simulationRunning)
    {
        hDev->m_freq_ctr.signal(1);
        // Adjust time dilation by computing dt from clockWorld time and the simulationTime
        double dt;
        if (g_dt_fixed > 0.0) dt = g_dt_fixed;
        else dt = compute_dt();

        hDev->m_posDevice = hDev->measured_pos();
        hDev->m_rotDevice = hDev->measured_rot();

        if(bGripper->m_gripper_pinch_btn >= 0){
            bGripper->set_gripper_angle(hDev->measured_gripper_angle());
            if(hDev->is_button_pressed(bGripper->m_gripper_pinch_btn)){
                hDev->enable_force_feedback(true);
            }
        }

        if(hDev->is_button_press_rising_edge(bGripper->mode_next_btn)) g_coordApp->next_mode();
        if(hDev->is_button_press_rising_edge(bGripper->mode_prev_btn)) g_coordApp->prev_mode();

        bool btn_1_rising_edge = hDev->is_button_press_rising_edge(bGripper->act_1_btn);
        bool btn_1_falling_edge = hDev->is_button_press_falling_edge(bGripper->act_1_btn);
        bool btn_2_rising_edge = hDev->is_button_press_rising_edge(bGripper->act_2_btn);
        bool btn_2_falling_edge = hDev->is_button_press_falling_edge(bGripper->act_2_btn);

        double gripper_offset = 0;
        switch (g_coordApp->m_simModes){
        case MODES::CAM_CLUTCH_CONTROL:
            g_clutch_btn_pressed  = hDev->is_button_pressed(bGripper->act_1_btn);
            g_cam_btn_pressed     = hDev->is_button_pressed(bGripper->act_2_btn);
            if(g_clutch_btn_pressed) g_btn_action_str = "Clutch Pressed";
            if(g_cam_btn_pressed)   {g_btn_action_str = "Cam Pressed";}
            if(btn_1_falling_edge || btn_2_falling_edge) g_btn_action_str = "";
            break;
        case MODES::GRIPPER_JAW_CONTROL:
            if (btn_1_rising_edge) gripper_offset = 0.1;
            if (btn_2_rising_edge) gripper_offset = -0.1;
            bGripper->offset_gripper_angle(gripper_offset);
            break;
        case MODES::CHANGE_CONT_LIN_GAIN:
            if(btn_1_rising_edge) g_coordApp->increment_K_lc(K_lc_offset);
            if(btn_2_rising_edge) g_coordApp->increment_K_lc(-K_lc_offset);
            break;
        case MODES::CHANGE_CONT_ANG_GAIN:
            if(btn_1_rising_edge) g_coordApp->increment_K_ac(K_ac_offset);
            if(btn_2_rising_edge) g_coordApp->increment_K_ac(-K_ac_offset);
            break;
        case MODES::CHANGE_CONT_LIN_DAMP:
            if(btn_1_rising_edge) g_coordApp->increment_B_lc(B_lc_offset);
            if(btn_2_rising_edge) g_coordApp->increment_B_lc(-B_lc_offset);
            break;
        case MODES::CHANGE_CONT_ANG_DAMP:
            if(btn_1_rising_edge) g_coordApp->increment_B_ac(B_ac_offset);
            if(btn_2_rising_edge) g_coordApp->increment_B_ac(-B_ac_offset);
            break;
        case MODES::CHANGE_DEV_LIN_GAIN:
            if(btn_1_rising_edge) g_coordApp->increment_K_lh(K_lh_offset);
            if(btn_2_rising_edge) g_coordApp->increment_K_lh(-K_lh_offset);
            break;
        case MODES::CHANGE_DEV_ANG_GAIN:
            if(btn_1_rising_edge) g_coordApp->increment_K_ah(K_ah_offset);
            if(btn_2_rising_edge) g_coordApp->increment_K_ah(-K_ah_offset);
            break;
        }


        if(g_cam_btn_pressed){
            if(bGripper->btn_cam_rising_edge){
                bGripper->btn_cam_rising_edge = false;
                bGripper->m_posRefLast = bGripper->m_posRef / bGripper->m_workspaceScaleFactor;
                bGripper->m_rotRefLast = bGripper->m_rotRef;
            }
            hDev->m_posDeviceClutched = hDev->m_posDevice;
            hDev->m_rotDeviceClutched = hDev->m_rotDevice;
        }
        else{
            bGripper->btn_cam_rising_edge = true;
        }
        if(g_clutch_btn_pressed){
            if(bGripper->btn_clutch_rising_edge){
                bGripper->btn_clutch_rising_edge = false;
                bGripper->m_posRefLast = bGripper->m_posRef / bGripper->m_workspaceScaleFactor;
                bGripper->m_rotRefLast = bGripper->m_rotRef;
            }
            hDev->m_posDeviceClutched = hDev->m_posDevice;
            hDev->m_rotDeviceClutched = hDev->m_rotDevice;
        }
        else{
            bGripper->btn_clutch_rising_edge = true;
        }

        bGripper->m_posRef = bGripper->m_posRefLast +
                (g_camera->getLocalRot() * (hDev->m_posDevice - hDev->m_posDeviceClutched));
        if (!g_coordApp->m_use_cam_frame_rot){
            bGripper->m_rotRef = bGripper->m_rotRefLast * g_camera->getLocalRot() *
                    cTranspose(hDev->m_rotDeviceClutched) * hDev->m_rotDevice *
                    cTranspose(g_camera->getLocalRot());
        }
        else{
            bGripper->m_rotRef = hDev->m_rotDevice;
        }
        bGripper->m_posRef.mul(bGripper->m_workspaceScaleFactor);

        // update position of tool
       bGripper->update_measured_pose();

        dposLast = dpos;
        dpos = bGripper->m_posRef - bGripper->m_posGripper;
        ddpos = (dpos - dposLast) / dt;

        drotLast = drot;
        drot = cTranspose(bGripper->m_rotGripper) * bGripper->m_rotRef;
        ddrot = (cTranspose(drot) * drotLast);

        double angle, dangle;
        cVector3d axis, daxis;
        drot.toAxisAngle(axis, angle);
        ddrot.toAxisAngle(daxis, dangle);

        cVector3d force, torque;

        force  = - g_force_enable * bGripper->K_lh_ramp * (bGripper->K_lc * dpos + (bGripper->B_lc) * ddpos);
        torque = - g_force_enable * bGripper->K_ah_ramp * ((bGripper->K_ac * angle) * axis);

        hDev->apply_wrench(force, torque);

        if (bGripper->K_lh_ramp < bGripper->K_lh)
        {
            bGripper->K_lh_ramp = bGripper->K_lh_ramp + 0.1 * dt * bGripper->K_lh;
        }
        else
        {
            bGripper->K_lh_ramp = bGripper->K_lh;
        }

        if (bGripper->K_ah_ramp < bGripper->K_ah)
        {
            bGripper->K_ah_ramp = bGripper->K_ah_ramp + 0.1 * dt * bGripper->K_ah;
        }
        else
        {
            bGripper->K_ah_ramp = bGripper->K_ah;
        }
        bGripper->set_loop_exec_flag();
    }
    // exit haptics thread
}
