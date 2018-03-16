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
cBulletMesh* bulletCylinder;
cBulletMultiMesh* bulletGear;
cBulletMultiMesh* bulletTorus;
cBulletMultiMesh* bulletBase;

// bullet static walls and ground
cBulletStaticPlane* bulletGround;

cBulletStaticPlane* bulletBoxWall[5];

cVector3d camPos(0,0,0);
cVector3d dev_vel;
cMatrix3d cam_rot_last, dev_rot_last, dev_rot_cur;
double dt_fixed = 0;
// Default switch index for clutches


//---------------------------------------------------------------------------
// CHAI3D VARIABLES
//---------------------------------------------------------------------------


// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cSpotLight *light;

// a label to display the rates [Hz] at which the simulation is running
cLabel* labelRates;
cLabel* labelDevRates[10];
cLabel* labelTimes;
cLabel* labelModes;
cLabel* labelBtnAction;
std::string btn_action_str = "";
bool cam_btn_pressed = false;
bool clutch_btn_pressed = false;
cPrecisionClock clockWorld;


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
cThread* hapticsThread[10];
// bullet simulation thread
cThread* bulletSimThread;

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
class Sim{
public:
    Sim(){
        workspaceScaleFactor = 30.0;
        K_lh = 0.05;
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
        posRefLast.set(0.0,0.0,0.0);
        rotRefLast.identity();
        m_loop_exec_flag = false;
    }
    void set_sim_params(cHapticDeviceInfo &a_hInfo);
    inline void set_loop_exec_flag(){m_loop_exec_flag=true;}
    inline void clear_loop_exec_flag(){m_loop_exec_flag = false;}
    inline bool is_loop_exec(){return m_loop_exec_flag;}
    inline double get_workspace_scale_factor(){return workspaceScaleFactor;}
    cVector3d posRef, posRefLast;
    cMatrix3d rotRef, rotRefLast;
    double workspaceScaleFactor;
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
    bool btn_cam_rising_edge;
    bool btn_clutch_rising_edge;
    bool m_loop_exec_flag;
};

void Sim::set_sim_params(cHapticDeviceInfo &a_hInfo){
    double maxStiffness	= a_hInfo.m_maxLinearStiffness / workspaceScaleFactor;

    // clamp the force output gain to the max device stiffness
    K_lh = cMin(K_lh, maxStiffness / K_lc);
    if (strcmp(a_hInfo.m_modelName.c_str(), "MTM-R") == 0 || strcmp(a_hInfo.m_modelName.c_str(), "MTMR") == 0 ||
        strcmp(a_hInfo.m_modelName.c_str(), "MTM-L") == 0 || strcmp(a_hInfo.m_modelName.c_str(), "MTML") == 0)
    {
        std::cout << "Device " << a_hInfo.m_modelName << " DETECTED, CHANGING BUTTON AND WORKSPACE MAPPING" << std::endl;
        workspaceScaleFactor = 10.0;
        K_lh = K_lh/3;
        act_1_btn     =  1;
        act_2_btn     =  2;
        mode_next_btn =  3;
        mode_prev_btn =  4;
    }

    if (strcmp(a_hInfo.m_modelName.c_str(), "Falcon") == 0)
    {
        std::cout << "Device " << a_hInfo.m_modelName << " DETECTED, CHANGING BUTTON AND WORKSPACE MAPPING" << std::endl;
        act_1_btn     = 0;
        act_2_btn     = 2;
        mode_next_btn = 3;
        mode_prev_btn = 1;
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

class ToolGripper: public Sim, public DataExchange{
public:
    ToolGripper(){gripper_angle = 3.0;}
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
    cVector3d posGripper;
    cMatrix3d rotGripper;
    double gripper_angle;

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
    posGripper  = gripper->getLocalPos();
    rotGripper = gripper->getLocalRot();
}

void ToolGripper::set_gripper_angle(double angle){
    if(!gripper->m_af_pos_ctrl_active) gripper->set_gripper_angle(angle);
}

void ToolGripper::offset_gripper_angle(double offset){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    gripper_angle += offset;
    gripper_angle = cClamp(gripper_angle, 0.0, 1.0);
    gripper->set_gripper_angle(gripper_angle);
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
    virtual void apply_wrench(cVector3d force, cVector3d torque);
    virtual bool is_button_pressed(int button_index);
    virtual bool is_button_press_rising_edge(int button_index);
    virtual bool is_button_press_falling_edge(int button_index);
    cShapeSphere* create_cursor();
    cGenericHapticDevicePtr hDevice;
    cHapticDeviceInfo hInfo;
    cVector3d posDevice, posDeviceClutched, velDevice, avelDevice;
    cMatrix3d rotDevice, rotDeviceClutched;
    cVector3d force, torque;
    double _m_workspace_scale_factor;
    cShapeSphere* m_cursor;
    bool m_btn_prev_state_rising[10] = {false};
    bool m_btn_prev_state_falling[10] = {false};
    cFrequencyCounter m_freq_ctr;

private:
    boost::mutex m_mutex;
};

cShapeSphere* Device::create_cursor(){
    m_cursor = new cShapeSphere(0.05);
    m_cursor->setShowEnabled(true);
    m_cursor->setShowFrame(true);
    m_cursor->setFrameSize(0.1);
    cMaterial mat;
    mat.setGreenLightSea();
    m_cursor->setMaterial(mat);
    return m_cursor;
}

cVector3d Device::measured_pos(){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    hDevice->getPosition(posDevice);
    m_cursor->setLocalPos(posDevice * _m_workspace_scale_factor);
    return posDevice;
}

cMatrix3d Device::measured_rot(){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    hDevice->getRotation(rotDevice);
    m_cursor->setLocalRot(rotDevice);
    return rotDevice;
}

void Device::update_measured_pose(){

}

cVector3d Device::measured_lin_vel(){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    hDevice->getLinearVelocity(velDevice);
    return velDevice;
}

cVector3d Device::mearured_ang_vel(){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    hDevice->getAngularVelocity(avelDevice);
    return avelDevice;
}

double Device::measured_gripper_angle(){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    double angle;
    hDevice->getGripperAngleRad(angle);
    return angle;
}

bool Device::is_button_pressed(int button_index){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    bool status;
    hDevice->getUserSwitch(button_index, status);
    return status;
}

bool Device::is_button_press_rising_edge(int button_index){
    boost::lock_guard<boost::mutex> lock(m_mutex);
    bool status;
    hDevice->getUserSwitch(button_index, status);
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
    hDevice->getUserSwitch(button_index, status);
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
    hDevice->setForceAndTorqueAndGripperForce(force, torque, 0.0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

    cHapticDeviceHandler *device_handler;
    ToolGripper bulletTools[MAX_DEVICES];
    Device hapticDevices[MAX_DEVICES];
    uint m_num_devices;
    cBulletWorld* m_bullet_world;

    // bool to enable the rotation of tool be in camera frame. i.e. Orienting the camera
    // re-orients the tool.
    bool _useCamFrameRot;
    MODES m_mode;
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
    m_bullet_world = NULL;
    m_bullet_world = a_bullet_world;
    device_handler = new cHapticDeviceHandler();
    m_num_devices = device_handler->getNumDevices();
    std::cerr << "Num of devices " << m_num_devices << std::endl;
    if (a_max_load_devs < m_num_devices) m_num_devices = a_max_load_devs;
    for (uint i = 0; i < m_num_devices; i++){
        retrieve_device_handle(i);
        create_bullet_gripper(i);
    }
    _useCamFrameRot = true;
    m_mode = CAM_CLUTCH_CONTROL;
    m_mode_str = "CAM_CLUTCH_CONTROL";
    m_mode_idx = 0;
}

void Coordination::next_mode(){
    m_mode_idx = (m_mode_idx + 1) % m_modes_enum_vec.size();
    m_mode = m_modes_enum_vec[m_mode_idx];
    m_mode_str = m_modes_enum_str[m_mode_idx];
    btn_action_str = "";
    cam_btn_pressed = false;
    clutch_btn_pressed = false;
    std::cout << m_mode_str << std::endl;
}


void Coordination::prev_mode(){
    m_mode_idx = (m_mode_idx - 1) % m_modes_enum_vec.size();
    m_mode = m_modes_enum_vec[m_mode_idx];
    m_mode_str = m_modes_enum_str[m_mode_idx];
    btn_action_str = "";
    cam_btn_pressed = false;
    clutch_btn_pressed = false;
    std::cout << m_mode_str << std::endl;
}

bool Coordination::retrieve_device_handle(uint dev_num){
    device_handler->getDeviceSpecifications(hapticDevices[dev_num].hInfo, dev_num);
    return device_handler->getDevice(hapticDevices[dev_num].hDevice, dev_num);
}

void Coordination::create_bullet_gripper(uint dev_num){
    std::ostringstream dev_str;
    dev_str << (dev_num + 1);
    std::string gripper_name = "Gripper" + dev_str.str();
    bulletTools[dev_num].gripper = new cBulletGripper(m_bullet_world, gripper_name);
    bulletTools[dev_num].set_sim_params(hapticDevices[dev_num].hInfo);
    bulletTools[dev_num].gripper->build();
    m_bullet_world->addChild(bulletTools[dev_num].gripper);
    hapticDevices[dev_num]._m_workspace_scale_factor = bulletTools[dev_num].get_workspace_scale_factor();
}

void Coordination::open_devices(){
    for (int i = 0 ; i < m_num_devices ; i++){
        hapticDevices[i].hDevice->open();
        std::string name = "Device" + std::to_string(i+1);
        m_bullet_world->addChild(hapticDevices[i].create_cursor());
    }
}

void Coordination::close_devices(){
    for (int i = 0 ; i < m_num_devices ; i++){
        hapticDevices[i].hDevice->close();
    }
}

int Coordination::num_of_haptics_loop_execd(){
    int num_devs_loop_execd = 0;
    for (int i = 0 ; i < m_num_devices ; i++){
        if (bulletTools[i].is_loop_exec()) num_devs_loop_execd++;
    }
    return num_devs_loop_execd;
}

bool Coordination::are_all_haptics_loop_exec(){
    bool flag = true;
    for (int i = 0 ; i < m_num_devices ; i++){
        flag &= bulletTools[i].is_loop_exec();
    }
    return flag;
}

void Coordination::clear_all_haptics_loop_exec_flags(){
    for (int i = 0 ; i < m_num_devices ; i++){
        bulletTools[i].clear_loop_exec_flag();
    }
}

double Coordination::increment_K_lh(double a_offset){
    for (int i = 0 ; i < m_num_devices ; i++){
        if (bulletTools[i].K_lh + a_offset <= 0)
        {
            bulletTools[i].K_lh = 0.0;
        }
        else{
            bulletTools[i].K_lh += a_offset;
        }
    }
    //Set the return value to the gain of the last device
    if(m_num_devices > 0){
        a_offset = bulletTools[m_num_devices-1].K_lh;
        btn_action_str = "K_lh = " + cStr(a_offset, 4);
    }
    return a_offset;
}

double Coordination::increment_K_ah(double a_offset){
    for (int i = 0 ; i < m_num_devices ; i++){
        if (bulletTools[i].K_ah + a_offset <=0){
            bulletTools[i].K_ah = 0.0;
        }
        else{
            bulletTools[i].K_ah += a_offset;
        }
    }
    //Set the return value to the gain of the last device
    if(m_num_devices > 0){
        a_offset = bulletTools[m_num_devices-1].K_ah;
        btn_action_str = "K_ah = " + cStr(a_offset, 4);
    }
    return a_offset;
}

double Coordination::increment_K_lc(double a_offset){
    for (int i = 0 ; i < m_num_devices ; i++){
        if (bulletTools[i].K_lc + a_offset <=0){
            bulletTools[i].K_lc = 0.0;
        }
        else{
            bulletTools[i].K_lc += a_offset;
        }
    }
    //Set the return value to the stiffness of the last device
    if(m_num_devices > 0){
        a_offset = bulletTools[m_num_devices-1].K_lc;
        btn_action_str = "K_lc = " + cStr(a_offset, 4);
    }
    return a_offset;
}

double Coordination::increment_K_ac(double a_offset){
    for (int i = 0 ; i < m_num_devices ; i++){
        if (bulletTools[i].K_ac + a_offset <=0){
            bulletTools[i].K_ac = 0.0;
        }
        else{
            bulletTools[i].K_ac += a_offset;
        }
    }
    //Set the return value to the stiffness of the last device
    if(m_num_devices > 0){
        a_offset = bulletTools[m_num_devices-1].K_ac;
        btn_action_str = "K_ac = " + cStr(a_offset, 4);
    }
    return a_offset;
}

double Coordination::increment_B_lc(double a_offset){
    for (int i = 0 ; i < m_num_devices ; i++){
        if (bulletTools[i].B_lc + a_offset <=0){
            bulletTools[i].B_lc = 0.0;
        }
        else{
            bulletTools[i].B_lc += a_offset;
        }
    }
    //Set the return value to the stiffness of the last device
    if(m_num_devices > 0){
        a_offset = bulletTools[m_num_devices-1].B_lc;
        btn_action_str = "B_lc = " + cStr(a_offset, 4);
    }
    return a_offset;
}

double Coordination::increment_B_ac(double a_offset){
    for (int i = 0 ; i < m_num_devices ; i++){
        if (bulletTools[i].B_ac + a_offset <=0){
            bulletTools[i].B_ac = 0.0;
        }
        else{
            bulletTools[i].B_ac += a_offset;
        }
    }
    //Set the return value to the stiffness of the last device
    if(m_num_devices > 0){
        a_offset = bulletTools[m_num_devices-1].B_ac;
        btn_action_str = "B_ac = " + cStr(a_offset, 4);
    }
    return a_offset;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
class RateSleep{
public:
    RateSleep(int a_freq){
        cycle_time = 1.0 / double(a_freq);
        m_clock.start();
        next_expected_time = m_clock.getCurrentTimeSeconds() + cycle_time;
    }
  bool sleep(){
      double cur_time = m_clock.getCurrentTimeSeconds();
      if (cur_time >= next_expected_time){
          next_expected_time = cur_time + cycle_time;
          return true;
      }
      while(m_clock.getCurrentTimeSeconds() <= next_expected_time){

      }
      next_expected_time = m_clock.getCurrentTimeSeconds() + cycle_time;
      return true;
  }
private:
  double next_expected_time;
  double cycle_time;
  cPrecisionClock m_clock;
};


std::shared_ptr<Coordination> coordPtr;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//===========================================================================
/*
    DEMO:    08-bullet-tool.cpp

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
    cout << "Demo: 08-bullet-coordination" << endl;
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
    bulletWorld = new cBulletWorld("World");

    // set the background color of the environment
    bulletWorld->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    camera = new cCamera(bulletWorld);
    bulletWorld->addChild(camera);

    // position and orient the camera
    camera->set(cVector3d(3.0, 0.0, 0.3),    // camera position (eye)
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


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFontPtr font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    labelTimes = new cLabel(font);
    labelModes = new cLabel(font);
    labelBtnAction = new cLabel(font);
    labelRates->m_fontColor.setBlack();
    labelTimes->m_fontColor.setBlack();
    labelModes->m_fontColor.setBlack();
    labelBtnAction->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelRates);
    camera->m_frontLayer->addChild(labelTimes);
    camera->m_frontLayer->addChild(labelModes);
    camera->m_frontLayer->addChild(labelBtnAction);

    //////////////////////////////////////////////////////////////////////////
    // BULLET WORLD
    //////////////////////////////////////////////////////////////////////////
    // set some gravity
    bulletWorld->setGravity(cVector3d(0.0, 0.0, -9.8));


    //////////////////////////////////////////////////////////////////////////
    // 3 BULLET BLOCKS
    //////////////////////////////////////////////////////////////////////////
    double size = 0.40;
    cMaterial meshMat;

    bulletGear = new cBulletMultiMesh(bulletWorld, "Gear");
    bulletGear->loadFromFile(RESOURCE_PATH("../resources/models/gear/gear.3ds"));
    bulletGear->scale(0.0014);
    bulletWorld->addChild(bulletGear);
    bulletGear->buildContactTriangles(0.001);
    bulletGear->setMass(0.3);
    bulletGear->estimateInertia();
    bulletGear->buildDynamicModel();
    meshMat.setPinkDeep();
    bulletGear->setMaterial(meshMat);
    bulletGear->m_bulletRigidBody->setFriction(1);

    bulletTorus = new cBulletMultiMesh(bulletWorld, "Torus");
    bulletTorus->loadFromFile(RESOURCE_PATH("../resources/models/gear/torus.3ds"));
    bulletTorus->scale(0.2);
    bulletTorus->setLocalPos(cVector3d(0.3,0,0));
    bulletWorld->addChild(bulletTorus);
    bulletTorus->buildContactTriangles(0.001);
    bulletTorus->setMass(0.8);
    bulletTorus->estimateInertia();
    bulletTorus->buildDynamicModel();
    meshMat.setOrangeTomato();
    bulletTorus->setMaterial(meshMat);

    bulletBase = new cBulletMultiMesh(bulletWorld, "Base");
    bulletBase->loadFromFile(RESOURCE_PATH("../resources/models/gear/base.3ds"));
    bulletBase->scale(0.3);
    bulletBase->setLocalPos(cVector3d(-0.3,0,0));
    bulletWorld->addChild(bulletBase);
    bulletBase->buildContactTriangles(0.001);
    bulletBase->setMass(10);
    bulletBase->estimateInertia();
    bulletBase->buildDynamicModel();
    meshMat.setBlueNavy();
    bulletBase->setMaterial(meshMat);
    bulletBase->m_bulletRigidBody->setFriction(1);

    if (argc > 1) coordPtr = std::make_shared<Coordination>(bulletWorld, std::atoi(argv[1]));
    else coordPtr = std::make_shared<Coordination>(bulletWorld);

    if (argc > 2) dt_fixed = atof(argv[2]);
    usleep(100);


    //////////////////////////////////////////////////////////////////////////
    // INVISIBLE WALLS
    //////////////////////////////////////////////////////////////////////////

    // we create 5 static walls to contain the dynamic objects within a limited workspace
    double planeWidth = 1.0;
    bulletBoxWall[0] = new cBulletStaticPlane(bulletWorld, cVector3d(0.0, 0.0, -1.0), -2.0 * planeWidth);
    bulletBoxWall[1] = new cBulletStaticPlane(bulletWorld, cVector3d(0.0, -1.0, 0.0), -1.5*planeWidth);
    bulletBoxWall[2] = new cBulletStaticPlane(bulletWorld, cVector3d(0.0, 1.0, 0.0), -1.5*planeWidth);
    bulletBoxWall[3] = new cBulletStaticPlane(bulletWorld, cVector3d(-1.0, 0.0, 0.0), -planeWidth);
    bulletBoxWall[4] = new cBulletStaticPlane(bulletWorld, cVector3d(1.0, 0.0, 0.0), -0.8 * planeWidth);

    for (int i = 0 ; i < 5 ; i++){
        cVector3d worldZ;
        worldZ.set(0,0,1);
        cVector3d planeOri = cCross(bulletBoxWall[i]->getPlaneNormal(), worldZ);
        cMatrix3d planeRot;
        planeRot.setAxisAngleRotationDeg(planeOri, 90);
        worldZ.set(0,0,1);
        bulletWorld->addChild(bulletBoxWall[i]);
        cCreatePlane(bulletBoxWall[i], 2.0, 3.0,
                     bulletBoxWall[i]->getPlaneConstant() * bulletBoxWall[i]->getPlaneNormal(),
                     planeRot);
        cMaterial matPlane;
        matPlane.setBlueSky();
        bulletBoxWall[i]->setMaterial(matPlane);
        bulletBoxWall[i]->setTransparencyLevel(0.5, true, true);
    }


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
    matGround.setGreenChartreuse();
    matGround.m_emission.setGrayLevel(0.3);
    bulletGround->setMaterial(matGround);
    bulletGround->m_bulletRigidBody->setFriction(1);

    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------
    coordPtr->open_devices();

    // create a thread which starts the main haptics rendering loop
    int dev_num[10] = {0,1,2,3,4,5,6,7,8,9};
    for (int i = 0 ; i < coordPtr->m_num_devices ; i++){
        hapticsThread[i] = new cThread();
        hapticsThread[i]->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS, &dev_num[i]);
    }
    //create a thread which starts the Bullet Simulation loop
    bulletSimThread = new cThread();
    bulletSimThread->start(updateBulletSim, CTHREAD_PRIORITY_HAPTICS);

    for (int i = 0 ; i < coordPtr->m_num_devices ; i++){
        labelDevRates[i] = new cLabel(font);
        labelDevRates[i]->m_fontColor.setBlack();
        labelDevRates[i]->setFontScale(0.8);
        camera->m_frontLayer->addChild(labelDevRates[i]);
    }

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
        cout << "Keyboard Options:" << endl << endl;    coordPtr->close_devices();
        cout << "[h] - Display help menu" << endl;
        cout << "[1] - Enable gravity" << endl;
        cout << "[2] - Disable gravity" << endl << endl;    coordPtr->close_devices();
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
        printf("linear haptic gain:  %f\n", coordPtr->increment_K_lh(-0.05));
    }

    // option - increase linear haptic gain
    else if (a_key == GLFW_KEY_4)
    {
        printf("linear haptic gain:  %f\n", coordPtr->increment_K_lh(0.05));
    }

    // option - decrease angular haptic gain
    else if (a_key == GLFW_KEY_5)
    {
        printf("angular haptic gain:  %f\n", coordPtr->increment_K_ah(-0.05));
    }

    // option - increase angular haptic gain
    else if (a_key == GLFW_KEY_6)
    {
        printf("angular haptic gain:  %f\n", coordPtr->increment_K_ah(0.05));
    }

    // option - decrease linear stiffness
    else if (a_key == GLFW_KEY_7)
    {
        printf("linear stiffness:  %f\n", coordPtr->increment_K_lc(-50));
    }

    // option - increase linear stiffness
    else if (a_key == GLFW_KEY_8)
    {
        printf("linear stiffness:  %f\n", coordPtr->increment_K_lc(50));
    }

    // option - decrease angular stiffness
    else if (a_key == GLFW_KEY_9)
    {
        printf("angular stiffness:  %f\n", coordPtr->increment_K_ac(-1));
    }

    // option - increase angular stiffness
    else if (a_key == GLFW_KEY_0)
    {
        printf("angular stiffness:  %f\n", coordPtr->increment_K_ac(1));
    }
    else if (a_key == GLFW_KEY_C){
        coordPtr->_useCamFrameRot = true;
        printf("Gripper Rotation w.r.t Camera Frame:\n");
    }
    else if (a_key == GLFW_KEY_W){
        coordPtr->_useCamFrameRot = false;
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
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // delete resources
    coordPtr->close_devices();
    for(int i = 0 ; i < coordPtr->m_num_devices ; i ++){delete hapticsThread[i];}
    delete bulletWorld;
    delete coordPtr->device_handler;
}

//---------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic and graphic rate data
    labelTimes->setText("Wall Time: " + cStr(clockWorld.getCurrentTimeSeconds(),2) + " s" +
                        + " / "+" Simulation Time: " + cStr(bulletWorld->getSimulationTime(),2) + " s");
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " + cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");
    labelModes->setText("MODE: " + coordPtr->m_mode_str);
    labelBtnAction->setText(" : " + btn_action_str);

    for (int i = 0 ; i < coordPtr->m_num_devices ; i++){
        labelDevRates[i]->setText(coordPtr->hapticDevices[i].hInfo.m_modelName + ": " + cStr(coordPtr->hapticDevices[i].m_freq_ctr.getFrequency(), 0) + " Hz");
        labelDevRates[i]->setLocalPos(10, (int)(height - (i+1)*20));
    }

    // update position of label
    labelTimes->setLocalPos((int)(0.5 * (width - labelTimes->getWidth())), 30);
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 10);
    labelModes->setLocalPos((int)(0.5 * (width - labelModes->getWidth())), 50);
    labelBtnAction->setLocalPos((int)(0.5 * (width - labelModes->getWidth()) + labelModes->getWidth()), 50);
    bool _pressed;
    if (coordPtr->m_num_devices > 0){
        coordPtr->hapticDevices[0].hDevice->getUserSwitch(coordPtr->bulletTools[0].act_2_btn, _pressed);
        if(_pressed && coordPtr->m_mode == MODES::CAM_CLUTCH_CONTROL){
            double scale = 0.3;
            dev_vel = coordPtr->hapticDevices[0].measured_lin_vel();
            coordPtr->hapticDevices[0].hDevice->getRotation(dev_rot_cur);
            camera->setLocalPos(camera->getLocalPos() + cMul(scale, cMul(camera->getGlobalRot(),dev_vel)));
            camera->setLocalRot(cMul(cam_rot_last, cMul(cTranspose(dev_rot_last), dev_rot_cur)));
        }
        if(!_pressed){
            cam_rot_last = camera->getGlobalRot();
            coordPtr->hapticDevices[0].hDevice->getRotation(dev_rot_last);
        }
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

// Function to fix time dilation

double compute_dt(bool adjust_int_steps = false){
    double dt = clockWorld.getCurrentTimeSeconds() - bulletWorld->getSimulationTime();
    int min_steps = 2;
    int max_steps = 10;
    if (adjust_int_steps){
        if (dt >= bulletWorld->getIntegrationTimeStep() * min_steps){
            int int_steps_max =  dt / bulletWorld->getIntegrationTimeStep();
            if (int_steps_max > max_steps){
                int_steps_max = max_steps;
            }
            bulletWorld->setIntegrationMaxIterations(int_steps_max + min_steps);        }
    }
    return dt;
}

//---------------------------------------------------------------------------

void updateBulletSim(){
    simulationRunning = true;
    simulationFinished = false;

    // start haptic device
    clockWorld.start(true);
    // main Bullet simulation loop
    int n = coordPtr->m_num_devices;
    cVector3d dpos[n], ddpos[n], dposLast[n];
    cMatrix3d drot[n], ddrot[n], drotLast[n];

    for(int i = 0 ; i < n; i ++){
        dpos[i].set(0,0,0); ddpos[i].set(0,0,0); dposLast[i].set(0,0,0);
        drot[i].identity(); ddrot[i].identity(); drotLast[i].identity();
    }
    RateSleep rateSleep(1000);
    while(simulationRunning)
    {
        // signal frequency counter
        freqCounterHaptics.signal(1);
        double dt;
        if (dt_fixed > 0.0) dt = dt_fixed;
        else dt = compute_dt(true);
        for (int i = 0 ; i<coordPtr->m_num_devices ; i++){
            // update position of tool
            coordPtr->bulletTools[i].update_measured_pose();

            dposLast[i] = dpos[i];
            dpos[i] = coordPtr->bulletTools[i].posRef - coordPtr->bulletTools[i].posGripper;
            ddpos[i] = (dpos[i] - dposLast[i]) / dt;

            drotLast[i] = drot[i];
            drot[i] = cTranspose(coordPtr->bulletTools[i].rotGripper) * coordPtr->bulletTools[i].rotRef;
            ddrot[i] = (cTranspose(drot[i]) * drotLast[i]);

            double angle, dangle;
            cVector3d axis, daxis;
            drot[i].toAxisAngle(axis, angle);
            ddrot[i].toAxisAngle(daxis, dangle);

            cVector3d force, torque;

            force = coordPtr->bulletTools[i].K_lc * dpos[i] +
                    (coordPtr->bulletTools[i].B_lc) * ddpos[i];
            torque = (coordPtr->bulletTools[i].K_ac * angle) * axis;
            coordPtr->bulletTools[i].rotGripper.mul(torque);

            coordPtr->bulletTools[i].apply_force(force);
            coordPtr->bulletTools[i].apply_torque(torque);
        }
        bulletWorld->updateDynamics(dt, clockWorld.getCurrentTimeSeconds(), freqCounterHaptics.getFrequency(), coordPtr->m_num_devices);
        coordPtr->clear_all_haptics_loop_exec_flags();
        rateSleep.sleep();
    }
    simulationFinished = true;
}


void updateHaptics(void* a_arg){
    int i = *(int*) a_arg;
    // simulation in now running
    simulationRunning = true;
    simulationFinished = false;

    // update position and orientation of tool
    Device *hDev = & coordPtr->hapticDevices[i];
    ToolGripper* bGripper = & coordPtr->bulletTools[i];
    hDev->posDeviceClutched.set(0.0,0.0,0.0);
    hDev->measured_rot();
    hDev->rotDeviceClutched.identity();
    bGripper->rotRefLast = hDev->rotDevice;

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
    while(simulationRunning)
    {
        hDev->m_freq_ctr.signal(1);
        // Adjust time dilation by computing dt from clockWorld time and the simulationTime
        double dt;
        if (dt_fixed > 0.0) dt = dt_fixed;
        else dt = compute_dt();

        // compute global reference frames for each object
        bGripper->set_gripper_angle(hDev->measured_gripper_angle());

        hDev->posDevice = hDev->measured_pos();
        hDev->rotDevice = hDev->measured_rot();

        if(hDev->is_button_press_rising_edge(bGripper->mode_next_btn)) coordPtr->next_mode();
        if(hDev->is_button_press_rising_edge(bGripper->mode_prev_btn)) coordPtr->prev_mode();

        bool btn_1_rising_edge = hDev->is_button_press_rising_edge(bGripper->act_1_btn);
        bool btn_1_falling_edge = hDev->is_button_press_falling_edge(bGripper->act_1_btn);
        bool btn_2_rising_edge = hDev->is_button_press_rising_edge(bGripper->act_2_btn);
        bool btn_2_falling_edge = hDev->is_button_press_falling_edge(bGripper->act_2_btn);

        double gripper_offset = 0;
        switch (coordPtr->m_mode){
        case MODES::CAM_CLUTCH_CONTROL:
            clutch_btn_pressed  = hDev->is_button_pressed(bGripper->act_1_btn);
            cam_btn_pressed     = hDev->is_button_pressed(bGripper->act_2_btn);
            if(clutch_btn_pressed) btn_action_str = "Clutch Pressed";
            if(cam_btn_pressed)   {btn_action_str = "Cam Pressed";}
            if(btn_1_falling_edge || btn_2_falling_edge) btn_action_str = "";
            break;
        case MODES::GRIPPER_JAW_CONTROL:
            if (btn_1_rising_edge) gripper_offset = 0.1;
            if (btn_2_rising_edge) gripper_offset = -0.1;
            bGripper->offset_gripper_angle(gripper_offset);
            break;
        case MODES::CHANGE_CONT_LIN_GAIN:
            if(btn_1_rising_edge) coordPtr->increment_K_lc(K_lc_offset);
            if(btn_2_rising_edge) coordPtr->increment_K_lc(-K_lc_offset);
            break;
        case MODES::CHANGE_CONT_ANG_GAIN:
            if(btn_1_rising_edge) coordPtr->increment_K_ac(K_ac_offset);
            if(btn_2_rising_edge) coordPtr->increment_K_ac(-K_ac_offset);
            break;
        case MODES::CHANGE_CONT_LIN_DAMP:
            if(btn_1_rising_edge) coordPtr->increment_B_lc(B_lc_offset);
            if(btn_2_rising_edge) coordPtr->increment_B_lc(-B_lc_offset);
            break;
        case MODES::CHANGE_CONT_ANG_DAMP:
            if(btn_1_rising_edge) coordPtr->increment_B_ac(B_ac_offset);
            if(btn_2_rising_edge) coordPtr->increment_B_ac(-B_ac_offset);
            break;
        case MODES::CHANGE_DEV_LIN_GAIN:
            if(btn_1_rising_edge) coordPtr->increment_K_lh(K_lh_offset);
            if(btn_2_rising_edge) coordPtr->increment_K_lh(-K_lh_offset);
            break;
        case MODES::CHANGE_DEV_ANG_GAIN:
            if(btn_1_rising_edge) coordPtr->increment_K_ah(K_ah_offset);
            if(btn_2_rising_edge) coordPtr->increment_K_ah(-K_ah_offset);
            break;
        }


        if(cam_btn_pressed){
            if(bGripper->btn_cam_rising_edge){
                bGripper->btn_cam_rising_edge = false;
                bGripper->posRefLast = bGripper->posRef / bGripper->workspaceScaleFactor;
                bGripper->rotRefLast = bGripper->rotRef;
            }
            hDev->posDeviceClutched = hDev->posDevice;
            hDev->rotDeviceClutched = hDev->rotDevice;
        }
        else{
            bGripper->btn_cam_rising_edge = true;
        }
        if(clutch_btn_pressed){
            if(bGripper->btn_clutch_rising_edge){
                bGripper->btn_clutch_rising_edge = false;
                bGripper->posRefLast = bGripper->posRef / bGripper->workspaceScaleFactor;
                bGripper->rotRefLast = bGripper->rotRef;
            }
            hDev->posDeviceClutched = hDev->posDevice;
            hDev->rotDeviceClutched = hDev->rotDevice;
        }
        else{
            bGripper->btn_clutch_rising_edge = true;
        }

        bGripper->posRef = bGripper->posRefLast +
                (camera->getLocalRot() * (hDev->posDevice - hDev->posDeviceClutched));
        if (!coordPtr->_useCamFrameRot){
            bGripper->rotRef = bGripper->rotRefLast * camera->getLocalRot() *
                    cTranspose(hDev->rotDeviceClutched) * hDev->rotDevice *
                    cTranspose(camera->getLocalRot());
        }
        else{
            bGripper->rotRef = hDev->rotDevice;
        }
        bGripper->posRef.mul(bGripper->workspaceScaleFactor);

        // update position of tool
       bGripper->update_measured_pose();

        dposLast = dpos;
        dpos = bGripper->posRef - bGripper->posGripper;
        ddpos = (dpos - dposLast) / dt;

        drotLast = drot;
        drot = cTranspose(bGripper->rotGripper) * bGripper->rotRef;
        ddrot = (cTranspose(drot) * drotLast);

        double angle, dangle;
        cVector3d axis, daxis;
        drot.toAxisAngle(axis, angle);
        ddrot.toAxisAngle(daxis, dangle);

        cVector3d force, torque;

        force = bGripper->K_lc * dpos + (bGripper->B_lc) * ddpos;
        torque = (bGripper->K_ac * angle) * axis;

        force  = - bGripper->K_lh_ramp * force;
        torque = - bGripper->K_ah_ramp * torque;
        force.set(0,0,0);
        torque.set(0,0,0);

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
