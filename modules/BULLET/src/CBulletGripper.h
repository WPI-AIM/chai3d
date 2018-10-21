//==============================================================================
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
    \version   3.2.0 $Rev: 2161 $
*/
//==============================================================================

#ifndef CBulletGripper_H
#define CBulletGripper_H

#include "chai3d.h"
#include "CBulletMultiMesh.h"
#include "CBullet.h"

namespace chai3d {

class cBulletGripper;
typedef std::shared_ptr<cBulletGripper> cBulletGripperPtr;

struct GripperSurfaceProperties{
public:
    GripperSurfaceProperties(){
        lin_damping = 0.0;
        ang_damping = 0.0;
        friction = 0.0;
        rolling_friction = 0.0;
    }
    void set_default(){
        lin_damping = 0.2;
        ang_damping = 1.0;
        friction = 0.5;
        rolling_friction = 0.5;
    }

    double lin_damping;
    double ang_damping;
    double friction;
    double rolling_friction;
};

enum GripperType{
    SINGLE_JOINT,
    MULTI_JOINT
};

class cBulletWorld;

struct cMultiBodyLink{
    btVector3* parentAxis, * childAxes;
    btVector3* parentPvt, * childPvt;
    double *jnt_lim_low, *jnt_lim_high;
    cBulletMultiMesh* link;
    btHinge2Constraint* hinge;
};

class cBulletGripper:public cBulletMultiMesh{
public:
    cBulletGripper(cBulletWorld *bulletWorld, std::string gripper_name = "Gripper", GripperType a_type = GripperType::MULTI_JOINT);
    ~cBulletGripper(){}

public:
    void set_gripper_angle(const double &angle,double dt=0.001);
    void set_surface_props(GripperSurfaceProperties &props);
    void set_scale(double scale);
    void build();
    virtual void updateCmdFromROS(double dt=0.001);
public:
    cBulletMultiMesh* linkR1, *linkL1, *linkR2, *linkL2;
    btHingeConstraint* hingeB_R1, *hingeB_L1, *hingeR1_R2, *hingeL1_L2;

private:
    btVector3 axisB_R1, axisB_L1, axisR1_R2, axisL1_L2;
    btVector3 axisR1_B, axisL1_B, axisR2_R1, axisL2_L1;
    btVector3 pvtB_R1, pvtB_L1, pvtR1_R2, pvtL1_L2;
    btVector3 pvtR1_B, pvtL1_B, pvtR2_R1, pvtL2_L1;
    cMaterial mat;
    double jR1_low_lim, jR1_high_lim;
    double jL1_low_lim, jL1_high_lim;
    double jR2_low_lim, jR2_high_lim;
    double jL2_low_lim, jL2_high_lim;
    double lBase_len, lR1_len, lL1_len;

    double gScale;
    double gA;
    GripperType m_gripperType;
};

}

#endif
