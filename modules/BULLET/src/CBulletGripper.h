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

class cBulletWorld;
class cBulletGripper:public cBulletMultiMesh{
public:
    cBulletGripper(cBulletWorld *bulletWorld, std::string gripper_name = "Gripper");
    ~cBulletGripper(){}

public:
    void set_gripper_angle(const double &angle);
    void set_surface_props(GripperSurfaceProperties &props);
    void set_scale(double scale);
    void build();
    virtual void updateCmdFromROS(double dt=0.001);
public:
    cBulletMultiMesh* bulletMeshGripperL2;
    btHingeConstraint* bulletHinge;

private:
    btVector3 axisA, axisB, pvtA, pvtB;
    cMaterial mat;
    double jaw_open_lim, jaw_close_lim;
};

}

#endif
