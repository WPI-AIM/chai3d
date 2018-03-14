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
#include "CBulletGripper.h"
#include <string.h>

// root resource path
std::string resourceRootGripper;
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRootGripper+std::string(p)).c_str())

namespace chai3d {

cBulletGripper::cBulletGripper(cBulletWorld *bulletWorld, std::string a_gripperName):cBulletMultiMesh(bulletWorld, a_gripperName){


    loadFromFile(RESOURCE_PATH("../resources/models/gripper/gripper_L1.3ds"));
    scale(0.1);
    setLocalPos(0.0,-0.2,0.0);

    bulletMeshGripperL2 = new cBulletMultiMesh(bulletWorld);
    bulletMeshGripperL2->loadFromFile(RESOURCE_PATH("../resources/models/gripper/gripper_L1.3ds"));
    bulletMeshGripperL2->scale(0.1);
    bulletMeshGripperL2->setLocalPos(0.0,0.2,0.0);
    cMatrix3d rotMat;
    rotMat.setAxisAngleRotationDeg(1,0,0,180);
    bulletMeshGripperL2->setLocalRot(rotMat);
    jaw_open_lim = 2.2;
    jaw_close_lim = 3.13;
}

void cBulletGripper::build(){
    setMass(0.05);
    buildContactTriangles(0.001);
    setShowFrame(true);
    estimateInertia();
    buildDynamicModel();
    m_dynamicWorld->addChild(this);


    bulletMeshGripperL2->setMass(0.05);
    bulletMeshGripperL2->buildContactTriangles(0.001);
    bulletMeshGripperL2->setShowFrame(true);
    bulletMeshGripperL2->estimateInertia();
    bulletMeshGripperL2->buildDynamicModel();
    m_dynamicWorld->addChild(bulletMeshGripperL2);

    axisA.setValue(0.0,0.0,1.0);
    axisB = -axisA;
    pvtA.setValue(0.2,0.1,0.0);
    pvtB.setValue(0.2,0.1,0.0);
    bulletHinge = new btHingeConstraint(*this->m_bulletRigidBody,
                                        *bulletMeshGripperL2->m_bulletRigidBody,
                                        pvtA, pvtB, axisA, axisB, true);

    m_dynamicWorld->m_bulletWorld->addConstraint(bulletHinge, true);
    bulletHinge->enableMotor(true);
    bulletHinge->setMaxMotorImpulse(0.3);
    bulletHinge->setLimit(jaw_open_lim, jaw_close_lim);

    GripperSurfaceProperties props;
    props.set_default();
    props.lin_damping = 0.5;
    set_surface_props(props);

    mat.setBlueLightSteel();
    setMaterial(mat);
    mat.setBlueMediumSlate();
    bulletMeshGripperL2->setMaterial(mat);
}

void cBulletGripper::set_gripper_angle(const double &angle){
    double jaw_angle = cClamp(angle, 0.0, 1.0);
    jaw_angle = jaw_open_lim + (1.0 - angle) * (jaw_close_lim - jaw_open_lim);
    bulletHinge->setMotorTarget(jaw_angle, 0.001);
}

void cBulletGripper::set_scale(double a_scale){
    // Do nothing for now
}

void cBulletGripper::set_surface_props(GripperSurfaceProperties &props){

    this->m_bulletRigidBody->setDamping(props.lin_damping, props.ang_damping);
    this->m_bulletRigidBody->setFriction(props.friction);
    this->m_bulletRigidBody->setRollingFriction(props.rolling_friction);

    bulletMeshGripperL2->m_bulletRigidBody->setDamping(props.lin_damping, props.ang_damping);
    bulletMeshGripperL2->m_bulletRigidBody->setFriction(props.friction);
    bulletMeshGripperL2->m_bulletRigidBody->setRollingFriction(props.rolling_friction);

}

void cBulletGripper::updateCmdFromROS(double dt){
    if (m_rosObjPtr.get() != nullptr){
        m_rosObjPtr->update_af_cmd();
        cVector3d force, torque;
        m_af_pos_ctrl_active = m_rosObjPtr->m_afCmd.pos_ctrl;
        if (m_rosObjPtr->m_afCmd.pos_ctrl){
            cVector3d cur_pos, cmd_pos, rot_axis;
            cQuaternion cur_rot, cmd_rot;
            cMatrix3d cur_rot_mat, cmd_rot_mat;
            btTransform b_trans;
            double rot_angle;
            double K_lin = 10, B_lin = 1;
            double K_ang = 5;
            m_bulletRigidBody->getMotionState()->getWorldTransform(b_trans);
            cur_pos.set(b_trans.getOrigin().getX(),
                        b_trans.getOrigin().getY(),
                        b_trans.getOrigin().getZ());

            cur_rot.x = b_trans.getRotation().getX();
            cur_rot.y = b_trans.getRotation().getY();
            cur_rot.z = b_trans.getRotation().getZ();
            cur_rot.w = b_trans.getRotation().getW();
            cur_rot.toRotMat(cur_rot_mat);

            cmd_pos.set(m_rosObjPtr->m_afCmd.px,
                        m_rosObjPtr->m_afCmd.py,
                        m_rosObjPtr->m_afCmd.pz);

            cmd_rot.x = m_rosObjPtr->m_afCmd.qx;
            cmd_rot.y = m_rosObjPtr->m_afCmd.qy;
            cmd_rot.z = m_rosObjPtr->m_afCmd.qz;
            cmd_rot.w = m_rosObjPtr->m_afCmd.qw;
            cmd_rot.toRotMat(cmd_rot_mat);

            m_dpos_prev = m_dpos;
            m_dpos = cmd_pos - cur_pos;
            m_ddpos = (m_dpos - m_dpos_prev)/dt;
            m_drot = cMul(cTranspose(cur_rot_mat), cmd_rot_mat);
            m_drot.toAxisAngle(rot_axis, rot_angle);

            force = K_lin * m_dpos + B_lin * m_ddpos;
            torque = cMul(K_ang * rot_angle, rot_axis);
            cur_rot_mat.mul(torque);
        }
        else{
            force.set(m_rosObjPtr->m_afCmd.Fx,
                      m_rosObjPtr->m_afCmd.Fy,
                      m_rosObjPtr->m_afCmd.Fz);
            torque.set(m_rosObjPtr->m_afCmd.Nx,
                       m_rosObjPtr->m_afCmd.Ny,
                       m_rosObjPtr->m_afCmd.Nz);
        }
        addExternalForce(force);
        addExternalTorque(torque);
    }
        if (m_rosObjPtr->m_afCmd.size_J_cmd > 0 && m_rosObjPtr->m_afCmd.pos_ctrl){
            set_gripper_angle(m_rosObjPtr->m_afCmd.J_cmd[0]);
        }
    }
}

