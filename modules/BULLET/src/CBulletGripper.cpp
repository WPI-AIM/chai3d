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

#define PI 3.14159

bool g_2 =  true;
bool g_1a = false;
bool g_2a = false;

// root resource path
std::string resourceRootGripper;
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRootGripper+std::string(p)).c_str())

namespace chai3d {

cBulletGripper::cBulletGripper(cBulletWorld *bulletWorld, std::string a_gripperName, GripperType a_type):cBulletMultiMesh(bulletWorld, a_gripperName){

    std::string hres_path_pre = "../resources/models/gripper/high_res/";
    std::string lres_path_pre = "../resources/models/gripper/low_res/";
    std::string _link_1_str = "";
    std::string _link_2_str = "";
    std::string hres_file, lres_file;
    cMultiMesh lowResColMesh;

    if (a_type == GripperType::SINGLE_JOINT){
        g_2 =  true;
        g_1a = false;
        g_2a = false;
        _link_1_str = "SingleLinkL1.STL";
        gScale = 1.0;

        j1_low_lim = 178.0 * (PI / 180.0);
        j1_high_lim = 120.0 * (PI / 180.0);
    }

    if (a_type == GripperType::MULTI_JOINT){
        g_2 =  true;
        g_1a = true;
        g_2a = true;
        _link_1_str = "MultiLinkL1.STL";
        _link_2_str = "MultiLinkL2.STL";

        gScale = 1.0;
        gA = 15 * (PI / 180);

        j1_low_lim = 150.0 * (PI / 180.0);
        j1_high_lim = 90.0 * (PI / 180.0);

        l1_len = 0.27;

    }
    hres_file = hres_path_pre + _link_1_str;
    lres_file = lres_path_pre + _link_1_str;
    loadFromFile(RESOURCE_PATH(hres_file.c_str()));
    lowResColMesh.loadFromFile(RESOURCE_PATH(lres_file.c_str()));
    scale(gScale);
    lowResColMesh.scale(gScale);
    buildContactTriangles(0.001, &lowResColMesh);
    setLocalPos(0.0,0.0,0.0);
    cMatrix3d rotMat;

    if(g_2){
    link2 = new cBulletMultiMesh(bulletWorld);
    hres_file = hres_path_pre + _link_1_str;
    lres_file = lres_path_pre + _link_1_str;
    link2->loadFromFile(RESOURCE_PATH(hres_file.c_str()));
    lowResColMesh.loadFromFile(RESOURCE_PATH(lres_file.c_str()));
    link2->scale(gScale);
    lowResColMesh.scale(gScale);
    link2->buildContactTriangles(0.001, &lowResColMesh);
    link2->setLocalPos(0.0,0.0,0.0);
    rotMat.setAxisAngleRotationDeg(1,0,0,180);
    link2->setLocalRot(rotMat);
    }

    if (g_1a){
    link1a = new cBulletMultiMesh(bulletWorld);
    hres_file = hres_path_pre + _link_2_str;
    lres_file = lres_path_pre + _link_2_str;
    link1a->loadFromFile(RESOURCE_PATH(hres_file.c_str()));
    lowResColMesh.loadFromFile(RESOURCE_PATH(lres_file.c_str()));
    link1a->scale(gScale);
    lowResColMesh.scale(gScale);
    link1a->buildContactTriangles(0.001, &lowResColMesh);
    link1a->setLocalPos(-gScale * l1_len * cos(gA), -gScale * l1_len * sin(gA), 0.0);
    rotMat.setAxisAngleRotationDeg(1,0,0,0);
    link1a->setLocalRot(rotMat);
    j1a_low_lim = -15.0 * (PI / 180.0);
    j1a_high_lim = 30.0 * (PI / 180.0);
    }

    if (g_2a){
    link2a = new cBulletMultiMesh(bulletWorld);
    hres_file = hres_path_pre + _link_2_str;
    lres_file = lres_path_pre + _link_2_str;
    link2a->loadFromFile(RESOURCE_PATH(hres_file.c_str()));
    lowResColMesh.loadFromFile(RESOURCE_PATH(lres_file.c_str()));
    link2a->scale(gScale);
    lowResColMesh.scale(gScale);
    link2a->buildContactTriangles(0.001, &lowResColMesh);
    link2a->setLocalPos(gScale * l1_len * cos(gA), gScale * l1_len * sin(gA), 0.0);
    rotMat.setAxisAngleRotationDeg(1,0,0,180);
    link2a->setLocalRot(rotMat);
    j2a_low_lim = -15.0 * (PI / 180.0);
    j2a_high_lim = 30.0 * (PI / 180.0);
    }
}

void cBulletGripper::build(){

    double a_mass = 0.05;
    setMass(a_mass);
//    setShowFrame(true);
    estimateInertia();
    buildDynamicModel();
    m_dynamicWorld->addChild(this);
    mat.setBlack();
    setMaterial(mat);

    if (g_2){
    link2->setMass(a_mass);
//    link2->setShowFrame(true);
    link2->estimateInertia();
    link2->buildDynamicModel();
    m_dynamicWorld->addChild(link2);
    mat.setBlueMediumSlate();
    link2->setMaterial(mat);


    axis1.setValue(0.0,0.0,1.0);
    axis2 = -axis1;
    pvt1.setValue(0.0,0.0,0.0);
    pvt2.setValue(0.0,0.0,0.0);
    hinge1 = new btHingeConstraint(*this->m_bulletRigidBody,
                                        *link2->m_bulletRigidBody,
                                        pvt1, pvt2, axis1, axis2, true);

    m_dynamicWorld->m_bulletWorld->addConstraint(hinge1, true);
    hinge1->enableMotor(true);
    hinge1->setMaxMotorImpulse(0.05);
    hinge1->setLimit(j1_low_lim, j1_high_lim);
    }

    if (g_1a){
    link1a->setMass(a_mass);
//    link1a->setShowFrame(true);
    link1a->estimateInertia();
    link1a->buildDynamicModel();
    m_dynamicWorld->addChild(link1a);
    mat.setPurpleBlueViolet();
    link1a->setMaterial(mat);

    axis1a.setValue(0.0,0.0,1.0);
    axis1aa = axis1a;
    pvt1a.setValue(-gScale * l1_len * cos(gA), -gScale * l1_len * sin(gA), 0.0);
    pvt1aa.setValue(0.0,0.0,0.0);
    hinge1a = new btHingeConstraint(*this->m_bulletRigidBody,
                                        *link1a->m_bulletRigidBody,
                                        pvt1a, pvt1aa, axis1a, axis1aa, true);

    m_dynamicWorld->m_bulletWorld->addConstraint(hinge1a, true);
    hinge1a->enableMotor(true);
    hinge1a->setMaxMotorImpulse(0.05);
    hinge1a->setLimit(j1a_low_lim, j1a_high_lim);
    }

    if (g_2a){
    link2a->setMass(a_mass);
//    link2a->setShowFrame(true);
    link2a->estimateInertia();
    link2a->buildDynamicModel();
    m_dynamicWorld->addChild(link2a);
    mat.setOrangeCoral();
    link2a->setMaterial(mat);

    axis2a.setValue(0.0,0.0,1.0);
    axis2aa = axis2a;
    pvt2a.setValue(-gScale * l1_len * cos(gA), -gScale * l1_len * sin(gA), 0.0);
    pvt2aa.setValue(0.0,0.0,0.0);
    hinge2a = new btHingeConstraint(*link2->m_bulletRigidBody,
                                        *link2a->m_bulletRigidBody,
                                        pvt2a, pvt2aa, axis2a, axis2aa, true);


    m_dynamicWorld->m_bulletWorld->addConstraint(hinge2a, true);
    hinge2a->enableMotor(true);
    hinge2a->setMaxMotorImpulse(0.05);
    hinge2a->setLimit(j2a_low_lim, j2a_high_lim);
    }

    GripperSurfaceProperties props;
    props.set_default();
    props.lin_damping = 0.5;
    set_surface_props(props);
}

void cBulletGripper::set_gripper_angle(const double &angle, double dt){
    double clipped_angle = cClamp(angle, 0.0, 1.0);
    if (g_2){
        double j1_angle = j1_low_lim + clipped_angle * (j1_high_lim - j1_low_lim);
        hinge1->setMotorTarget(j1_angle, dt);
    }
    if (g_1a){
        double j1a_angle = j1a_low_lim + clipped_angle * (j1a_high_lim - j1a_low_lim);
        hinge1a->setMotorTarget(j1a_angle, dt);
    }
    if (g_2a){
        double j2a_angle = j2a_low_lim + clipped_angle * (j2a_high_lim - j2a_low_lim);
        hinge2a->setMotorTarget(j2a_angle, dt);
    }
}

void cBulletGripper::set_scale(double a_scale){
    // Do nothing for now
}

void cBulletGripper::set_surface_props(GripperSurfaceProperties &props){

    this->m_bulletRigidBody->setDamping(props.lin_damping, props.ang_damping);
    this->m_bulletRigidBody->setFriction(props.friction);
    this->m_bulletRigidBody->setRollingFriction(props.rolling_friction);

    if (g_2){
    link2->m_bulletRigidBody->setDamping(props.lin_damping, props.ang_damping);
    link2->m_bulletRigidBody->setFriction(props.friction);
    link2->m_bulletRigidBody->setRollingFriction(props.rolling_friction);
    }

    if (g_1a){
    link1a->m_bulletRigidBody->setDamping(props.lin_damping, props.ang_damping);
    link1a->m_bulletRigidBody->setFriction(props.friction);
    link1a->m_bulletRigidBody->setRollingFriction(props.rolling_friction);
    }

    if (g_2a){
    link2a->m_bulletRigidBody->setDamping(props.lin_damping, props.ang_damping);
    link2a->m_bulletRigidBody->setFriction(props.friction);
    link2a->m_bulletRigidBody->setRollingFriction(props.rolling_friction);
    }

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
