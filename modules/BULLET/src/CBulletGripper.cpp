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

// root resource path
std::string resourceRootGripper;
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRootGripper+std::string(p)).c_str())
namespace chai3d {

cBulletGripper::cBulletGripper(cBulletWorld *bulletWorld, std::string a_gripperName, GripperType a_type):cBulletMultiMesh(bulletWorld, a_gripperName){

    std::string hres_path_pre = "../resources/models/gripper/high_res/";
    std::string lres_path_pre = "../resources/models/gripper/low_res/";
    std::string _base_link_str = "";
    std::string _link_R1_str = "";
    std::string _link_L1_str = "";
    std::string _link_R2_str = "";
    std::string _link_L2_str = "";
    std::string hres_file, lres_file;
    cMultiMesh lowResColMesh;
    m_gripperType = a_type;
//    m_gripperType = GripperType::SINGLE_JOINT;
    _base_link_str = "BaseLink.STL";
    lBase_len = 0.18;

    if (m_gripperType == GripperType::SINGLE_JOINT){
        _link_R1_str = "SingleLinkR1.STL";
        _link_L1_str = "SingleLinkL1.STL";
        gScale = 1.0;

        jR1_low_lim =  15 * (PI / 180.0);
        jR1_high_lim = -30.0 * (PI / 180.0);
        jL1_low_lim = -jR1_low_lim;
        jL1_high_lim = -jR1_high_lim;
    }

    if (m_gripperType == GripperType::MULTI_JOINT){
        _link_R1_str = "MultiLinkR1.STL";
        _link_L1_str = "MultiLinkL1.STL";
        _link_R2_str = "MultiLinkR2.STL";
        _link_L2_str = "MultiLinkL2.STL";

        gScale = 1.0;
        gA = 15 * (PI / 180);

        jR1_low_lim =  0.0 * (PI / 180.0);
        jR1_high_lim =-40.0 * (PI / 180.0);
        jL1_low_lim = -jR1_low_lim;
        jL1_high_lim = -jR1_high_lim;

        jR2_low_lim = 5.0 * (PI / 180.0);
        jR2_high_lim = -30.0 * (PI / 180.0);
        jL2_low_lim = -jR2_low_lim;
        jL2_high_lim = -jR2_high_lim;

        lR1_len = 0.27;
        lL1_len = 0.27;

    }
    hres_file = hres_path_pre + _base_link_str;
    lres_file = lres_path_pre + _base_link_str;
    loadFromFile(RESOURCE_PATH(hres_file.c_str()));
    lowResColMesh.loadFromFile(RESOURCE_PATH(lres_file.c_str()));
    scale(gScale);
    lowResColMesh.scale(gScale);
    buildContactTriangles(0.001, &lowResColMesh);
    setLocalPos(0.0,0.0,0.0);

    linkR1 = new cBulletMultiMesh(bulletWorld);
    hres_file = hres_path_pre + _link_R1_str;
    lres_file = lres_path_pre + _link_R1_str;
    linkR1->loadFromFile(RESOURCE_PATH(hres_file.c_str()));
    lowResColMesh.loadFromFile(RESOURCE_PATH(lres_file.c_str()));
    linkR1->scale(gScale);
    lowResColMesh.scale(gScale);
    linkR1->buildContactTriangles(0.001, &lowResColMesh);

    linkL1 = new cBulletMultiMesh(bulletWorld);
    hres_file = hres_path_pre + _link_L1_str;
    lres_file = lres_path_pre + _link_L1_str;
    linkL1->loadFromFile(RESOURCE_PATH(hres_file.c_str()));
    lowResColMesh.loadFromFile(RESOURCE_PATH(lres_file.c_str()));
    linkL1->scale(gScale);
    lowResColMesh.scale(gScale);
    linkL1->buildContactTriangles(0.001, &lowResColMesh);

    if (m_gripperType == GripperType::MULTI_JOINT){
        linkR2 = new cBulletMultiMesh(bulletWorld);
        hres_file = hres_path_pre + _link_R2_str;
        lres_file = lres_path_pre + _link_R2_str;
        linkR2->loadFromFile(RESOURCE_PATH(hres_file.c_str()));
        lowResColMesh.loadFromFile(RESOURCE_PATH(lres_file.c_str()));
        linkR2->scale(gScale);
        lowResColMesh.scale(gScale);
        linkR2->buildContactTriangles(0.001, &lowResColMesh);

        linkL2 = new cBulletMultiMesh(bulletWorld);
        hres_file = hres_path_pre + _link_L2_str;
        lres_file = lres_path_pre + _link_L2_str;
        linkL2->loadFromFile(RESOURCE_PATH(hres_file.c_str()));
        lowResColMesh.loadFromFile(RESOURCE_PATH(lres_file.c_str()));
        linkL2->scale(gScale);
        lowResColMesh.scale(gScale);
        linkL2->buildContactTriangles(0.001, &lowResColMesh);
    }

}

void cBulletGripper::build(){

    double a_mass = 0.05;
    setMass(0.02);
//    setShowFrame(true);
    estimateInertia();
    buildDynamicModel();
    m_dynamicWorld->addChild(this);
    mat.setPinkHot();
    setMaterial(mat);

    //////////////////////////////////////////////////////////
    linkR1->setMass(a_mass);
    linkR1->estimateInertia();
    linkR1->buildDynamicModel();
    m_dynamicWorld->addChild(linkR1);
    mat.setBlueMediumSlate();
    linkR1->setMaterial(mat);

    axisB_R1.setValue(0.0,0.0,1.0);
    axisR1_B = axisB_R1;
    pvtB_R1.setValue(0.0, lBase_len / 2.0, 0.0);
    pvtR1_B.setValue(0.0,0.0,0.0);
    hingeB_R1 = new btHingeConstraint(*this->m_bulletRigidBody,
                                    *linkR1->m_bulletRigidBody,
                                    pvtB_R1, pvtR1_B, axisB_R1, axisR1_B, true);

    m_dynamicWorld->m_bulletWorld->addConstraint(hingeB_R1, true);
    hingeB_R1->enableMotor(true);
    hingeB_R1->setMaxMotorImpulse(0.05);
    hingeB_R1->setLimit(jR1_low_lim, jR1_high_lim);

    //////////////////////////////////////////////////////////

    linkL1->setMass(a_mass);
    linkL1->estimateInertia();
    linkL1->buildDynamicModel();
    m_dynamicWorld->addChild(linkL1);
    mat.setPurpleBlueViolet();
    linkL1->setMaterial(mat);

    axisB_L1.setValue(0.0,0.0,1.0);
    axisL1_B = axisB_L1;
    pvtB_L1.setValue(0.0, -lBase_len / 2.0, 0.0);
    pvtL1_B.setValue(0.0,0.0,0.0);
    hingeB_L1 = new btHingeConstraint(*this->m_bulletRigidBody,
                                    *linkL1->m_bulletRigidBody,
                                    pvtB_L1, pvtL1_B, axisB_L1, axisL1_B, true);

    m_dynamicWorld->m_bulletWorld->addConstraint(hingeB_L1, true);
    hingeB_L1->enableMotor(true);
    hingeB_L1->setMaxMotorImpulse(0.05);
    hingeB_L1->setLimit(jL1_low_lim, jL1_high_lim);

    //////////////////////////////////////////////////////////
    if (m_gripperType == GripperType::MULTI_JOINT){
        linkR2->setMass(a_mass);
        linkR2->estimateInertia();
        linkR2->buildDynamicModel();
        m_dynamicWorld->addChild(linkR2);
        mat.setOrangeCoral();
        linkR2->setMaterial(mat);

        axisR1_R2.setValue(0.0,0.0,1.0);
        axisR2_R1 = axisR1_R2;
        pvtR1_R2.setValue(-gScale * lR1_len * cos(gA), gScale * lR1_len * sin(gA), 0.0);
        pvtR2_R1.setValue(0.0,0.0,0.0);
        hingeR1_R2 = new btHingeConstraint(*linkR1->m_bulletRigidBody,
                                           *linkR2->m_bulletRigidBody,
                                           pvtR1_R2, pvtR2_R1, axisR1_R2, axisR2_R1, true);


        m_dynamicWorld->m_bulletWorld->addConstraint(hingeR1_R2, true);
        hingeR1_R2->enableMotor(true);
        hingeR1_R2->setMaxMotorImpulse(0.05);
        hingeR1_R2->setLimit(jR2_low_lim, jR2_high_lim);

        //////////////////////////////////////////////////////////

        linkL2->setMass(a_mass);
        linkL2->estimateInertia();
        linkL2->buildDynamicModel();
        m_dynamicWorld->addChild(linkL2);
        mat.setOrangeCoral();
        linkL2->setMaterial(mat);

        axisL1_L2.setValue(0.0,0.0,1.0);
        axisL2_L1 = axisL1_L2;
        pvtL1_L2.setValue(-gScale * lL1_len * cos(gA), -gScale * lL1_len * sin(gA), 0.0);
        pvtL2_L1.setValue(0.0,0.0,0.0);
        hingeL1_L2 = new btHingeConstraint(*linkL1->m_bulletRigidBody,
                                           *linkL2->m_bulletRigidBody,
                                           pvtL1_L2, pvtL2_L1, axisL1_L2, axisL2_L1, true);


        m_dynamicWorld->m_bulletWorld->addConstraint(hingeL1_L2, true);
        hingeL1_L2->enableMotor(true);
        hingeL1_L2->setMaxMotorImpulse(0.05);
        hingeL1_L2->setLimit(jL2_low_lim, jL2_high_lim);
    }

    //////////////////////////////////////////////////////////

    GripperSurfaceProperties props;
    props.set_default();
    props.lin_damping = 0.5;
    set_surface_props(props);
}

void cBulletGripper::set_gripper_angle(const double &angle, double dt){
    double clipped_angle = cClamp(angle, 0.0, 1.0);
    double j1_angle;
    j1_angle = jR1_low_lim + clipped_angle * (jR1_high_lim - jR1_low_lim);
    hingeB_R1->setMotorTarget(j1_angle, dt);
    j1_angle = jL1_low_lim + clipped_angle * (jL1_high_lim - jL1_low_lim);
    hingeB_L1->setMotorTarget(j1_angle, dt);
    if (m_gripperType == GripperType::MULTI_JOINT){
        j1_angle = jR2_low_lim + clipped_angle * (jR2_high_lim - jR2_low_lim);
        hingeR1_R2->setMotorTarget(j1_angle, dt);
        j1_angle = jL2_low_lim + clipped_angle * (jL2_high_lim - jL2_low_lim);
        hingeL1_L2->setMotorTarget(j1_angle, dt);
    }
}

void cBulletGripper::set_scale(double a_scale){
    // Do nothing for now
}

void cBulletGripper::set_surface_props(GripperSurfaceProperties &props){

    this->m_bulletRigidBody->setDamping(props.lin_damping, props.ang_damping);
    this->m_bulletRigidBody->setFriction(props.friction);
    this->m_bulletRigidBody->setRollingFriction(props.rolling_friction);

    linkR1->m_bulletRigidBody->setDamping(props.lin_damping, props.ang_damping);
    linkR1->m_bulletRigidBody->setFriction(props.friction);
    linkR1->m_bulletRigidBody->setRollingFriction(props.rolling_friction);

    linkL1->m_bulletRigidBody->setDamping(props.lin_damping, props.ang_damping);
    linkL1->m_bulletRigidBody->setFriction(props.friction);
    linkL1->m_bulletRigidBody->setRollingFriction(props.rolling_friction);

    if (m_gripperType == GripperType::MULTI_JOINT){
        linkR2->m_bulletRigidBody->setDamping(props.lin_damping, props.ang_damping);
        linkR2->m_bulletRigidBody->setFriction(props.friction);
        linkR2->m_bulletRigidBody->setRollingFriction(props.rolling_friction);

        linkL2->m_bulletRigidBody->setDamping(props.lin_damping, props.ang_damping);
        linkL2->m_bulletRigidBody->setFriction(props.friction);
        linkL2->m_bulletRigidBody->setRollingFriction(props.rolling_friction);
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
