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
    \version   3.2.1 $Rev: 2161 $
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

///
/// \brief afGripper::setGripperAngle
/// \param angle
/// \param dt
///
void afGripper::setGripperAngle(const double &angle, double dt){
}

///
/// \brief afGripper::loadMultiBody
/// \param a_file
/// \param a_gripper_name
/// \param a_suffix_name
/// \return
///
bool afGripper::loadMultiBody(std::string a_file,
                                                   std::string a_gripper_name,
                                                   std::string a_suffix_name){
    m_gripper_name = a_gripper_name;
    m_suffix_name = a_suffix_name;

    YAML::Node multiBodyNode = YAML::LoadFile(a_file);
    if (!multiBodyNode){
        std::cerr << "FAILED TO LOAD YAML CONFIG FILE \n";
        return NULL;
    }

    YAML::Node multiBodyMeshPathHR = multiBodyNode["high resolution path"];
    YAML::Node multiBodyMeshPathLR = multiBodyNode["low resolution path"];
    YAML::Node multiBodyNameSpace = multiBodyNode["namespace"];
    YAML::Node multiBodyRidigBodies = multiBodyNode["bodies"];
    YAML::Node multiBodyJoints = multiBodyNode["joints"];


    std::string color_config;
    if (multiBodyNode["color config"].IsDefined())
        s_colorsNode = YAML::LoadFile(multiBodyNode["color config"].as<std::string>().c_str());

    afGripperLinkPtr tmpBody;
    if(multiBodyMeshPathHR.IsDefined() && multiBodyMeshPathLR.IsDefined()){
        m_multibody_high_res_path = multiBodyMeshPathHR.as<std::string>();
        m_multibody_low_res_path = multiBodyMeshPathLR.as<std::string>();
    }
    else{
        m_multibody_high_res_path = "../resources/models/puzzle/high_res/";
        m_multibody_low_res_path = "../resources/models/puzzle/low_res/";
    }
    if (multiBodyNameSpace.IsDefined()){
        m_multibody_namespace = multiBodyNameSpace.as<std::string>();
    }
    else{
        m_multibody_namespace = "/chai/env/";
    }

    size_t totalBodys = multiBodyRidigBodies.size();
    for (size_t i = 0; i < totalBodys; ++i) {
        tmpBody = new afGripperLink(m_chaiWorld);
        std::string body_name = multiBodyRidigBodies[i].as<std::string>();
//        printf("Loading body: %s \n", body_name .c_str());
        if (tmpBody->load(a_file.c_str(), body_name, this)){
            m_afRigidBodyMap[body_name.c_str()] = tmpBody;
        }
    }
    afJoint *tmpJoint;
    size_t totalJoints = multiBodyJoints.size();
    for (size_t i = 0; i < totalJoints; ++i) {
        tmpJoint = new afJoint();
        std::string jnt_name = multiBodyJoints[i].as<std::string>();
//        printf("Loading body: %s \n", jnt_name.c_str());
        if (tmpJoint->load(a_file.c_str(), jnt_name, this)){
            m_afJointMap[jnt_name] = tmpJoint;
        }
    }

    m_rootLink = static_cast<afGripperLinkPtr>(afMultiBody::getRootRigidBody());
    if (m_rootLink == NULL){
        std::cerr << "WARNING, NO ROOT PARENT EXISTS \n";
    }
    else{
        std::string sfx = m_suffix_name;
        sfx.erase(remove_if(sfx.begin(), sfx.end(), isspace), sfx.end());
        m_rootLink->createAFObject(m_gripper_name + sfx, m_multibody_namespace);
    }

    return true;
}

///
/// \brief afGripper::getRootRigidBody
/// \return
///
afGripperLinkPtr afGripper::getRootRigidBody(){
    if (m_rootLink == NULL){
        std::cerr << "WARNING, NO ROOT PARENT EXISTS \n";
    }
    return m_rootLink;
}

///
/// \brief afGripper::~afGripper
///
afGripper::~afGripper(){
    afRigidBodyMap::const_iterator lIt = m_afRigidBodyMap.begin();
    for ( ; lIt != m_afRigidBodyMap.end() ; ++lIt){
        delete lIt->second;
    }
    afJointMap::const_iterator jIt = m_afJointMap.begin();
    for (; jIt != m_afJointMap.end() ; ++jIt){
        delete jIt->second;
    }
    }
}
