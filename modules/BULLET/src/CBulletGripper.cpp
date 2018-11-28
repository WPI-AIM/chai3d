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
/// \brief cBulletGripper::set_gripper_angle
/// \param angle
/// \param dt
///
void afBulletGripper::set_gripper_angle(const double &angle, double dt){
}

///
/// \brief cBulletGripper::load_multibody
/// \param file
/// \return
///
afBulletGripperLink* afBulletGripper::load_multibody(std::string a_file,
                                                   std::string a_gripper_name,
                                                   std::string a_suffix_name){
    m_gripper_name = a_gripper_name;
    m_suffix_name = a_suffix_name;

    YAML::Node multiBodyNode = YAML::LoadFile(a_file);
    if (!multiBodyNode){
        std::cerr << "FAILED TO LOAD YAML CONFIG FILE \n";
        return NULL;
    }
    std::string color_config;
    if (multiBodyNode["color_config"].IsDefined())
        m_colorsNode = YAML::LoadFile(multiBodyNode["color_config"].as<std::string>().c_str());

    afBulletGripperLink *tmpBody;
    if (multiBodyNode["high_res_path"].IsDefined() && multiBodyNode["low_res_path"].IsDefined()){
        high_res_path = multiBodyNode["high_res_path"].as<std::string>();
        low_res_path = multiBodyNode["low_res_path"].as<std::string>();
    }
    else{
        high_res_path = "../resources/models/gripper/high_res/";
        low_res_path = "../resources/models/gripper/low_res/";
    }
    size_t totalBodys = multiBodyNode["bodies"].size();
    std::vector<std::string> temp_body_names;
    for (size_t i = 0; i < totalBodys; ++i) {
        tmpBody = new afBulletGripperLink(m_chaiWorld);
        std::string body_name = multiBodyNode["bodies"][i].as<std::string>();
//        printf("Loading body: %s \n", body_name .c_str());
        if (tmpBody->load(a_file.c_str(), body_name, this)){
            m_bodyMap[body_name.c_str()] = tmpBody;
            temp_body_names.push_back(body_name.c_str());
        }
    }
    afJoint *tmpJoint;
    size_t totalJoints = multiBodyNode["joints"].size();
    for (size_t i = 0; i < totalJoints; ++i) {
        tmpJoint = new afJoint();
        std::string jnt_name = multiBodyNode["joints"][i].as<std::string>();
//        printf("Loading body: %s \n", jnt_name.c_str());
        if (tmpJoint->load(a_file.c_str(), jnt_name, this)){
            m_jointMap[jnt_name] = tmpJoint;
        }
    }
    afBulletGripperLink* rootParentBody = NULL;
    size_t rootParents = 0;
    std::vector<std::string>::const_iterator nIt;
    cBodyMap::const_iterator mIt;
    for(nIt = temp_body_names.begin() ; nIt != temp_body_names.end() ; ++nIt){
        mIt = m_bodyMap.find(*nIt);
        if((*mIt).second->m_parentBodies.size() == 0){
            rootParentBody = static_cast<afBulletGripperLink*>((*mIt).second);
            rootParents++;
        }
    }
    if (rootParents > 1 || rootParents == 0)
        std::cerr << "WARNING!: " << rootParents << " ROOT PARENTS FOUND, EXPECTED 1\n";
    else{
        std::string sfx = m_suffix_name;
        sfx.erase(remove_if(sfx.begin(), sfx.end(), isspace), sfx.end());
        rootParentBody->createAFObject(m_gripper_name + sfx);
    }

    return rootParentBody;
}

afBulletGripper::~afBulletGripper(){
    cBodyMap::const_iterator lIt = m_bodyMap.begin();
    for ( ; lIt != m_bodyMap.end() ; ++lIt){
        delete lIt->second;
    }
    cJointMap::const_iterator jIt = m_jointMap.begin();
    for (; jIt != m_jointMap.end() ; ++jIt){
        delete jIt->second;
    }
    }
}
