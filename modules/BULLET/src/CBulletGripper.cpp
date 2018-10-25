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
/// \brief cBulletGripperLink::load
/// \param file
/// \param a_link_name
/// \param mB
/// \return
///
bool cBulletGripperLink::load(std::string file, std::string a_link_name, cBulletGripper* mB){
    YAML::Node baseNode = YAML::LoadFile(file);
    if (baseNode.IsNull()) return false;

    YAML::Node fileNode = baseNode[a_link_name];
    if (fileNode.IsNull()) return false;

    if(fileNode["name"].IsDefined()){
        m_name = fileNode["name"].as<std::string>();
    }

    if(fileNode["mesh"].IsDefined())
        m_mesh_name = fileNode["mesh"].as<std::string>();

    if(fileNode["mass"].IsDefined()){
        m_mass = fileNode["mass"].as<double>();
        if(fileNode["linear_gain"].IsDefined()){
            K_lin = fileNode["linear_gain"]["P"].as<double>();
            D_lin = fileNode["linear_gain"]["D"].as<double>();
            _lin_gains_computed = true;
        }
        if(fileNode["angular_gain"].IsDefined()){
            K_ang = fileNode["angular_gain"]["P"].as<double>();
            D_ang = fileNode["angular_gain"]["D"].as<double>();
            _ang_gains_computed = true;
        }
    }

    if(fileNode["scale"].IsDefined())
        m_scale = fileNode["scale"].as<double>();

    std::string rel_path_high_res = mB->high_res_path + m_mesh_name;
    std::string rel_path_low_res = mB->low_res_path + m_mesh_name;
    loadFromFile(RESOURCE_PATH(rel_path_high_res));
    m_lowResMesh.loadFromFile(RESOURCE_PATH(rel_path_low_res));
    scale(m_scale);
    m_lowResMesh.scale(m_scale);
    buildContactTriangles(0.001, &m_lowResMesh);
    setMass(m_mass);
    estimateInertia();
    buildDynamicModel();

    if(fileNode["position"].IsDefined()){
        double x = fileNode["position"]["x"].as<double>();
        double y = fileNode["position"]["y"].as<double>();
        double z = fileNode["position"]["z"].as<double>();
        pos.set(x,y,z);
        setLocalPos(pos);
    }

    if(fileNode["rotation"].IsDefined()){
        double r = fileNode["rotation"]["r"].as<double>();
        double p = fileNode["rotation"]["p"].as<double>();
        double y = fileNode["rotation"]["y"].as<double>();
        rot.setExtrinsicEulerRotationRad(y,p,r,cEulerOrder::C_EULER_ORDER_ZXY);
        setLocalRot(rot);
    }

    if(fileNode["color_raw"].IsDefined()){
        m_mat.setColorf(fileNode["color_raw"]["r"].as<float>(),
                fileNode["color_raw"]["g"].as<float>(),
                fileNode["color_raw"]["b"].as<float>(),
                fileNode["color_raw"]["a"].as<float>());
    }
    else if(fileNode["color"].IsDefined()){
        std::vector<double> rgba = mB->get_color_rgba(fileNode["color"].as<std::string>());
        m_mat.setColorf(rgba[0], rgba[1], rgba[2], rgba[3]);
    }

    m_surfaceProps.m_linear_damping = 0.5;
    m_surfaceProps.m_angular_damping = 1.0;
    m_surfaceProps.m_static_friction = 0.5;
    m_surfaceProps.m_rolling_friction = 0.5;

    if (fileNode["damping"]["linear"].IsDefined())
        m_surfaceProps.m_linear_damping = fileNode["damping"]["linear"].as<double>();
    if (fileNode["damping"]["angular"].IsDefined())
        m_surfaceProps.m_angular_damping = fileNode["damping"]["angular"].as<double>();
    if (fileNode["friction"]["static"].IsDefined())
        m_surfaceProps.m_static_friction = fileNode["friction"]["static"].as<double>();
    if (fileNode["friction"]["rolling"].IsDefined())
        m_surfaceProps.m_rolling_friction = fileNode["friction"]["rolling"].as<double>();

    setMaterial(m_mat);
    set_surface_properties(this, &m_surfaceProps);
    mB->m_chaiWorld->addChild(this);
    return true;
}

///
/// \brief cBulletGripper::set_gripper_angle
/// \param angle
/// \param dt
///
void cBulletGripper::set_gripper_angle(const double &angle, double dt){
}

///
/// \brief cBulletGripper::load_multibody
/// \param file
/// \return
///
cBulletGripperLink* cBulletGripper::load_multibody(std::string a_file,
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

    cBulletGripperLink *tmpLink;
    if (multiBodyNode["high_res_path"].IsDefined() && multiBodyNode["low_res_path"].IsDefined()){
        high_res_path = multiBodyNode["high_res_path"].as<std::string>();
        low_res_path = multiBodyNode["low_res_path"].as<std::string>();
    }
    else{
        high_res_path = "../resources/models/gripper/high_res/";
        low_res_path = "../resources/models/gripper/low_res/";
    }
    size_t totalLinks = multiBodyNode["links"].size();
    std::vector<std::string> temp_link_names;
    for (size_t i = 0; i < totalLinks; ++i) {
        tmpLink = new cBulletGripperLink(m_chaiWorld);
        std::string link_name = multiBodyNode["links"][i].as<std::string>();
//        printf("Loading link: %s \n", link_name .c_str());
        if (tmpLink->load(a_file.c_str(), link_name, this)){
            m_linkMap[link_name.c_str()] = tmpLink;
            temp_link_names.push_back(link_name.c_str());
        }
    }
    Joint *tmpJoint;
    size_t totalJoints = multiBodyNode["joints"].size();
    for (size_t i = 0; i < totalJoints; ++i) {
        tmpJoint = new Joint();
        std::string jnt_name = multiBodyNode["joints"][i].as<std::string>();
//        printf("Loading link: %s \n", jnt_name.c_str());
        if (tmpJoint->load(a_file.c_str(), jnt_name, this)){
            m_jointMap[jnt_name] = tmpJoint;
        }
    }
    cBulletGripperLink* rootParentLink = NULL;
    size_t rootParents = 0;
    std::vector<std::string>::const_iterator nIt;
    cLinkMap::const_iterator mIt;
    for(nIt = temp_link_names.begin() ; nIt != temp_link_names.end() ; ++nIt){
        mIt = m_linkMap.find(*nIt);
        if((*mIt).second->m_parentLinks.size() == 0){
            rootParentLink = static_cast<cBulletGripperLink*>((*mIt).second);
            rootParents++;
        }
    }
    if (rootParents > 1 || rootParents == 0)
        std::cerr << "WARNING!: " << rootParents << " ROOT PARENTS FOUND, EXPECTED 1\n";
    else{
        std::string sfx = m_suffix_name;
        sfx.erase(remove_if(sfx.begin(), sfx.end(), isspace), sfx.end());
        rootParentLink->create_af_object(m_gripper_name + sfx);
    }

    return rootParentLink;
}

cBulletGripper::~cBulletGripper(){
    cLinkMap::const_iterator lIt = m_linkMap.begin();
    for ( ; lIt != m_linkMap.end() ; ++lIt){
        delete lIt->second;
    }
    cJointMap::const_iterator jIt = m_jointMap.begin();
    for (; jIt != m_jointMap.end() ; ++jIt){
        delete jIt->second;
    }
    }
}
