
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
    \Motivation: https://www.gamedev.net/articles/programming/engines-and-middleware/yaml-basics-and-parsing-with-yaml-cpp-r3508/
    \version   3.2.1 $Rev: 2161 $
*/
//==============================================================================
#include "CBulletMultiBody.h"

#define PI 3.14159
// root resource path
std::string resourceRootMB;
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRootMB+std::string(p)).c_str())

namespace chai3d{

///
/// \brief Link::load
/// \param file
/// \param name
/// \return
///
bool Link::load (std::string file, std::string name, cBulletMultiBody* multiBody) {

    YAML::Node baseNode = YAML::LoadFile(file);
    if (baseNode.IsNull()) return false;

    YAML::Node fileNode = baseNode[name];
    if (fileNode.IsNull()) return false;

    if(fileNode["name"].IsDefined())
        m_name = fileNode["name"].as<std::string>();
    if(fileNode["mesh"].IsDefined())
        m_mesh_name = fileNode["mesh"].as<std::string>();
    if(fileNode["mass"].IsDefined())
        m_mass = fileNode["mass"].as<double>();
    if(fileNode["scale"].IsDefined())
        m_scale = fileNode["scale"].as<double>();

    std::string rel_path_high_res = multiBody->high_res_path + m_mesh_name;
    std::string rel_path_low_res = multiBody->low_res_path + m_mesh_name;
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
    if(fileNode["color"].IsDefined()){
        std::string color = fileNode["color"].as<std::string>();
        if (strcmp(color.c_str(), "red") == 0)
            m_mat.setRed();
        else if (strcmp(color.c_str(), "green") == 0)
            m_mat.setGreen();
        else if (strcmp(color.c_str(), "blue") == 0)
            m_mat.setBlue();
        else if (strcmp(color.c_str(), "black") == 0)
            m_mat.setBlack();
        else if (strcmp(color.c_str(), "white") == 0)
            m_mat.setWhite();
        else if (strcmp(color.c_str(), "yellow") == 0)
            m_mat.setYellow();
        else
            printf("Color \"%s\" not defined, using default", color.c_str());
    }
    setMaterial(m_mat);
    multiBody->m_chaiWorld->addChild(this);
    return true;
}

///
/// \brief Joint::assign_vec
/// \param name
/// \param v
/// \param node
///
void Joint::assign_vec(std::string name, btVector3* v, YAML::Node* node){
    v->setX((*node)[name.c_str()]["x"].as<double>());
    v->setY((*node)[name.c_str()]["y"].as<double>());
    v->setZ((*node)[name.c_str()]["z"].as<double>());
    print_vec(name, v);
}

void Joint::print_vec(std::string name, btVector3* v){
    printf("\t -%s: \n "
           "\t\t px = %f \n "
           "\t\t py = %f \n "
           "\t\t pz = %f \n",
           name.c_str(), v->x(), v->y(), v->z());
}

///
/// \brief Joint::load
/// \param filem_name =
/// \param name
/// \return
///
bool Joint::load(std::string file, std::string name, cBulletMultiBody* multiBody){
    YAML::Node baseNode = YAML::LoadFile(file);
    if (baseNode.IsNull()) return false;

    YAML::Node fileNode = baseNode[name];
    if (fileNode.IsNull()) return false;

    m_name = fileNode["name"].as<std::string>();
    m_parent_name = fileNode["parent"].as<std::string>();
    printf("\t -Name of Parent Link = %s \n", m_parent_name.c_str());
    m_child_name = fileNode["child"].as<std::string>();
    printf("\t -Name of Child Link = %s \n", m_child_name.c_str());

    assign_vec("parent_pivot", &m_pvtA,  &fileNode);
    assign_vec("parent_axis", &m_axisA, &fileNode);
    assign_vec("child_pivot", &m_pvtB,  &fileNode);
    assign_vec("child_axis", &m_axisB,  &fileNode);

    btRigidBody* bodyA, * bodyB;
    if (multiBody->m_linkMap.find(m_parent_name.c_str()) != multiBody->m_linkMap.end()
            || multiBody->m_linkMap.find(m_child_name.c_str()) != multiBody->m_linkMap.end()){
        bodyA = multiBody->m_linkMap[m_parent_name.c_str()]->m_bulletRigidBody;
        bodyB = multiBody->m_linkMap[m_child_name.c_str()]->m_bulletRigidBody;
    }
    else{
        std::cerr <<" Couldn't find rigid bodies for joint: " << m_name << std::endl;
        return -1;
    }
    m_hinge = new btHingeConstraint(*bodyA, *bodyB, m_pvtA, m_pvtB, m_axisA, m_axisB, true);
    if (fileNode["enable_motor"].IsDefined()){
        enable_motor = fileNode["enable_motor"].as<int>();
        m_hinge->enableMotor(enable_motor);
    }
    if(fileNode["max_motor_impluse"].IsDefined()){
        max_motor_impluse = fileNode["max_motor_impluse"].as<double>();
        m_hinge->setMaxMotorImpulse(max_motor_impluse);
    }
    else{
        max_motor_impluse = 0.05;
        m_hinge->setMaxMotorImpulse(max_motor_impluse);
    }
    if(fileNode["joint_limits"].IsDefined()){
        jnt_lim_low = fileNode["joint_limits"]["low"].as<double>();
        jnt_lim_high = fileNode["joint_limits"]["high"].as<double>();
        m_hinge->setLimit(jnt_lim_low, jnt_lim_high);
    }
    multiBody->m_chaiWorld->m_bulletWorld->addConstraint(m_hinge);
    return true;
}

///
/// \brief cBulletMultiBody::cBulletMultiBody
/// \param bulletWorld
///
cBulletMultiBody::cBulletMultiBody(cBulletWorld *bulletWorld){
    m_chaiWorld = bulletWorld;
}

///
/// \brief cBulletMultiBody::load_yaml
/// \param file
/// \return
///

bool cBulletMultiBody::load_yaml (std::string file) {

    YAML::Node baseNode = YAML::LoadFile(file);
    if (baseNode.IsNull()) return false; //File Not Found?

    Link *tmpLink;
    if (baseNode["high_res_path"].IsDefined() && baseNode["low_res_path"].IsDefined()){
        high_res_path = baseNode["high_res_path"].as<std::string>();
        low_res_path = baseNode["low_res_path"].as<std::string>();
    }
    else{
        high_res_path = "../resources/models/gripper/high_res/";
        low_res_path = "../resources/models/gripper/low_res/";
    }
    size_t totalLinks = baseNode["links"].size();
    for (size_t i = 0; i < totalLinks; ++i) {
        tmpLink = new Link(m_chaiWorld);
        if (tmpLink->load(file, baseNode["links"][i].as<std::string>(), this))
            m_linkMap[baseNode["links"][i].as<std::string>()] = tmpLink;
    }
    Joint *tmpJoint;
    size_t totalJoints = baseNode["joints"].size();
    for (size_t i = 0; i < totalJoints; ++i) {
        tmpJoint = new Joint();
        std::cout << "Joint name: " << baseNode["joints"][i] << std::endl;
        if (tmpJoint->load(file, baseNode["joints"][i].as<std::string>(), this))
                m_jointMap[baseNode["joints"][i].as<std::string>()] = tmpJoint;
    }

    return true;
}

}

