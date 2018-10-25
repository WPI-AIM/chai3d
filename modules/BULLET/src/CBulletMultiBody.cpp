
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
//==============================================================================
#include "chai3d.h"

#define PI 3.14159
// root resource path
std::string resourceRootMB;
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRootMB+std::string(p)).c_str())

namespace chai3d{

std::map<std::string, std::string> ConfigHandler::m_gripperConfigFiles;
YAML::Node ConfigHandler::m_colorsNode;


///
/// \brief ConfigHandler::ConfigHandler
/// \param a_config_file
///
ConfigHandler::ConfigHandler(){

}

///
/// \brief ConfigHandler::load_yaml
/// \param a_config_file
/// \return
///
bool ConfigHandler::load_yaml(std::string a_config_file){
    YAML::Node configNode = YAML::LoadFile(a_config_file);
    if(!configNode){
        std::cerr << "ERROR! FAILED TO LOAD CONFIG FILE \n";
    }
    // Check if a world config file or a plain multibody config file
    if (configNode["path"].IsDefined() && configNode["multibody_config"].IsDefined()){
        m_path = configNode["path"].as<std::string>();
        m_puzzle_config = m_path + configNode["multibody_config"].as<std::string>();

    }
    else{
        std::cerr << "PATH AND MULTIBODY CONFIG NOT DEFINED \n";
        return 0;
    }
    if(configNode["color_config"].IsDefined()){
        m_color_config = m_path + configNode["color_config"].as<std::string>();
        m_colorsNode = YAML::LoadFile(m_color_config.c_str());
        if (!m_colorsNode){
            std::cerr << "ERROR! COLOR CONFIG NOT FOUND \n";
        }
    }
    else{
        return 0;
    }

    if(configNode["gripper_configs"].IsDefined()){
        for(size_t i=0 ; i < configNode["gripper_configs"].size(); ++i){
            std::string gconf = configNode["gripper_configs"][i].as<std::string>();
            m_gripperConfigFiles[gconf] = m_path + configNode[gconf].as<std::string>();
        }
    }
    else{
        std::cerr << "ERROR! GRIPPER CONFIGS NOT DEFINED \n";
        return 0;
    }
    return 1;
}

///
/// \brief ConfigHandler::get_puzzle_config
/// \return
///
std::string ConfigHandler::get_puzzle_config(){
    return m_puzzle_config;
}

///
/// \brief ConfigHandler::get_color_config
/// \return
///
std::string ConfigHandler::get_color_config(){
    return m_color_config;
}

///
/// \brief ConfigHandler::get_gripper_config
/// \param a_gripper_name
/// \return
///
std::string ConfigHandler::get_gripper_config(std::string a_gripper_name){
    if(m_gripperConfigFiles.find(a_gripper_name) != m_gripperConfigFiles.end()){
        return m_gripperConfigFiles[a_gripper_name];
    }
    else{
        std::cerr << "WARNING! GRIPPER CONFIG FOR \"" << a_gripper_name
                  << "\" NOT FOUND, RETURNING DEFAULT \n";
        return m_gripperConfigFiles["Default"];
    }
}

///
/// \brief ConfigHandler::get_color_rgba
/// \param a_color_name
/// \return
///
std::vector<double> ConfigHandler::get_color_rgba(std::string a_color_name){
    std::vector<double> color_rgba = {0.5, 0.5, 0.5, 0.5};
    // Help from https://stackoverflow.com/questions/15425442/retrieve-random-key-element-for-stdmap-in-c
    if(strcmp(a_color_name.c_str(), "random") == 0 || strcmp(a_color_name.c_str(), "RANDOM") == 0){
        YAML::const_iterator it = m_colorsNode.begin();
        std::advance(it, rand() % m_colorsNode.size());
        color_rgba[0] = it->second["r"].as<int>() / 255.0;
        color_rgba[1] = it->second["g"].as<int>() / 255.0;
        color_rgba[2] = it->second["b"].as<int>() / 255.0;
        color_rgba[3] = it->second["a"].as<int>() / 255.0;
    }
    else if(m_colorsNode[a_color_name].IsDefined()){
        color_rgba[0] = m_colorsNode[a_color_name]["r"].as<int>() / 255.0;
        color_rgba[1] = m_colorsNode[a_color_name]["g"].as<int>() / 255.0;
        color_rgba[2] = m_colorsNode[a_color_name]["b"].as<int>() / 255.0;
        color_rgba[3] = m_colorsNode[a_color_name]["a"].as<int>() / 255.0;
    }
    else{
        std::cerr << "WARNING! COLOR NOT FOUND, RETURNING BALANCED COLOR\n";
    }
    return color_rgba;
}

///
/// \brief Link::Link
/// \param a_world
///
Link::Link(cBulletWorld* a_world): cBulletMultiMesh(a_world){
}

///
/// \brief Link::populate_parent_tree
/// \param a_link
///
void Link::populate_parents_tree(Link* a_link, Joint* a_jnt){
    m_childrenLinks.push_back(a_link);
    m_childrenLinks.insert(m_childrenLinks.end(),
                           a_link->m_childrenLinks.begin(),
                           a_link->m_childrenLinks.end());
    m_joints.push_back(a_jnt);
    m_joints.insert(m_joints.end(),
                           a_link->m_joints.begin(),
                           a_link->m_joints.end());
}

///
/// \brief Link::set_parent_link
/// \param parentLink
///
void Link::add_parent_link(Link* a_parentLink){
    m_parentLinks.push_back(a_parentLink);
}

///
/// \brief Link::set_child_link
/// \param childLink
/// \param jnt
///
void Link::add_child_link(Link* a_childLink, Joint* a_jnt){
    a_childLink->add_parent_link(this);
    m_childrenLinks.push_back(a_childLink);
    m_childrenLinks.insert(m_childrenLinks.end(),
                           a_childLink->m_childrenLinks.begin(),
                           a_childLink->m_childrenLinks.end());
    m_joints.push_back(a_jnt);
    m_joints.insert(m_joints.end(),
                           a_childLink->m_joints.begin(),
                           a_childLink->m_joints.end());
    for (m_linkIt = m_parentLinks.begin() ; m_linkIt != m_parentLinks.end() ; ++m_linkIt){
        (*m_linkIt)->populate_parents_tree(a_childLink, a_jnt);
    }
}

///
/// \brief Link::load
/// \param file
/// \param name
/// \return
///
bool Link::load (std::string file, std::string name, cBulletMultiBody* mB, std::string name_remapping) {
    YAML::Node baseNode = YAML::LoadFile(file);
    if (baseNode.IsNull()) return false;

    YAML::Node fileNode = baseNode[name];
    if (fileNode.IsNull()) return false;

    if(fileNode["name"].IsDefined()){
        m_name = fileNode["name"].as<std::string>();
        create_af_object(m_name);
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
    if(fileNode["friction"].IsDefined())
        m_bulletRigidBody->setFriction(fileNode["friction"].as<double>());

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

    if(fileNode["damping"].IsDefined()){
        setDamping(fileNode["damping"]["linear"].as<double>(), fileNode["damping"]["angular"].as<double>());
    }
    setMaterial(m_mat);
    m_mat.setRed();
    mB->m_chaiWorld->addChild(this);
    return true;
}

///
/// \brief Link::create_af_object
/// \param a_obj_name
///
void Link::create_af_object(std::string a_obj_name){
    m_rosObjPtr.reset(new chai_env::Object(a_obj_name));
}

///
/// \brief Link::compute_gains
///
void Link::compute_gains(){
    if (_lin_gains_computed && _ang_gains_computed){
        return;
    }

    double lumped_mass = m_mass;
    cVector3d lumped_intertia = m_inertia;
    for(m_linkIt = m_childrenLinks.begin() ; m_linkIt != m_childrenLinks.end() ; ++m_linkIt){
        lumped_mass += (*m_linkIt)->getMass();
        lumped_intertia += (*m_linkIt)->getInertia();
    }
    if (!_lin_gains_computed){
        K_lin = lumped_mass * 20;
        D_lin = K_lin / 10;
        _lin_gains_computed = true;
    }
    if (!_ang_gains_computed){
        K_ang = lumped_mass * 10;
        D_ang = K_ang / 2;
        _ang_gains_computed = true;
    }
}

///
/// \brief Link::updateCmdFromROS
/// \param dt
///
void Link::updateCmdFromROS(double dt){
    if (m_rosObjPtr.get() != nullptr){
        m_rosObjPtr->update_af_cmd();
        cVector3d force, torque;
        m_af_pos_ctrl_active = m_rosObjPtr->m_afCmd.pos_ctrl;
        if (m_rosObjPtr->m_afCmd.pos_ctrl){
            compute_gains();
            cVector3d cur_pos, cmd_pos, rot_axis, rot_axix_w_gain;
            cQuaternion cur_rot, cmd_rot;
            cMatrix3d cur_rot_mat, cmd_rot_mat;
            btTransform b_trans;
            double rot_angle;
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
            m_drot_prev = m_drot;
            m_drot = cMul(cTranspose(cur_rot_mat), cmd_rot_mat);
            m_drot.toAxisAngle(rot_axis, rot_angle);

            force = K_lin * m_dpos + D_lin * m_ddpos;
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
        size_t jntCmdSize = m_rosObjPtr->m_afCmd.size_J_cmd;
        if (jntCmdSize > 0 && m_parentLinks.size() == 0){
            size_t jntCnt = m_joints.size() < jntCmdSize ? m_joints.size() : jntCmdSize;
            for (size_t jnt = 0 ; jnt < jntCnt ; jnt++){
                if (m_rosObjPtr->m_afCmd.pos_ctrl)
                    m_joints[jnt]->command_position(m_rosObjPtr->m_afCmd.J_cmd[jnt]);
                else
                    m_joints[jnt]->command_torque(m_rosObjPtr->m_afCmd.J_cmd[jnt]);
            }

        }
    }
}

///
/// \brief Link::set_angle
/// \param angle
/// \param dt
///
void Link::set_angle(double &angle, double dt){
    if (m_parentLinks.size() == 0){
        double clipped_angle = cClamp(angle, 0.0, 1.0);
        for (size_t jnt = 0 ; jnt < m_joints.size() ; jnt++){
            double ang;
            ang = m_joints[jnt]->jnt_lim_low + clipped_angle * (m_joints[jnt]->jnt_lim_high - m_joints[jnt]->jnt_lim_low);
            m_joints[jnt]->m_hinge->setMotorTarget(ang, dt);
        }

    }
}

///
/// \brief Link::set_angle
/// \param angle
/// \param dt
///
void Link::set_angle(std::vector<double> &angles, double dt){
    if (m_parentLinks.size() == 0){
        double jntCmdSize = m_joints.size() < angles.size() ? m_joints.size() : angles.size();
        for (size_t jnt = 0 ; jnt < jntCmdSize ; jnt++){
            double clipped_angle = cClamp(angles[jnt], 0.0, 1.0);
            m_joints[jnt]->m_hinge->setMotorTarget(clipped_angle, dt);
        }

    }
}

///
/// \brief Joint::Joint
///
Joint::Joint(){

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
//    print_vec(name, v);
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
bool Joint::load(std::string file, std::string name, cBulletMultiBody* mB, std::string name_remapping){
    YAML::Node baseNode = YAML::LoadFile(file);
    if (baseNode.IsNull()) return false;

    YAML::Node fileNode = baseNode[name];
    if (fileNode.IsNull()) return false;

    if (!fileNode["parent"].IsDefined() || !fileNode["child"].IsDefined()){
        std::cerr << "ERROR: PARENT/CHILD FOR: " << name << " NOT DEFINED \n";
        return false;
    }
    m_name = fileNode["name"].as<std::string>();
    m_parent_name = fileNode["parent"].as<std::string>();
    m_child_name = fileNode["child"].as<std::string>();

    if (!fileNode["parent_pivot"].IsDefined() ||
            !fileNode["parent_axis"].IsDefined() ||
            !fileNode["child_pivot"].IsDefined() ||
            !fileNode["child_axis"].IsDefined()){
        std::cerr << "ERROR: JOINT CONFIGURATION FOR: " << name << " NOT DEFINED \n";
        return false;
    }

    assign_vec("parent_pivot", &m_pvtA,  &fileNode);
    assign_vec("parent_axis", &m_axisA, &fileNode);
    assign_vec("child_pivot", &m_pvtB,  &fileNode);
    assign_vec("child_axis", &m_axisB,  &fileNode);

    Link * linkA, * linkB;
    if (mB->m_linkMap.find((m_parent_name + name_remapping).c_str()) != mB->m_linkMap.end()
            || mB->m_linkMap.find((m_child_name + name_remapping).c_str()) != mB->m_linkMap.end()){
        linkA =  mB->m_linkMap[(m_parent_name + name_remapping).c_str()];
        linkB = mB->m_linkMap[(m_child_name + name_remapping).c_str()];
        bodyA = linkA->m_bulletRigidBody;
        bodyB = linkB->m_bulletRigidBody;
        linkA->add_child_link(linkB, this);
    }
    else{
        std::cerr <<"ERROR:COULDN'T FIND RIGID BODIES FOR: " << m_name+name_remapping << std::endl;
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
    mB->m_chaiWorld->m_bulletWorld->addConstraint(m_hinge, true);
    return true;
}

///
/// \brief Joint::command_position
/// \param cmd
///
void Joint::command_position(double &cmd){
   m_hinge->setMotorTarget(cmd, 0.001);
}

///
/// \brief Joint::command_torque
/// \param cmd
///
void Joint::command_torque(double &cmd){
    btTransform trA = bodyA->getWorldTransform();
    btVector3 hingeAxisInWorld = trA.getBasis()*m_axisA;
    bodyA->applyTorque(-hingeAxisInWorld * cmd);
    bodyB->applyTorque(hingeAxisInWorld * cmd);
}

///
/// \brief cBulletMultiBody::cBulletMultiBody
/// \param bulletWorld
///
cBulletMultiBody::cBulletMultiBody(cBulletWorld *a_chaiWorld){
    m_chaiWorld = a_chaiWorld;
}

///
/// \brief cBulletMultiBody::compute_n_digits
/// \param n
///
/// Help from: https://stackoverflow.com/questions/1489830/efficient-way-to-determine-number-of-digits-in-an-integer
/// and https://stackoverflow.com/questions/11151548/get-the-number-of-digits-in-an-int/11151594
void cBulletMultiBody::remap_name(std::string &name, std::string remap_idx_str){
    if (remap_idx_str.length() == 0){
        return;
    }
    int cur_idx = std::stoi(remap_idx_str);
    if (cur_idx == 1){
        name += remap_idx_str;
        return;
    }
    else{
        int n_digits = 1;
        while(cur_idx/=10){
            n_digits++;
        }
        name.erase(name.end() - n_digits, name.end());
        name += remap_idx_str;
    }
}

///
/// \brief cBulletMultiBody::get_link_name_remapping
/// \param a_link_name
/// \return
///
std::string cBulletMultiBody::get_link_name_remapping(std::string a_link_name){
    int occurances = 0;
    std::string remap_string = "" ;
    std::stringstream ss;
    if (m_linkMap.find(a_link_name) == m_linkMap.end()){
        return remap_string;
    }
    do{
        ss.str(std::string());
        occurances++;
        ss << occurances;
        remap_string = ss.str();
    }
    while(m_linkMap.find(a_link_name + remap_string) != m_linkMap.end() && occurances < 100);
    return remap_string;
}

///
/// \brief cBulletMultiBody::get_link_name_remapping
/// \param a_link_name
/// \return
///
std::string cBulletMultiBody::get_joint_name_remapping(std::string a_joint_name){
    int occurances = 0;
    std::string remap_string = "" ;
    std::stringstream ss;
    if (m_jointMap.find(a_joint_name) == m_jointMap.end()){
        return remap_string;
    }

    do{
        ss.str(std::string());
        occurances++;
        ss << occurances;
        remap_string = ss.str();
    }
    while(m_jointMap.find(a_joint_name + remap_string) != m_jointMap.end() && occurances < 100);
    return remap_string;
}

///
/// \brief cBulletMultiBody::load_multibody
/// \param file
/// \return
///
Link* cBulletMultiBody::load_multibody(std::string a_multibody_config){
    if (a_multibody_config.empty()){
        a_multibody_config = get_puzzle_config();
    }
    YAML::Node multiBodyNode = YAML::LoadFile(a_multibody_config);
    if (!multiBodyNode){
        std::cerr << "FAILED TO LOAD YAML CONFIG FILE \n";
        return NULL;
    }

    Link *tmpLink;
    if (multiBodyNode["high_res_path"].IsDefined() && multiBodyNode["low_res_path"].IsDefined()){
        high_res_path = multiBodyNode["high_res_path"].as<std::string>();
        low_res_path = multiBodyNode["low_res_path"].as<std::string>();
    }
    else{
        high_res_path = "../resources/models/puzzle/high_res/";
        low_res_path = "../resources/models/puzzle/low_res/";
    }
    size_t totalLinks = multiBodyNode["links"].size();
    std::vector<std::string> temp_link_names;
    for (size_t i = 0; i < totalLinks; ++i) {
        tmpLink = new Link(m_chaiWorld);
        std::string link_name = multiBodyNode["links"][i].as<std::string>();
        std::string remap_str = get_link_name_remapping(link_name);
//        printf("Loading link: %s \n", (link_name + remap_str).c_str());
        if (tmpLink->load(a_multibody_config.c_str(), link_name, this, remap_str)){
            m_linkMap[(link_name + remap_str).c_str()] = tmpLink;
            temp_link_names.push_back((link_name + remap_str).c_str());
        }
    }
    Joint *tmpJoint;
    size_t totalJoints = multiBodyNode["joints"].size();
    for (size_t i = 0; i < totalJoints; ++i) {
        tmpJoint = new Joint();
        std::string jnt_name = multiBodyNode["joints"][i].as<std::string>();
        std::string remap_str = get_joint_name_remapping(jnt_name);
//        printf("Loading link: %s \n", (jnt_name + remap_str).c_str());
        if (tmpJoint->load(a_multibody_config.c_str(), jnt_name, this, remap_str)){
            m_jointMap[jnt_name+remap_str] = tmpJoint;
        }
    }
    Link* rootParentLink = NULL;
    size_t rootParents = 0;
    std::vector<std::string>::const_iterator nIt;
    cLinkMap::const_iterator mIt;
    for(nIt = temp_link_names.begin() ; nIt != temp_link_names.end() ; ++nIt){
        mIt = m_linkMap.find(*nIt);
        if((*mIt).second->m_parentLinks.size() == 0){
            rootParentLink = (*mIt).second;
            rootParents++;
        }
    }

    if (rootParents > 1)
        std::cerr << "WARNING! " << rootParents << " ROOT PARENTS FOUND, RETURNING LAST ONE\n";
    else if (rootParents == 0)
        std::cerr << "WARNING! " << rootParents << " ROOT PARENTS FOUND, RETURNING NULL\n";


    return rootParentLink;
}
}

