
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

/// Declare Static Variables
cMaterial afRigidBody::m_mat;
afRigidBodySurfaceProperties afRigidBody::m_surfaceProps;

cMaterial afSoftBody::m_mat;

std::string afConfigHandler::s_path;
std::string afConfigHandler::s_color_config;
std::vector<std::string> afConfigHandler::s_multiBody_configs;
std::string afConfigHandler::s_world_config;
YAML::Node afConfigHandler::s_colorsNode;
std::map<std::string, std::string> afConfigHandler::s_gripperConfigFiles;

cBulletWorld* afWorld::m_chaiWorld;
double afWorld::m_encl_length;
double afWorld::m_encl_width;
double afWorld::m_encl_height;

/// End declare static variables

/// Utility Functions

///
/// \brief assignXYZ
/// \param v
///
void assignXYZ(YAML::Node* node, btVector3 *v){
    v->setX((*node)["x"].as<double>());
    v->setY((*node)["y"].as<double>());
    v->setZ((*node)["z"].as<double>());
}

///
/// \brief assignXYZ
/// \param node
/// \param v
///
void assignXYZ(YAML::Node* node, cVector3d *v){
    v->x((*node)["x"].as<double>());
    v->y((*node)["y"].as<double>());
    v->z((*node)["z"].as<double>());
}


///
/// \brief assignRPY
/// \param v
/// \param node
///
void assignRPY(YAML::Node* node, btVector3 *v){
    v->setX((*node)["r"].as<double>());
    v->setY((*node)["p"].as<double>());
    v->setZ((*node)["y"].as<double>());
}


///////////////////////////////////////////////

///
/// \brief afConfigHandler::afConfigHandler
///
afConfigHandler::afConfigHandler(){

}

///
/// \brief afConfigHandler::load_yaml
/// \param a_config_file
/// \return
///
bool afConfigHandler::loadYAML(std::string a_config_file){
    configNode = YAML::LoadFile(a_config_file);
    if(!configNode){
        std::cerr << "ERROR! FAILED TO LOAD CONFIG FILE \n";
    }

    //Declare all the YAML Params that we want to look for
    YAML::Node cfgPath = configNode["path"];
    YAML::Node cfgMultiBodyFiles = configNode["multibody configs"];
    YAML::Node cfgColorFile = configNode["color config"];
    YAML::Node cfgGripperFiles = configNode["gripper configs"];
    YAML::Node cfgWorldFiles = configNode["world config"];

    // Check if a world config file or a plain multibody config file
    if (cfgPath.IsDefined() && cfgMultiBodyFiles.IsDefined()){
        s_path = cfgPath.as<std::string>();
    }
    else{
        std::cerr << "PATH NOT DEFINED \n";
        return 0;
    }
    if (cfgMultiBodyFiles.IsDefined()){
        for (int i = 0 ; i < cfgMultiBodyFiles.size() ; i++){
            s_multiBody_configs.push_back(s_path + cfgMultiBodyFiles[i].as<std::string>());
        }
    }
    else{
        std::cerr << "PATH AND MULTIBODY CONFIG NOT DEFINED \n";
        return 0;
    }
    if(cfgColorFile.IsDefined()){
        s_color_config = s_path + cfgColorFile.as<std::string>();
        s_colorsNode = YAML::LoadFile(s_color_config.c_str());
        if (!s_colorsNode){
            std::cerr << "ERROR! COLOR CONFIG NOT FOUND \n";
        }
    }
    else{
        return 0;
    }

    if(cfgGripperFiles.IsDefined()){
        for(size_t i=0 ; i < cfgGripperFiles.size(); ++i){
            std::string gconf = cfgGripperFiles[i].as<std::string>();
            s_gripperConfigFiles[gconf] = s_path + configNode[gconf].as<std::string>();
        }
    }
    else{
        std::cerr << "ERROR! GRIPPER CONFIGS NOT DEFINED \n";
        return 0;
    }

    if(cfgWorldFiles.IsDefined()){
        std::string gconf = cfgWorldFiles.as<std::string>();
        s_world_config = s_path + gconf;
    }
    else{
        std::cerr << "ERROR! WORLD CONFIG NOT DEFINED \n";
        return 0;
    }

    return 1;
}

///
/// \brief afConfigHandler::get_world_config
/// \return
///
std::string afConfigHandler::getWorldConfig(){
    return s_world_config;
}

///
/// \brief afConfigHandler::get_puzzle_config
/// \return
///
std::string afConfigHandler::getMultiBodyConfig(int i){
    if (i <= numMultiBodyConfig()){
        return s_multiBody_configs[i];
    }
    else{
        printf("i = %d, Whereas only %d multi bodies specified", i, s_multiBody_configs.size());
        return "";
    }
}

///
/// \brief afConfigHandler::get_color_config
/// \return
///
std::string afConfigHandler::getColorConfig(){
    return s_color_config;
}

///
/// \brief afConfigHandler::get_gripper_config
/// \param a_gripper_name
/// \return
///
std::string afConfigHandler::getGripperConfig(std::string a_gripper_name){
    if(s_gripperConfigFiles.find(a_gripper_name) != s_gripperConfigFiles.end()){
        return s_gripperConfigFiles[a_gripper_name];
    }
    else{
        std::cerr << "WARNING! GRIPPER CONFIG FOR \"" << a_gripper_name
                  << "\" NOT FOUND, RETURNING DEFAULT \n";
        return s_gripperConfigFiles["Default"];
    }
}

///
/// \brief afConfigHandler::get_color_rgba
/// \param a_color_name
/// \return
///
std::vector<double> afConfigHandler::getColorRGBA(std::string a_color_name){
    std::vector<double> color_rgba = {0.5, 0.5, 0.5, 0.5};
    // Help from https://stackoverflow.com/questions/15425442/retrieve-random-key-element-for-stdmap-in-c
    if(strcmp(a_color_name.c_str(), "random") == 0 || strcmp(a_color_name.c_str(), "RANDOM") == 0){
        YAML::const_iterator it = s_colorsNode.begin();
        std::advance(it, rand() % s_colorsNode.size());
        color_rgba[0] = it->second["r"].as<int>() / 255.0;
        color_rgba[1] = it->second["g"].as<int>() / 255.0;
        color_rgba[2] = it->second["b"].as<int>() / 255.0;
        color_rgba[3] = it->second["a"].as<int>() / 255.0;
    }
    else if(s_colorsNode[a_color_name].IsDefined()){
        color_rgba[0] = s_colorsNode[a_color_name]["r"].as<int>() / 255.0;
        color_rgba[1] = s_colorsNode[a_color_name]["g"].as<int>() / 255.0;
        color_rgba[2] = s_colorsNode[a_color_name]["b"].as<int>() / 255.0;
        color_rgba[3] = s_colorsNode[a_color_name]["a"].as<int>() / 255.0;
    }
    else{
        std::cerr << "WARNING! COLOR NOT FOUND, RETURNING BALANCED COLOR\n";
    }
    return color_rgba;
}

///
/// \brief afBody::afBody
/// \param a_world
///
afRigidBody::afRigidBody(cBulletWorld* a_world): cBulletMultiMesh(a_world){
}

///
/// \brief afBody::populate_parents_tree
/// \param a_body
/// \param a_jnt
///
void afRigidBody::populateParentsTree(afRigidBodyPtr a_body, afJointPtr a_jnt){
    m_childrenBodies.push_back(a_body);
    m_childrenBodies.insert(m_childrenBodies.end(),
                           a_body->m_childrenBodies.begin(),
                           a_body->m_childrenBodies.end());
    m_joints.push_back(a_jnt);
    m_joints.insert(m_joints.end(),
                           a_body->m_joints.begin(),
                           a_body->m_joints.end());
}

///
/// \brief afBody::add_parent_body
/// \param a_parentBody
///
void afRigidBody::addParentBody(afRigidBody* a_parentBody){
    m_parentBodies.push_back(a_parentBody);
}

///
/// \brief afBody::add_child_body
/// \param a_childBody
/// \param a_jnt
///
void afRigidBody::addChildBody(afRigidBody* a_childBody, afJointPtr a_jnt){
    a_childBody->addParentBody(this);
    a_childBody->m_parentBodies.insert(a_childBody->m_parentBodies.end(),
                                      m_parentBodies.begin(), m_parentBodies.end());
    m_childrenBodies.push_back(a_childBody);
    m_childrenBodies.insert(m_childrenBodies.end(),
                           a_childBody->m_childrenBodies.begin(),
                           a_childBody->m_childrenBodies.end());
    m_joints.push_back(a_jnt);
    m_joints.insert(m_joints.end(),
                           a_childBody->m_joints.begin(),
                           a_childBody->m_joints.end());
    for (m_bodyIt = m_parentBodies.begin() ; m_bodyIt != m_parentBodies.end() ; ++m_bodyIt){
        (*m_bodyIt)->populateParentsTree(a_childBody, a_jnt);
    }

    for (m_bodyIt = a_childBody->m_childrenBodies.begin() ; m_bodyIt != a_childBody->m_childrenBodies.end() ; ++m_bodyIt){
        (*m_bodyIt)->addParentBody(this);
    }
}

///
/// \brief afBody::load
/// \param file
/// \param name
/// \param mB
/// \param name_remapping
/// \return
///
bool afRigidBody::load(std::string file, std::string name, afMultiBodyPtr mB) {
    YAML::Node baseNode = YAML::LoadFile(file);
    if (baseNode.IsNull()) return false;

    YAML::Node bodyNode = baseNode[name];
    if (bodyNode.IsNull()) return false;

    // Declare all the yaml parameters that we want to look for
    YAML::Node bodyName = bodyNode["name"];
    YAML::Node bodyMesh = bodyNode["mesh"];
    YAML::Node bodyCollisionMesh = bodyNode["collision mesh"];
    YAML::Node bodyScale = bodyNode["scale"];
    YAML::Node bodyInertialOffsetPos = bodyNode["inertial offset"]["position"];
    YAML::Node bodyInertialOffsetRot = bodyNode["inertial offset"]["orientation"];
    YAML::Node bodyMeshPathHR = bodyNode["high resolution path"];
    YAML::Node bodyMeshPathLR = bodyNode["low resolution path"];
    YAML::Node bodyNameSpace = bodyNode["namespace"];
    YAML::Node bodyMass = bodyNode["mass"];
    YAML::Node bodyLinGain = bodyNode["linear gain"];
    YAML::Node bodyAngGain = bodyNode["angular gain"];
    YAML::Node bodyInertia = bodyNode["inertia"];
    YAML::Node bodyPos = bodyNode["location"]["position"];
    YAML::Node bodyRot = bodyNode["location"]["orientation"];
    YAML::Node bodyColorRaw = bodyNode["color raw"];
    YAML::Node bodyColor = bodyNode["color"];
    YAML::Node bodyLinDamping = bodyNode["damping"]["linear"];
    YAML::Node bodyAngDamping = bodyNode["damping"]["angular"];
    YAML::Node bodyStaticFriction = bodyNode["friction"]["static"];
    YAML::Node bodyRollingFriction = bodyNode["friction"]["rolling"];


    if(bodyName.IsDefined()){
        m_name = bodyName.as<std::string>();
        m_name.erase(std::remove(m_name.begin(), m_name.end(), ' '), m_name.end());
    }

    if(bodyMesh.IsDefined())
        m_mesh_name = bodyMesh.as<std::string>();
    if (m_mesh_name.empty()){
        std::cerr << "WARNING: Body " << m_name << "'s mesh field is empty, ignoring\n";
        return 0;
    }

    if(bodyCollisionMesh.IsDefined())
        m_collision_mesh_name = bodyCollisionMesh.as<std::string>();
    else
        m_collision_mesh_name = m_mesh_name;

    if(bodyScale.IsDefined())
        m_scale = bodyScale.as<double>();

    std::string high_res_filepath;
    std::string low_res_filepath;

    // Each ridig body can have a seperate path for its low and high res meshes
    // Incase they are defined, we use those paths and if they are not, we use
    // the paths for the whole file
    if (bodyMeshPathHR.IsDefined()){
        high_res_filepath = bodyMeshPathHR.as<std::string>() + m_mesh_name;
    }
    else{
        high_res_filepath = mB->getHighResPath() + m_mesh_name;
    }

    if (bodyMeshPathLR.IsDefined()){
        low_res_filepath = bodyMeshPathLR.as<std::string>() + m_collision_mesh_name;
    }
    else{
        // If low res path is not defined, use the high res path to load the high-res mesh for collision
        low_res_filepath = mB->getLowResPath() + m_collision_mesh_name;
    }

    if (bodyNameSpace.IsDefined()){
        m_body_namespace = bodyNameSpace.as<std::string>();
    }
    else{
        m_body_namespace = mB->getNameSpace();
    }

    loadFromFile(RESOURCE_PATH(high_res_filepath));
    m_lowResMesh.loadFromFile(RESOURCE_PATH(low_res_filepath));
    scale(m_scale);
    m_lowResMesh.scale(m_scale);

    btTransform iOffTrans;
    btQuaternion iOffQuat;
    btVector3 iOffPos;
    iOffQuat.setEuler(0,0,0);
    iOffPos.setValue(0,0,0);
    if(bodyInertialOffsetPos.IsDefined()){
        if(bodyInertialOffsetRot.IsDefined()){
            double r = bodyInertialOffsetRot["r"].as<double>();
            double p = bodyInertialOffsetRot["p"].as<double>();
            double y = bodyInertialOffsetRot["y"].as<double>();
            iOffQuat.setEulerZYX(y, p, r);
        }
        assignXYZ( &bodyInertialOffsetPos, &iOffPos);
    }
    else{
        // Call the compute inertial offset before the build contact triangle method
        // as this method clears the low res mesh.
        iOffPos = computeInertialOffset(m_lowResMesh.m_meshes[0][0]);
    }

    iOffTrans.setOrigin(iOffPos);
    iOffTrans.setRotation(iOffQuat);
    setInertialOffsetTransform(iOffTrans);

    // Build contact triangles
    buildContactTriangles(0.001, &m_lowResMesh);


    if(bodyMass.IsDefined()){
        m_mass = bodyMass.as<double>();
        if(bodyLinGain.IsDefined()){
            K_lin = bodyLinGain["P"].as<double>();
            D_lin = bodyLinGain["D"].as<double>();
            _lin_gains_computed = true;
        }
        if(bodyAngGain.IsDefined()){
            K_ang = bodyAngGain["P"].as<double>();
            D_ang = bodyAngGain["D"].as<double>();
            _ang_gains_computed = true;
        }
    }

    if(bodyInertia.IsDefined()){
        setInertia(cVector3d(bodyInertia["ix"].as<double>(), bodyInertia["iy"].as<double>(), bodyInertia["iz"].as<double>()));
    }
    else{
        estimateInertia();
    }

    buildDynamicModel();

    if(bodyPos.IsDefined()){
        assignXYZ( &bodyPos, &m_initialPos);
        setLocalPos(m_initialPos);
    }

    if(bodyRot.IsDefined()){
        double r = bodyRot["r"].as<double>();
        double p = bodyRot["p"].as<double>();
        double y = bodyRot["y"].as<double>();
        m_initialRot.setExtrinsicEulerRotationRad(y,p,r,cEulerOrder::C_EULER_ORDER_ZYX);
        setLocalRot(m_initialRot);
    }

    if(bodyColorRaw.IsDefined()){
            m_mat.setColorf(bodyColorRaw["r"].as<float>(),
                            bodyColorRaw["g"].as<float>(),
                            bodyColorRaw["b"].as<float>(),
                            bodyColorRaw["a"].as<float>());
        }
    else if(bodyColor.IsDefined()){
        std::vector<double> rgba = mB->getColorRGBA(bodyColor.as<std::string>());
        m_mat.setColorf(rgba[0], rgba[1], rgba[2], rgba[3]);
    }

    if (bodyLinDamping.IsDefined())
        m_surfaceProps.m_linear_damping = bodyLinDamping.as<double>();
    if (bodyAngDamping.IsDefined())
        m_surfaceProps.m_angular_damping = bodyAngDamping.as<double>();
    if (bodyStaticFriction.IsDefined())
        m_surfaceProps.m_static_friction = bodyStaticFriction.as<double>();
    if (bodyRollingFriction.IsDefined())
        m_surfaceProps.m_rolling_friction = bodyRollingFriction.as<double>();

    setMaterial(m_mat);
    setConfigProperties(this, &m_surfaceProps);
    mB->m_chaiWorld->addChild(this);
    return true;
}

///
/// \brief afRigidBody::computeInertialOffset
/// \param mesh
/// \return
///
btVector3 afRigidBody::computeInertialOffset(cMesh* mesh){
    cVector3d intertialOffset(0,0,0);
    cVector3d vPos;

    int nvertices = mesh->getNumVertices();
    int i;
    double idx;
    for (i = 0, idx = 0 ; i < nvertices ; i++, idx++){
        vPos = mesh->m_vertices->getLocalPos(i);
        intertialOffset = ((( idx ) / ( idx + 1.0 )) * intertialOffset) + (( 1.0 / ( idx + 1.0 )) * vPos);
    }
    return btVector3(intertialOffset.x(), intertialOffset.y(), intertialOffset.z());
}

///
/// \brief afBody::compute_gains
///
void afRigidBody::computeControllerGains(){
    if (_lin_gains_computed && _ang_gains_computed){
        return;
    }

    double lumped_mass = m_mass;
    cVector3d lumped_intertia = m_inertia;
    for(m_bodyIt = m_childrenBodies.begin() ; m_bodyIt != m_childrenBodies.end() ; ++m_bodyIt){
        lumped_mass += (*m_bodyIt)->getMass();
        lumped_intertia += (*m_bodyIt)->getInertia();
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
/// \brief afBody::set_surface_properties
/// \param a_body
/// \param a_props
///
void afRigidBody::setConfigProperties(const afRigidBodyPtr a_body, const afRigidBodySurfacePropertiesPtr a_props){
    a_body->m_bulletRigidBody->setFriction(a_props->m_static_friction);
    a_body->m_bulletRigidBody->setDamping(a_props->m_linear_damping, a_props->m_angular_damping);
    a_body->m_bulletRigidBody->setRollingFriction(a_props->m_rolling_friction);
}

///
/// \brief afBody::updateCmdFromROS
/// \param dt
///
void afRigidBody::updateCmdFromROS(double dt){
    #ifdef C_ENABLE_CHAI_ENV_SUPPORT
    if (m_afObjPtr.get() != nullptr){
        m_afObjPtr->update_af_cmd();
        cVector3d force, torque;
        m_af_pos_ctrl_active = m_afObjPtr->m_afCmd.pos_ctrl;
        if (m_afObjPtr->m_afCmd.pos_ctrl){
            computeControllerGains();
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

            cmd_pos.set(m_afObjPtr->m_afCmd.px,
                        m_afObjPtr->m_afCmd.py,
                        m_afObjPtr->m_afCmd.pz);

            cmd_rot.x = m_afObjPtr->m_afCmd.qx;
            cmd_rot.y = m_afObjPtr->m_afCmd.qy;
            cmd_rot.z = m_afObjPtr->m_afCmd.qz;
            cmd_rot.w = m_afObjPtr->m_afCmd.qw;
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
            force.set(m_afObjPtr->m_afCmd.Fx,
                      m_afObjPtr->m_afCmd.Fy,
                      m_afObjPtr->m_afCmd.Fz);
            torque.set(m_afObjPtr->m_afCmd.Nx,
                       m_afObjPtr->m_afCmd.Ny,
                       m_afObjPtr->m_afCmd.Nz);
        }
        addExternalForce(force);
        addExternalTorque(torque);
        size_t jntCmdSize = m_afObjPtr->m_afCmd.size_J_cmd;
        if (jntCmdSize > 0 && m_parentBodies.size() == 0){
            size_t jntCnt = m_joints.size() < jntCmdSize ? m_joints.size() : jntCmdSize;
            for (size_t jnt = 0 ; jnt < jntCnt ; jnt++){
                if (m_afObjPtr->m_afCmd.pos_ctrl)
                    m_joints[jnt]->commandPosition(m_afObjPtr->m_afCmd.J_cmd[jnt]);
                else
                    m_joints[jnt]->commandTorque(m_afObjPtr->m_afCmd.J_cmd[jnt]);
            }

        }
    }
    #endif
}

///
/// \brief afBody::set_angle
/// \param angle
/// \param dt
///
void afRigidBody::setAngle(double &angle, double dt){
    if (m_parentBodies.size() == 0){
        for (size_t jnt = 0 ; jnt < m_joints.size() ; jnt++){
            m_joints[jnt]->m_hinge->setMotorTarget(angle, dt);
        }

    }
}

///
/// \brief afBody::set_angle
/// \param angles
/// \param dt
///
void afRigidBody::setAngle(std::vector<double> &angles, double dt){
    if (m_parentBodies.size() == 0){
        double jntCmdSize = m_joints.size() < angles.size() ? m_joints.size() : angles.size();
        for (size_t jnt = 0 ; jnt < jntCmdSize ; jnt++){
            m_joints[jnt]->m_hinge->setMotorTarget(angles[jnt], dt);
        }

    }
}

afRigidBody::~afRigidBody(){
    int numConstraints = m_bulletRigidBody->getNumConstraintRefs();
    for (int i = numConstraints - 1 ; i >=0 ; i--){
        btTypedConstraint * tConstraint = m_bulletRigidBody->getConstraintRef(i);
        m_bulletRigidBody->removeConstraintRef(tConstraint);
    }
}

///
/// \brief afSoftBody::afSoftBody
/// \param a_chaiWorld
///
afSoftBody::afSoftBody(cBulletWorld *a_chaiWorld): cBulletSoftMultiMesh(a_chaiWorld){

}

///
/// \brief afSoftBody::load
/// \param file
/// \param name
/// \param mB
/// \return
///
bool afSoftBody::load(std::string file, std::string name, afMultiBodyPtr mB) {
    YAML::Node baseNode = YAML::LoadFile(file);
    if (baseNode.IsNull()) return false;

    YAML::Node softBodyNode = baseNode[name];
    if (softBodyNode.IsNull()) return false;

    // Declare all the yaml parameters that we want to look for
    YAML::Node softBodyName = softBodyNode["name"];
    YAML::Node softBodyMesh = softBodyNode["mesh"];
    YAML::Node softBodyScale = softBodyNode["scale"];
    YAML::Node softBodyInertialOffsetPos = softBodyNode["inertial offset"]["position"];
    YAML::Node softBodyInertialOffsetRot = softBodyNode["inertial offset"]["orientation"];
    YAML::Node softBodyMeshPathHR = softBodyNode["high resolution path"];
    YAML::Node softBodyMeshPathLR = softBodyNode["low resolution path"];
    YAML::Node softBodyNameSpace = softBodyNode["namespace"];
    YAML::Node softBodyMass = softBodyNode["mass"];
    YAML::Node softBodyLinGain = softBodyNode["linear gain"];
    YAML::Node softBodyAngGain = softBodyNode["angular gain"];
    YAML::Node softBodyPos = softBodyNode["location"]["position"];
    YAML::Node softBodyRot = softBodyNode["location"]["orientation"];
    YAML::Node softBodyColorRaw = softBodyNode["color raw"];
    YAML::Node softBodyColor = softBodyNode["color"];
    YAML::Node softBodyConfigData = softBodyNode["config"];
    YAML::Node softBodyRandomizeConstraints = softBodyNode["randomize constraints"];

    YAML::Node cfg_kVCF = softBodyConfigData["kVCF"];
    YAML::Node cfg_kDP = softBodyConfigData["kDP"];
    YAML::Node cfg_kDG = softBodyConfigData["kDG"];
    YAML::Node cfg_kLF = softBodyConfigData["kLF"];
    YAML::Node cfg_kPR = softBodyConfigData["kPR"];
    YAML::Node cfg_kVC = softBodyConfigData["kVC"];
    YAML::Node cfg_kDF = softBodyConfigData["kDF"];
    YAML::Node cfg_kMT = softBodyConfigData["kMT"];
    YAML::Node cfg_kCHR = softBodyConfigData["kCHR"];
    YAML::Node cfg_kKHR = softBodyConfigData["kKHR"];
    YAML::Node cfg_kSHR = softBodyConfigData["kSHR"];
    YAML::Node cfg_kAHR = softBodyConfigData["kAHR"];
    YAML::Node cfg_kSRHR_CL = softBodyConfigData["kSRHR_CL"];
    YAML::Node cfg_kSKHR_CL = softBodyConfigData["kSKHR_CL"];
    YAML::Node cfg_kSSHR_CL = softBodyConfigData["kSSHR_CL"];
    YAML::Node cfg_kSR_SPLT_CL = softBodyConfigData["kSR_SPLT_CL"];
    YAML::Node cfg_kSK_SPLT_CL = softBodyConfigData["kSK_SPLT_CL"];
    YAML::Node cfg_kSS_SPLT_CL = softBodyConfigData["kSS_SPLT_CL"];
    YAML::Node cfg_maxvolume = softBodyConfigData["maxvolume"];
    YAML::Node cfg_timescale = softBodyConfigData["timescale"];
    YAML::Node cfg_viterations = softBodyConfigData["viterations"];
    YAML::Node cfg_piterations = softBodyConfigData["piterations"];
    YAML::Node cfg_diterations = softBodyConfigData["diterations"];
    YAML::Node cfg_citerations = softBodyConfigData["citerations"];
    YAML::Node cfg_collisions = softBodyConfigData["collisions"];

    if(softBodyName.IsDefined()){
        m_name = softBodyName.as<std::string>();
        m_name.erase(std::remove(m_name.begin(), m_name.end(), ' '), m_name.end());
    }

    if(softBodyMesh.IsDefined())
        m_mesh_name = softBodyMesh.as<std::string>();

    if(softBodyScale.IsDefined())
        m_scale = softBodyScale.as<double>();

    if(softBodyInertialOffsetPos.IsDefined()){
        btTransform trans;
        btQuaternion quat;
        btVector3 pos;
        quat.setEuler(0,0,0);
        pos.setValue(0,0,0);
        if(softBodyInertialOffsetRot.IsDefined()){
            double r = softBodyInertialOffsetRot["r"].as<double>();
            double p = softBodyInertialOffsetRot["p"].as<double>();
            double y = softBodyInertialOffsetRot["y"].as<double>();
            quat.setEulerZYX(y, p, r);
        }
        assignXYZ( &softBodyInertialOffsetPos, &pos);
        trans.setRotation(quat);
        trans.setOrigin(pos);
        setInertialOffsetTransform(trans);
    }

    std::string rel_path_high_res;
    std::string rel_path_low_res;
    if (softBodyMeshPathHR.IsDefined())
        rel_path_high_res = softBodyMeshPathHR.as<std::string>() + m_mesh_name;
    else
        rel_path_high_res = mB->getHighResPath() + m_mesh_name;

    if (softBodyMeshPathLR.IsDefined())
        rel_path_high_res = softBodyMeshPathLR.as<std::string>() + m_mesh_name;
    else
        rel_path_low_res = mB->getLowResPath() + m_mesh_name;

    loadFromFile(RESOURCE_PATH(rel_path_high_res));
    m_lowResMesh.loadFromFile(RESOURCE_PATH(rel_path_low_res));
    scale(m_scale);
    m_lowResMesh.scale(m_scale);
    buildContactTriangles(0.001, &m_lowResMesh);

    if(softBodyMass.IsDefined()){
        m_mass = softBodyMass.as<double>();
        if(softBodyLinGain.IsDefined()){
            K_lin = softBodyLinGain["P"].as<double>();
            D_lin = softBodyLinGain["D"].as<double>();
            _lin_gains_computed = true;
        }
        if(softBodyAngGain.IsDefined()){
            K_ang = softBodyAngGain["P"].as<double>();
            D_ang = softBodyAngGain["D"].as<double>();
            _ang_gains_computed = true;
        }
    }

    buildDynamicModel();

    if(softBodyPos.IsDefined()){
        assignXYZ( &softBodyPos, &pos);
        setLocalPos(pos);
    }

    if(softBodyRot.IsDefined()){
        double r = softBodyRot["r"].as<double>();
        double p = softBodyRot["p"].as<double>();
        double y = softBodyRot["y"].as<double>();
        rot.setExtrinsicEulerRotationRad(y,p,r,cEulerOrder::C_EULER_ORDER_ZXY);
        setLocalRot(rot);
    }

    if(softBodyColorRaw.IsDefined()){
            m_mat.setColorf(softBodyColorRaw["r"].as<float>(),
                            softBodyColorRaw["g"].as<float>(),
                            softBodyColorRaw["b"].as<float>(),
                            softBodyColorRaw["a"].as<float>());
        }
    else if(softBodyColor.IsDefined()){
        std::vector<double> rgba = mB->getColorRGBA(softBodyColor.as<std::string>());
        m_mat.setColorf(rgba[0], rgba[1], rgba[2], rgba[3]);

    }

    if (softBodyConfigData.IsNull()){
        printf("Warning, no soft body config properties defined");
    }
    else{
        if (cfg_kVCF.IsDefined()) m_bulletSoftBody->m_cfg.kVCF = cfg_kVCF.as<double>();
        if (cfg_kDP.IsDefined()) m_bulletSoftBody->m_cfg.kDP = cfg_kDP.as<double>();
        if (cfg_kDG.IsDefined()) m_bulletSoftBody->m_cfg.kDG = cfg_kDG.as<double>();
        if (cfg_kLF.IsDefined()) m_bulletSoftBody->m_cfg.kLF = cfg_kLF.as<double>();
        if (cfg_kPR.IsDefined()) m_bulletSoftBody->m_cfg.kPR = cfg_kPR.as<double>();
        if (cfg_kVC.IsDefined()) m_bulletSoftBody->m_cfg.kVC = cfg_kVC.as<double>();
        if (cfg_kDF.IsDefined()) m_bulletSoftBody->m_cfg.kDF = cfg_kDF.as<double>();
        if (cfg_kMT.IsDefined()) m_bulletSoftBody->m_cfg.kMT = cfg_kMT.as<double>();
        if (cfg_kCHR.IsDefined()) m_bulletSoftBody->m_cfg.kCHR = cfg_kCHR.as<double>();
        if (cfg_kKHR.IsDefined()) m_bulletSoftBody->m_cfg.kKHR = cfg_kKHR.as<double>();
        if (cfg_kSHR.IsDefined()) m_bulletSoftBody->m_cfg.kSHR = cfg_kSHR.as<double>();
        if (cfg_kAHR.IsDefined()) m_bulletSoftBody->m_cfg.kAHR = cfg_kAHR.as<double>();
        if (cfg_kSRHR_CL.IsDefined()) m_bulletSoftBody->m_cfg.kSRHR_CL = cfg_kSRHR_CL.as<double>();
        if (cfg_kSKHR_CL.IsDefined()) m_bulletSoftBody->m_cfg.kSKHR_CL = cfg_kSKHR_CL.as<double>();
        if (cfg_kSSHR_CL.IsDefined()) m_bulletSoftBody->m_cfg.kSSHR_CL = cfg_kSSHR_CL.as<double>();
        if (cfg_kSR_SPLT_CL.IsDefined()) m_bulletSoftBody->m_cfg.kSR_SPLT_CL = cfg_kSR_SPLT_CL.as<double>();
        if (cfg_kSK_SPLT_CL.IsDefined()) m_bulletSoftBody->m_cfg.kSK_SPLT_CL = cfg_kSK_SPLT_CL.as<double>();
        if (cfg_kSS_SPLT_CL.IsDefined()) m_bulletSoftBody->m_cfg.kSS_SPLT_CL = cfg_kSS_SPLT_CL.as<double>();
        if (cfg_maxvolume.IsDefined()) m_bulletSoftBody->m_cfg.maxvolume = cfg_maxvolume.as<double>();
        if (cfg_timescale.IsDefined()) m_bulletSoftBody->m_cfg.maxvolume = cfg_timescale.as<double>();
        if (cfg_viterations.IsDefined()) m_bulletSoftBody->m_cfg.viterations = cfg_viterations.as<double>();
        if (cfg_piterations.IsDefined()) m_bulletSoftBody->m_cfg.piterations = cfg_piterations.as<double>();
        if (cfg_diterations.IsDefined()) m_bulletSoftBody->m_cfg.diterations = cfg_diterations.as<double>();
        if (cfg_citerations.IsDefined()) m_bulletSoftBody->m_cfg.citerations = cfg_citerations.as<double>();
        if (cfg_collisions.IsDefined()) m_bulletSoftBody->m_cfg.collisions = cfg_collisions.as<double>();
    }

    if (softBodyRandomizeConstraints.IsDefined())
        if (softBodyRandomizeConstraints.as<bool>() == true)
            m_bulletSoftBody->randomizeConstraints();


    setMaterial(m_mat);
//    setConfigProperties(this, &m_bulletSoftBody->m_cfg);
    mB->m_chaiWorld->addChild(this);
    return true;
}

void afSoftBody::setConfigProperties(const afSoftBodyPtr a_body, const afSoftBodyConfigPropertiesPtr a_configProps){

}

///
/// \brief afJoint::afJoint
///
afJoint::afJoint(){

}

///
/// \brief afJoint::print_vec
/// \param name
/// \param v
///
void afJoint::printVec(std::string name, btVector3* v){
    printf("\t -%s: \n "
           "\t\t px = %f \n "
           "\t\t py = %f \n "
           "\t\t pz = %f \n",
           name.c_str(), v->x(), v->y(), v->z());
}

///
/// \brief afJoint::load
/// \param file
/// \param name
/// \param mB
/// \param name_remapping
/// \return
///
bool afJoint::load(std::string file, std::string name, afMultiBodyPtr mB, std::string name_remapping){
    YAML::Node baseNode = YAML::LoadFile(file);
    if (baseNode.IsNull()) return false;

    YAML::Node baseJointNode = baseNode[name];
    if (baseJointNode.IsNull()) return false;

    // Declare all the yaml parameters that we want to look for
    YAML::Node jointParentName = baseJointNode["parent"];
    YAML::Node jointChildName = baseJointNode["child"];
    YAML::Node jointName = baseJointNode["name"];
    YAML::Node jointParentPivot = baseJointNode["parent pivot"];
    YAML::Node jointChildPivot = baseJointNode["child pivot"];
    YAML::Node jointParentAxis = baseJointNode["parent axis"];
    YAML::Node jointChildAxis = baseJointNode["child axis"];
    YAML::Node jointOrigin = baseJointNode["origin"];
    YAML::Node jointAxis = baseJointNode["axis"];
    YAML::Node jointEnableMotor = baseJointNode["enable motor"];
    YAML::Node jointMaxMotorImpulse = baseJointNode["max motor impulse"];
    YAML::Node jointLimits = baseJointNode["joint limits"];
    YAML::Node jointOffset = baseJointNode["offset"];
    YAML::Node jointDamping = baseJointNode["joint damping"];
    YAML::Node jointType = baseJointNode["type"];

    if (!jointParentName.IsDefined() || !jointChildName.IsDefined()){
        std::cerr << "ERROR: PARENT/CHILD FOR: " << name << " NOT DEFINED \n";
        return false;
    }
    m_name = jointName.as<std::string>();
    m_name.erase(std::remove(m_name.begin(), m_name.end(), ' '), m_name.end());
    m_parent_name = jointParentName.as<std::string>();
    m_child_name = jointChildName.as<std::string>();
    // Joint Transform in Parent
    btTransform T_j_p;
    // Joint Axis
    btVector3 joint_axis(0,0,1);
    m_enable_motor = false;
    m_max_motor_impulse = 0.05;
    m_joint_offset = 0.0;
    m_lower_limit = -100;
    m_higher_limit = 100;
    //Default joint type is revolute if not type is specified
    m_jointType = JointType::revolute;

    btRigidBody * bodyA, * bodyB;
    afRigidBodyPtr afBodyA, afBodyB;

    if (mB->m_afRigidBodyMap.find((m_parent_name + name_remapping).c_str()) != mB->m_afRigidBodyMap.end()
            || mB->m_afRigidBodyMap.find((m_child_name + name_remapping).c_str()) != mB->m_afRigidBodyMap.end()){
        afBodyA =  mB->m_afRigidBodyMap[(m_parent_name + name_remapping).c_str()];
        afBodyB = mB->m_afRigidBodyMap[(m_child_name + name_remapping).c_str()];
        bodyA = afBodyA->m_bulletRigidBody;
        bodyB = afBodyB->m_bulletRigidBody;
    }
    else{
        std::cerr <<"ERROR:COULDN'T FIND RIGID BODIES FOR: " << m_name+name_remapping << std::endl;
        return -1;
    }

    if (jointParentPivot.IsDefined() & jointParentAxis.IsDefined() & jointChildPivot.IsDefined() & jointChildAxis.IsDefined()){
        assignXYZ( &jointParentPivot, &m_pvtA);
        assignXYZ( &jointParentAxis, &m_axisA);
        assignXYZ( &jointChildPivot, &m_pvtB);
        assignXYZ( &jointChildAxis, &m_axisB);

        // Scale the pivot before transforming as the default scale methods don't move this pivot
        m_pvtA *= afBodyA->m_scale;
        m_pvtA = afBodyA->getInertialOffsetTransform().inverse() * m_pvtA;
        m_pvtB = afBodyB->getInertialOffsetTransform().inverse() * m_pvtB;
        m_axisA = afBodyA->getInertialOffsetTransform().getBasis().inverse() * m_axisA;
        m_axisB = afBodyB->getInertialOffsetTransform().getBasis().inverse() * m_axisB;
    }
    else if(jointOrigin.IsDefined()){
        btQuaternion quat;
        btVector3 pos;
        YAML::Node jointXYZ = jointOrigin['position'];
        YAML::Node jointRPY = jointOrigin['orientation'];
        if (jointXYZ.IsDefined()){
            assignXYZ(&jointXYZ, &pos);
            T_j_p.setOrigin(pos);
        }
        if (jointRPY.IsDefined()){
            quat.setEulerZYX(jointRPY['y'].as<double>(),
                    jointRPY['p'].as<double>(),
                    jointRPY['r'].as<double>());
            T_j_p.setRotation(quat);
        }

        if (jointAxis.IsDefined()){
            assignXYZ(&jointAxis, &joint_axis);
        }
    }
    else{
        std::cerr << "ERROR: JOINT CONFIGURATION FOR: " << name << " NOT DEFINED \n";
        return false;
    }

    // For Testing Joints
//    if (strcmp(m_name.c_str(), "test") == 0){
//        btTransform tA, tB;
//        btQuaternion quat;

//        quat.setEulerZYX(m_axisA.getZ(), m_axisA.getY(), m_axisA.getX());

//        tA.setOrigin(m_pvtA);
//        tA.setRotation(quat);

//        quat.setEulerZYX(m_axisB.getZ(), m_axisB.getY(), m_axisB.getX());

//        tB.setOrigin(m_pvtB);
//        tB.setRotation(quat);

//        m_hinge = new btHingeConstraint(*bodyA, *bodyB, tA, tB, true);
//        if(jointLimits.IsDefined()){
//            m_lower_limit = jointLimits["low"].as<double>() + m_joint_offset;
//            m_higher_limit = jointLimits["high"].as<double>() + m_joint_offset;
//            m_hinge->setLimit(m_lower_limit, m_higher_limit);
//        }
//        mB->m_chaiWorld->m_bulletWorld->addConstraint(m_hinge, true);
//        afBodyA->addChildBody(afBodyB, this);
//        return true;
//    }

    if(jointOffset.IsDefined()){
        m_joint_offset = jointOffset.as<double>();
    }

    if(jointLimits.IsDefined()){
        m_lower_limit = jointLimits["low"].as<double>();
        m_higher_limit = jointLimits["high"].as<double>();
    }

    if (jointType.IsDefined()){
        if ((strcmp(jointType.as<std::string>().c_str(), "hinge") == 0)
                || (strcmp(jointType.as<std::string>().c_str(), "revolute") == 0)
                || (strcmp(jointType.as<std::string>().c_str(), "continuous") == 0)){
            m_jointType = JointType::revolute;
        }
        else if ((strcmp(jointType.as<std::string>().c_str(), "slider") == 0)
                || (strcmp(jointType.as<std::string>().c_str(), "prismatic") == 0)){
            m_jointType = JointType::prismatic;
        }
        else if ((strcmp(jointType.as<std::string>().c_str(), "fixed") == 0)){
            m_jointType = JointType::fixed;
        }

    }
    if (m_jointType == JointType::revolute){
        m_hinge = new btHingeConstraint(*bodyA, *bodyB, m_pvtA, m_pvtB, m_axisA, m_axisB, true);
        if (jointEnableMotor.IsDefined()){
            m_enable_motor = jointEnableMotor.as<int>();
            // Don't enable motor yet, only enable when set position is called
            if(jointMaxMotorImpulse.IsDefined()){
                m_max_motor_impulse = jointMaxMotorImpulse.as<double>();
                m_hinge->setMaxMotorImpulse(m_max_motor_impulse);
            }
        }

        if(jointLimits.IsDefined()){
            m_hinge->setLimit(m_lower_limit + m_joint_offset, m_higher_limit + m_joint_offset);
        }

        mB->m_chaiWorld->m_bulletWorld->addConstraint(m_hinge, true);
    }
    if (m_jointType == JointType::prismatic){
        btTransform frameA, frameB;
        btQuaternion quat;
        frameA.setIdentity();
        frameB.setIdentity();

        // Bullet takes the x axis as the default for prismatic joints
        btVector3 nx(1,0,0);

        double rot_anglA = nx.angle(m_axisA);
        if (rot_anglA < 0.01){
            quat.setEulerZYX(0,0,0);
        }
        else if (abs( 3.14 - rot_anglA) < 0.01 ){
            quat.setEulerZYX(0, 3.14, 0);
        }
        else{
            btVector3 rot_axisA = nx.cross(m_axisA);
            quat.setRotation(rot_axisA, rot_anglA);
        }
        frameA.setRotation(quat);
        frameA.setOrigin(m_pvtA);

        nx.setValue(1,0,0);
        double rot_anglB = nx.angle(m_axisB);
        if (rot_anglB < 0.01){
            quat.setEulerZYX(0,0,0);
        }
        else if (abs( 3.14 - rot_anglB) < 0.01 ){
            quat.setEulerZYX(0, 3.14, 0);
        }
        else{
            btVector3 rot_axisB = nx.cross(m_axisB);
            quat.setRotation(rot_axisB, rot_anglB);
        }
        btQuaternion offset_quat;
        offset_quat.setRotation(m_axisB, m_joint_offset);
        frameB.setRotation(offset_quat * quat);
        frameB.setOrigin(m_pvtB);

        m_slider = new btSliderConstraint(*bodyA, *bodyB, frameA, frameB, true);

        if (jointEnableMotor.IsDefined()){
            m_enable_motor = jointEnableMotor.as<int>();
            // Don't enable motor yet, only enable when set position is called
            if(jointMaxMotorImpulse.IsDefined()){
                m_max_motor_impulse = jointMaxMotorImpulse.as<double>();
                m_slider->setMaxLinMotorForce(m_max_motor_impulse);
            }
        }

        if(jointLimits.IsDefined()){
            m_slider->setLowerLinLimit(m_lower_limit);
            m_slider->setUpperLinLimit(m_higher_limit);
        }

        mB->m_chaiWorld->m_bulletWorld->addConstraint(m_slider, true);
        afBodyA->addChildBody(afBodyB, this);
    }
    return true;
}

///
/// \brief afJoint::command_position
/// \param cmd
///
void afJoint::commandPosition(double &cmd){
    // The torque commands disable the motor, so double check and re-enable the motor
    // if it was set to be enabled in the first place
    if (m_enable_motor){
        if (!m_hinge->getEnableAngularMotor()){
            m_hinge->enableMotor(m_enable_motor);
            m_hinge->setMaxMotorImpulse(m_max_motor_impulse);
        }
        m_hinge->setMotorTarget(cmd + m_joint_offset, 0.001);
    }
    else{
        std::cerr << "WARNING, MOTOR NOT ENABLED FOR JOINT: " << m_name << std::endl;
    }
}

///
/// \brief afJoint::command_torque
/// \param cmd
///
void afJoint::commandTorque(double &cmd){
    // If the motor was enabled, disable it before setting joint torques
    if (m_hinge->getEnableAngularMotor())
        m_hinge->enableMotor(false);{
    }
    btTransform trA = m_hinge->getRigidBodyA().getWorldTransform();
    btVector3 hingeAxisInWorld = trA.getBasis()*m_axisA;
    m_hinge->getRigidBodyA().applyTorque(-hingeAxisInWorld * cmd);
    m_hinge->getRigidBodyB().applyTorque(hingeAxisInWorld * cmd);
}

///
/// \brief afJoint::~afJoint
///
afJoint::~afJoint(){
    delete m_hinge;
}

///
/// \brief afWorld::afWorld
/// \param a_chaiWorld
///
afWorld::afWorld(cBulletWorld* a_chaiWorld){
    m_chaiWorld = a_chaiWorld;
    m_encl_length = 4.0;
    m_encl_width = 4.0;
    m_encl_height = 3.0;
}

///
/// \brief afWorld::get_enclosure_length
/// \return
///
double afWorld::getEnclosureLength(){
    return m_encl_length;
}

///
/// \brief afWorld::get_enclosure_width
/// \return
///
double afWorld::getEnclosureWidth(){
   return m_encl_width;
}

///
/// \brief afWorld::get_enclosure_height
/// \return
///
double afWorld::getEnclosureHeight(){
    return m_encl_height;
}

///
/// \brief afWorld::get_enclosure_extents
/// \param length
/// \param width
/// \param height
///
void afWorld::getEnclosureExtents(double &length, double &width, double &height){
    length = m_encl_length;
    width = m_encl_width;
    height = m_encl_height;
}


///
/// \brief afWorld::load_world
/// \param a_world_config
/// \return
///
bool afWorld::loadWorld(std::string a_world_config){
    if (a_world_config.empty()){
        a_world_config = getWorldConfig();
    }
    YAML::Node worldNode = YAML::LoadFile(a_world_config);
    if (!worldNode){
        std::cerr << "FAILED TO LOAD YAML CONFIG FILE \n";
        return -1;
    }
    else{
        m_encl_length = worldNode["enclosure size"]["length"].as<double>();
        m_encl_width = worldNode["enclosure size"]["width"].as<double>();
        m_encl_height = worldNode["enclosure size"]["height"].as<double>();
    }

    return true;

}

///
/// \brief afMultiBody::afMultiBody
///
afMultiBody::afMultiBody(){

}


/// Help from: https://stackoverflow.com/questions/1489830/efficient-way-to-determine-number-of-digits-in-an-integer
/// and https://stackoverflow.com/questions/11151548/get-the-number-of-digits-in-an-int/11151594
///
///
/// \brief afMultiBody::remapName
/// \param name
/// \param remap_idx_str
///
void afMultiBody::remapName(std::string &name, std::string remap_idx_str){
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

template<typename T>
///
/// \brief afMultiBody::remapBodyName
/// \param a_body_name
/// \param tMap
/// \return
///
std::string afMultiBody::remapBodyName(std::string a_body_name,const T* tMap){
    int occurances = 0;
    std::string remap_string = "" ;
    std::stringstream ss;
    if (tMap->find(a_body_name) == tMap->end()){
        return remap_string;
    }
    do{
        ss.str(std::string());
        occurances++;
        ss << occurances;
        remap_string = ss.str();
    }
    while(tMap->find(a_body_name + remap_string) != tMap->end() && occurances < 100);
    return remap_string;
}

///
/// \brief afMultiBody::remapJointName
/// \param a_joint_name
/// \return
///
std::string afMultiBody::remapJointName(std::string a_joint_name){
    int occurances = 0;
    std::string remap_string = "" ;
    std::stringstream ss;
    if (m_afJointMap.find(a_joint_name) == m_afJointMap.end()){
        return remap_string;
    }

    do{
        ss.str(std::string());
        occurances++;
        ss << occurances;
        remap_string = ss.str();
    }
    while(m_afJointMap.find(a_joint_name + remap_string) != m_afJointMap.end() && occurances < 100);
    return remap_string;
}

///
/// \brief afMultiBody::loadMultiBody
/// \return
///
bool afMultiBody::loadMultiBody(){
    return loadMultiBody(0);
}

///
/// \brief afMultiBody::loadMultiBody
/// \param i
/// \return
///
bool afMultiBody::loadMultiBody(int i){
    std::string multibody_config = getMultiBodyConfig(i);
    return loadMultiBody(multibody_config);
}

///
/// \brief afMultiBody::loadAllMultiBodies
///
void afMultiBody::loadAllMultiBodies(){
    for (int i = 0 ; i < numMultiBodyConfig(); i++){
        loadMultiBody(i);
    }
}

///
/// \brief afMultiBody::loadMultiBody
/// \param a_multibody_config
/// \return
///
bool afMultiBody::loadMultiBody(std::string a_multibody_config){
    if (a_multibody_config.empty()){
        a_multibody_config = getMultiBodyConfig();
    }
    YAML::Node multiBodyNode = YAML::LoadFile(a_multibody_config);
    if (!multiBodyNode){
        std::cerr << "FAILED TO LOAD YAML CONFIG FILE \n";
        return NULL;
    }

    // Declare all the yaml parameters that we want to look for
    YAML::Node multiBodyMeshPathHR = multiBodyNode["high resolution path"];
    YAML::Node multiBodyMeshPathLR = multiBodyNode["low resolution path"];
    YAML::Node multiBodyNameSpace = multiBodyNode["namespace"];
    YAML::Node multiBodyRidigBodies = multiBodyNode["bodies"];
    YAML::Node multiBodySoftBodies = multiBodyNode["soft bodies"];
    YAML::Node multiBodyJoints = multiBodyNode["joints"];

    /// Loading Rigid Bodies
    afRigidBodyPtr tmpRigidBody;
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

    size_t totalRigidBodies = multiBodyRidigBodies.size();
    for (size_t i = 0; i < totalRigidBodies; ++i) {
        tmpRigidBody = new afRigidBody(m_chaiWorld);
        std::string body_name = multiBodyRidigBodies[i].as<std::string>();
        std::string remap_str = remapBodyName(body_name, &m_afRigidBodyMap);
//        printf("Loading body: %s \n", (body_name + remap_str).c_str());
        if (tmpRigidBody->load(a_multibody_config.c_str(), body_name, this)){
            m_afRigidBodyMap[(body_name + remap_str).c_str()] = tmpRigidBody;
            tmpRigidBody->createAFObject(tmpRigidBody->m_name + remap_str, tmpRigidBody->m_body_namespace);
        }
    }


    /// Loading Soft Bodies
    afSoftBodyPtr tmpSoftBody;
    size_t totalSoftBodies = multiBodySoftBodies.size();
    for (size_t i = 0; i < totalSoftBodies; ++i) {
        tmpSoftBody = new afSoftBody(m_chaiWorld);
        std::string body_name = multiBodySoftBodies[i].as<std::string>();
        std::string remap_str = remapBodyName(body_name, &m_afSoftBodyMap);
//        printf("Loading body: %s \n", (body_name + remap_str).c_str());
        if (tmpSoftBody->load(a_multibody_config.c_str(), body_name, this)){
            m_afSoftBodyMap[(body_name + remap_str).c_str()] = tmpSoftBody;
//            tmpSoftBody->createAFObject(tmpSoftBody->m_name + remap_str);
        }
    }

    /// Loading Joints
    afJointPtr tmpJoint;
    size_t totalJoints = multiBodyJoints.size();
    for (size_t i = 0; i < totalJoints; ++i) {
        tmpJoint = new afJoint();
        std::string jnt_name = multiBodyJoints[i].as<std::string>();
        std::string remap_str = remapJointName(jnt_name);
//        printf("Loading body: %s \n", (jnt_name + remap_str).c_str());
        if (tmpJoint->load(a_multibody_config.c_str(), jnt_name, this, remap_str)){
            m_afJointMap[jnt_name+remap_str] = tmpJoint;
        }
    }

    removeOverlappingCollisionChecking();
    return true;
}

///
/// \brief afMultiBody::removeOverlappingCollisionChecking
///
void afMultiBody::removeOverlappingCollisionChecking(){
    // This function checks all the constraints of each ridig body
    // if there are more than 1, it means that multiple bodies share each other
    // In this case, iteratively go over all the shared bodies and ignore their
    // collision if their common body has the same pivot
    afRigidBodyMap::iterator rBodyIt = m_afRigidBodyMap.begin();
    std::vector<btRigidBody*> bodyFamily;
    std::pair<btVector3, btRigidBody*> pvtAandConnectedBody;
    std::vector< std::pair<btVector3, btRigidBody*> > pvtAandConnectedBodyVec;
    for ( ; rBodyIt != m_afRigidBodyMap.end() ; ++rBodyIt){
        afRigidBodyPtr afBody = rBodyIt->second;
        btRigidBody* rBody = afBody->m_bulletRigidBody;
        bodyFamily.clear();
        for(int cIdx = 0 ; cIdx < rBody->getNumConstraintRefs() ; cIdx++){
            if (rBody->getConstraintRef(cIdx)->getConstraintType() == btTypedConstraintType::HINGE_CONSTRAINT_TYPE){
                btHingeConstraint* joint = (btHingeConstraint*) rBody->getConstraintRef(cIdx);
                if (&joint->getRigidBodyA() == rBody){
                    pvtAandConnectedBody.first = joint->getAFrame().getOrigin();
                    pvtAandConnectedBody.second = &joint->getRigidBodyB();
                    pvtAandConnectedBodyVec.push_back(pvtAandConnectedBody);
                }
                else if (&joint->getRigidBodyB() == rBody){
                    pvtAandConnectedBody.first = joint->getBFrame().getOrigin();
                    pvtAandConnectedBody.second = &joint->getRigidBodyA();
                    pvtAandConnectedBodyVec.push_back(pvtAandConnectedBody);
                }
            }
        }
        if (pvtAandConnectedBodyVec.size() > 1){
            for (int pvtIdx1 = 0 ; pvtIdx1 < pvtAandConnectedBodyVec.size() - 1 ; pvtIdx1++ ){
                btVector3 pvtA1 = pvtAandConnectedBodyVec[pvtIdx1].first;
                btRigidBody* connectedBodyA1 = pvtAandConnectedBodyVec[pvtIdx1].second;
                for (int pvtIdx2 = pvtIdx1 + 1 ; pvtIdx2 < pvtAandConnectedBodyVec.size() ; pvtIdx2++ ){
                    btVector3 pvtA2 = pvtAandConnectedBodyVec[pvtIdx2].first;
                    btRigidBody* connectedBodyA2 = pvtAandConnectedBodyVec[pvtIdx2].second;
                    btVector3 diff = pvtA1 - pvtA2;
                    if (diff.length() < 0.1){
                        connectedBodyA1->setIgnoreCollisionCheck(connectedBodyA2, true);
                    }
                }

            }
        }
    }
}

/////
///// \brief afMultiBody::removeOverlappingCollisionChecking
/////
//void afMultiBody::removeOverlappingCollisionChecking(){
//    // This function checks all the constraints of each ridig body
//    // if there are more than 1, it means that multiple bodies share each other
//    // In this case, iteratively go over all the shared bodies and ignore their
//    // collision

//    std::vector<btRigidBody*> rigidBodiesA, rigidBodiesB;
//    std::vector<afJointPtr> afJoints;
//    int num_joints = m_afJointMap.size();
//    rigidBodiesA.resize(num_joints);
//    rigidBodiesB.resize(num_joints);
//    afJoints.resize(num_joints);
//    std::vector<bool> aBodiesChecked(num_joints, false);
//    std::vector<bool> bBodiesChecked(num_joints, false);

//    afJointMap::iterator jIt = m_afJointMap.begin();
//    for (int jIdx = 0; jIt != m_afJointMap.end() ; ++ jIt, jIdx++){
//        afJointPtr afJoint = jIt->second;
//        rigidBodiesA[jIdx] = &afJoint->m_hinge->getRigidBodyA();
//        rigidBodiesB[jIdx] = &afJoint->m_hinge->getRigidBodyB();
//        afJoints[jIdx] = afJoint;
//    }
//    std::pair<btVector3, btRigidBody*> pvtConnectedBodyPair;
//    std::vector< std::pair<btVector3, btRigidBody*> > pvtConnectedBodyPairs;
//    std::vector < std::vector< std::pair<btVector3, btRigidBody*> > > bodyPvtConnectedBodyPairs;

//    for (int a1 = 0 ; a1 < rigidBodiesA.size() ; a1 ++){
//        if (aBodiesChecked[a1] == false){
//            btRigidBody* rBodyA = rigidBodiesA[a1];
//            for (int a2 = a1 ; a2 < rigidBodiesA.size() ; a2 ++){
//                btRigidBody* rBodyB = rigidBodiesA[a2];
//                if (rBodyA == rBodyB){
//                    pvtConnectedBodyPair.first = afJoints[a1]->m_pvtA;
//                    pvtConnectedBodyPair.second = rigidBodiesB[a2];
//                    pvtConnectedBodyPairs.push_back(pvtConnectedBodyPair);
//                    aBodiesChecked[a2] = true;
//                }
//            }
//            for (int b1 = 0 ; b1 < rigidBodiesB.size() ; b1 ++){
//                if (bBodiesChecked[b1] == false){
//                    btRigidBody* rBodyB = rigidBodiesB[b1];
//                    if (rBodyA == rBodyB){
//                        pvtConnectedBodyPair.first = afJoints[b1]->m_pvtB;
//                        pvtConnectedBodyPair.second = rigidBodiesA[b1];
//                        pvtConnectedBodyPairs.push_back(pvtConnectedBodyPair);
//                        bBodiesChecked[b1] = true;
//                    }
//                }
//            }
//            bodyPvtConnectedBodyPairs.push_back(pvtConnectedBodyPairs);
//        }
//    }

//}

///
/// \brief afMultiBody::getRidigBody
/// \param a_name
/// \return
///
afRigidBodyPtr afMultiBody::getRidigBody(std::string a_name){
    if (m_afRigidBodyMap.find(a_name) != m_afRigidBodyMap.end()){
        return m_afRigidBodyMap[a_name];
    }
    else{
        std::cerr << "CAN'T FIND ANY BODY NAMED: " << a_name << std::endl;
        return NULL;
    }
}

///
/// \brief afMultiBody::getRootRigidBody
/// \param a_bodyPtr
/// \return
///
afRigidBodyPtr afMultiBody::getRootRigidBody(afRigidBodyPtr a_bodyPtr){
    /// Find Root Body
    afRigidBodyPtr rootParentBody;
    std::vector<int> lineageSize;
    size_t rootParents = 0;
    if (a_bodyPtr){
        if (a_bodyPtr->m_parentBodies.size() == 0){
            rootParentBody = a_bodyPtr;
            rootParents++;
        }
        else{
            lineageSize.resize(a_bodyPtr->m_parentBodies.size());
            std::vector<afRigidBodyPtr>::const_iterator rIt = a_bodyPtr->m_parentBodies.begin();
            for (int parentNum=0; rIt != a_bodyPtr->m_parentBodies.end() ; parentNum++, ++rIt){
                if ((*rIt)->m_parentBodies.size() == 0){
                    rootParentBody = (*rIt);
                    rootParents++;
                }
                lineageSize[parentNum] = (*rIt)->m_parentBodies.size();
            }
        }
    }
    else{
        lineageSize.resize(m_afRigidBodyMap.size());
        afRigidBodyMap::const_iterator mIt = m_afRigidBodyMap.begin();
        for(int bodyNum=0; mIt != m_afRigidBodyMap.end() ; bodyNum++, ++mIt){
            if ((*mIt).second->m_parentBodies.size() == 0){
                rootParentBody = (*mIt).second;
                ++rootParents;
            }
            lineageSize[bodyNum] = (*mIt).second->m_parentBodies.size();
        }

    }
    // In case no root parent is found, it is understood that
    // the multibody chain is cyclical, perhaps return
    // the body with least number of parents
    if (rootParents == 0){
        auto minLineage = std::min_element(lineageSize.begin(), lineageSize.end());
        int idx = std::distance(lineageSize.begin(), minLineage);
        rootParentBody = a_bodyPtr->m_parentBodies[idx];
        rootParents++;
        std::cerr << "WARNING! CYCLICAL CHAIN OF BODIES FOUND WITH NO UNIQUE PARENT, RETURING THE BODY WITH LEAST PARENTS";
    }

    if (rootParents > 1)
        std::cerr << "WARNING! " << rootParents << " ROOT PARENTS FOUND, RETURNING THE LAST ONE\n";

    return rootParentBody;
}

///
/// \brief afMultiBody::~afMultiBody
///
afMultiBody::~afMultiBody(){
//    afJointMap::const_iterator jIt = m_afJointMap.begin();
//    for (; jIt != m_afJointMap.end() ; ++jIt){
//        delete jIt->second;
//    }
//    afRigidBodyMap::iterator rIt = m_afRigidBodyMap.begin();
//    for ( ; rIt != m_afRigidBodyMap.end() ; ++rIt){
//        if (rIt->second)
//            delete rIt->second;
//    }
//    afSoftBodyMap::const_iterator sIt = m_afSoftBodyMap.begin();
//    for ( ; sIt != m_afSoftBodyMap.end() ; ++sIt){
//        delete sIt->second;
//    }
}

}
