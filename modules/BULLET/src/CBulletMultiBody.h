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
    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar
    \Motivation: https://www.gamedev.net/articles/programming/engines-and-middleware/yaml-basics-and-parsing-with-yaml-cpp-r3508/
    \version   3.2.1 $Rev: 2161 $
*/
//==============================================================================

#ifndef CBulletMultiBody_H
#define CBulletMultiBody_H

#include "CBulletMultiMesh.h"
#include "CBulletSoftMultiMesh.h"
#include "CBulletWorld.h"
#include "chai3d.h"
#include <yaml-cpp/yaml.h>
#include <boost/filesystem/path.hpp>

namespace chai3d {

class afMultiBody;
class afRigidBody;
class afSoftBody;
class afJoint;
struct afRigidBodySurfaceProperties;
struct afSoftBodyConfigProperties;

typedef afMultiBody* afMultiBodyPtr;
typedef afRigidBody* afRigidBodyPtr;
typedef afSoftBody* afSoftBodyPtr;
typedef afJoint* afJointPtr;
typedef afRigidBodySurfaceProperties* afRigidBodySurfacePropertiesPtr;
typedef afSoftBodyConfigProperties* afSoftBodyConfigPropertiesPtr;
typedef std::map<std::string, afRigidBodyPtr> afRigidBodyMap;
typedef std::map<std::string, afSoftBodyPtr> afSoftBodyMap;
typedef std::map<std::string, afJointPtr> afJointMap;

///
/// \brief The afConfigHandler class
///
class afConfigHandler{

public:

    afConfigHandler();
    virtual ~afConfigHandler(){}
    std::string getConfigFile(std::string a_config_name);
    std::string getMultiBodyConfig(int i=0);
    std::string getColorConfig();
    std::string getWorldConfig();
    std::vector<double> getColorRGBA(std::string a_color_name);
    std::string getGripperConfig(std::string a_gripper_name);
    bool loadBaseConfig(std::string file);
    inline int numMultiBodyConfig(){return s_multiBody_configs.size();}

private:

    static boost::filesystem::path s_boostBaseDir;
    static std::string s_color_config;
    static std::vector<std::string> s_multiBody_configs;
    static std::string s_world_config;
    YAML::Node configNode;
    static std::map<std::string, std::string> s_gripperConfigFiles;

protected:

    static YAML::Node s_colorsNode;

};

///
/// \brief The afBodySurfaceProperties struct
///
struct afRigidBodySurfaceProperties{
public:
    afRigidBodySurfaceProperties(){
        m_linear_damping = 0.1;
        m_angular_damping = 0.01;
        m_static_friction = 0.5;
        m_dynamic_friction = 0.5;
        m_rolling_friction = 0.5;
    }
    double m_linear_damping;
    double m_angular_damping;
    double m_static_friction;
    double m_dynamic_friction;
    double m_rolling_friction;
};

///
/// \brief The afSoftBodySurfaceProperties struct
///
struct afSoftBodyConfigProperties: public btSoftBody::Config{

};

///
/// \brief The afBody class
///
class afRigidBody: public cBulletMultiMesh{

    friend class afMultiBody;
    friend class afJoint;

public:

    afRigidBody(cBulletWorld* a_chaiWorld);
    virtual ~afRigidBody();
    virtual void afObjectCommandExecute(double dt);
    virtual bool loadRidigBody(std::string rb_config_file, std::string node_name, afMultiBodyPtr mB);
    virtual bool loadRidigBody(YAML::Node* rb_node, std::string node_name, afMultiBodyPtr mB);
    virtual void addChildBody(afRigidBodyPtr childBody, afJointPtr jnt);

    std::vector<afJointPtr> m_joints;
    std::vector<afRigidBodyPtr> m_childrenBodies;
    std::vector<afRigidBodyPtr> m_parentBodies;

    virtual void setAngle(double &angle, double dt);
    virtual void setAngle(std::vector<double> &angle, double dt);
    static void setConfigProperties(const afRigidBodyPtr a_body, const afRigidBodySurfacePropertiesPtr a_surfaceProps);
    std::string m_body_namespace;
    btVector3 computeInertialOffset(cMesh* mesh);

    //! This method toggles the viewing of frames of this rigid bodyl.
    inline void toggleFrameVisibility(){m_showFrame = !m_showFrame;}

    //! Get Min/Max publishing frequency for afObjectState for this body
    inline int getMinPublishFrequency(){return _min_publish_frequency;}
    inline int getMaxPublishFrequency(){return _max_publish_frequency;}

protected:

    double m_scale;
    double m_total_mass;
    std::string m_mesh_name, m_collision_mesh_name;
    cMultiMesh m_lowResMesh;
    cVector3d m_initialPos;
    cMatrix3d m_initialRot;
    std::vector<afRigidBodyPtr>::const_iterator m_bodyIt;
    double K_lin, D_lin;
    double K_ang, D_ang;
    bool _lin_gains_computed = false;
    bool _ang_gains_computed = false;
    bool _publish_joint_positions = false;
    bool _publish_children_names = false;
    int _min_publish_frequency;
    int _max_publish_frequency;
    void computeControllerGains();

protected:

    void addParentBody(afRigidBodyPtr a_body);
    void populateParentsTree(afRigidBodyPtr a_body, afJointPtr a_jnt);
    // Update the children for this body in the afObject State Message
    virtual void afObjectStateSetChildrenNames();
    // Update the joint positions of children in afObject State Message
    virtual void afObjectSetJointPositions();
    static afRigidBodySurfaceProperties m_surfaceProps;
    static cMaterial m_mat;

private:
    std::vector<float> m_joint_positions;

};

///
/// \brief The afSoftBody class
///
class afSoftBody: public cBulletSoftMultiMesh{

    friend class afMultiBody;

public:

    afSoftBody(cBulletWorld* a_chaiWorld);
    virtual void afObjectCommandExecute(double dt){}
    virtual bool loadSoftBody(std::string sb_config_file, std::string node_name, afMultiBodyPtr mB);
    virtual bool loadSoftBody(YAML::Node* sb_node, std::string node_name, afMultiBodyPtr mB);
    virtual void addChildBody(afSoftBodyPtr childBody, afJointPtr jnt){}

    std::vector<afJointPtr> m_joints;
    std::vector<afSoftBodyPtr> m_childrenBodies;
    std::vector<afSoftBodyPtr> m_parentBodies;

    void setAngle(double &angle, double dt);
    void setAngle(std::vector<double> &angle, double dt);
    static void setConfigProperties(const afSoftBodyPtr a_body, const afSoftBodyConfigPropertiesPtr a_configProps);

protected:

    double m_scale;
    double m_total_mass;
    std::string m_mesh_name;
    cMultiMesh m_lowResMesh;
    cVector3d pos;
    cMatrix3d rot;
    std::vector<afSoftBodyPtr>::const_iterator m_bodyIt;
    double K_lin, D_lin;
    double K_ang, D_ang;
    bool _lin_gains_computed = false;
    bool _ang_gains_computed = false;
    void computeGains();

protected:

    void addParentBody(afSoftBodyPtr a_body);
    void populateParentsTree(afSoftBodyPtr a_body, afJointPtr a_jnt);
    static afSoftBodyConfigProperties m_configProps;
    static cMaterial m_mat;
};

enum JointType{
    revolute = 0,
    prismatic = 1,
    fixed = 2
};

///
/// \brief The afJoint class
///
class afJoint{
    friend class afRigidBody;
    friend class afGripperLink;
    friend class afMultiBody;

public:

    afJoint();
    virtual ~afJoint();
    virtual bool loadJoint(std::string jnt_config_file, std::string node_name, afMultiBodyPtr mB, std::string name_remapping_idx = "");
    virtual bool loadJoint(YAML::Node* jnt_node, std::string node_name, afMultiBodyPtr mB, std::string name_remapping_idx = "");
    void commandTorque(double &cmd);
    void commandPosition(double &cmd);
    double getPosition();

protected:

    std::string m_name;
    std::string m_parent_name, m_child_name;
    std::string m_joint_name;
    btVector3 m_axisA, m_axisB;
    btVector3 m_pvtA, m_pvtB;
    double m_joint_damping;
    double m_max_effort;
    bool m_enable_actuator;
    double m_max_motor_impulse;
    double m_lower_limit, m_higher_limit;
    double m_joint_offset;
    btRigidBody *m_rbodyA, *m_rbodyB;
    void printVec(std::string name, btVector3* v);
    btQuaternion getRotationBetweenVectors(btVector3 &v1, btVector3 &v2);

protected:

    btTypedConstraint *m_btConstraint;
    JointType m_jointType;

private:
    // Add these two pointers for faster access to constraint internals
    // rather than having to cast the m_btConstraint ptr in high speed
    // control loops
    btHingeConstraint* m_hinge;
    btSliderConstraint* m_slider;
};


///
/// \brief The afWorld class
///
class afWorld: public afConfigHandler{

    friend class afMultiBody;

public:
    afWorld(cBulletWorld *bulletWorld);
    virtual ~afWorld(){}
    virtual bool loadWorld(std::string a_world_config = "");
    double getEnclosureLength();
    double getEnclosureWidth();
    double getEnclosureHeight();
    void getEnclosureExtents(double &length, double &width, double &height);
    static cBulletWorld *m_chaiWorld;

protected:

    afWorld(){}

private:

    static double m_encl_length;
    static double m_encl_width;
    static double m_encl_height;

};


///
/// \brief The afMultiBody class
///
class afMultiBody: public afWorld{

    friend class afRigidBody;
    friend class afSoftBody;
    friend class afJoint;

public:

    afMultiBody();
    afMultiBody(cBulletWorld* a_chaiWorld){m_chaiWorld = a_chaiWorld;}
    virtual ~afMultiBody();
    virtual bool loadMultiBody(int i);
    virtual bool loadMultiBody(std::string a_multibody_config);
    void loadAllMultiBodies();
    afRigidBodyPtr getRidigBody(std::string a_name, bool suppress_warning=false);
    afRigidBodyPtr getRootRigidBody(afRigidBodyPtr a_bodyPtr = NULL);
    afSoftBodyPtr getSoftBody(std::string a_name);
    inline std::string getHighResMeshesPath(){return m_multibody_high_res_meshes_path;}
    inline std::string getLowResMeshesPath(){return m_multibody_low_res_meshes_path;}
    inline std::string getMultiBodyPath(){return m_multibody_path;}
    inline std::string getNameSpace(){return m_multibody_namespace;}
    inline const afSoftBodyMap* getSoftBodyMap(){return &m_afSoftBodyMap;}
    inline const afRigidBodyMap* getRigidBodyMap(){return &m_afRigidBodyMap;}
    // We can have multiple bodies connected to a single body.
    // There isn't a direct way in bullet to disable collision
    // between all these bodies connected in a tree
    void removeOverlappingCollisionChecking();
    //Remove collision checking for this entire multi-body, mostly for
    // debugging purposes
    void ignoreCollisionChecking();

protected:

    afRigidBodyMap m_afRigidBodyMap;
    afSoftBodyMap m_afSoftBodyMap;
    afJointMap m_afJointMap;
    std::string m_multibody_high_res_meshes_path, m_multibody_low_res_meshes_path;
    std::string m_multibody_namespace;
    std::string m_multibody_path;

protected:

    cMaterial mat;
    template <typename T>
    std::string remapBodyName(std::string a_body_name, const T* tMap);
    std::string remapJointName(std::string a_joint_name);
    void remapName(std::string &name, std::string remap_idx_str);
};

}

#endif
