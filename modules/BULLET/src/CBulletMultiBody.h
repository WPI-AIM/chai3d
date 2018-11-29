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

#ifndef CBulletMultiBody_H
#define CBulletMultiBody_H

#include "CBulletMultiMesh.h"
#include "CBulletSoftMultiMesh.h"
#include "CBulletWorld.h"
#include "chai3d.h"
#include <yaml-cpp/yaml.h>

namespace chai3d {

class afBulletMultiBody;
class afRigidBody;
class afSoftBody;
class afJoint;

typedef std::shared_ptr<afBulletMultiBody> cBulletMultiBodyPtr;
typedef std::shared_ptr<afRigidBody> cMultiBodyBodyPtr;
typedef std::shared_ptr<afJoint> cMultiBodyJointPtr;
typedef std::map<std::string, afRigidBody*> cRigidBodyMap;
typedef std::map<std::string, afSoftBody*> cSoftBodyMap;
typedef std::map<std::string, afJoint*> cJointMap;

///
/// \brief The afConfigHandler class
///
class afConfigHandler{

public:

    afConfigHandler();
    std::string getConfigFile(std::string a_config_name);
    std::string getPuzzleConfig();
    std::string getColorConfig();
    std::string getWorldConfig();
    std::vector<double> getColorRGBA(std::string a_color_name);
    std::string getGripperConfig(std::string a_gripper_name);
    bool loadYAML(std::string file);

private:

    static std::string m_path;
    static std::string m_color_config;
    static std::string m_puzzle_config;
    static std::string m_world_config;
    YAML::Node configNode;
    static std::map<std::string, std::string> m_gripperConfigFiles;

protected:

    static YAML::Node m_colorsNode;

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
struct afSoftBodyCongifProperties: public btSoftBody::Config{

};

///
/// \brief The afBody class
///
class afRigidBody: public cBulletMultiMesh{

    friend class afBulletMultiBody;
    friend class afJoint;

public:

    afRigidBody(cBulletWorld* a_chaiWorld);
    virtual void updateCmdFromROS(double dt);
    virtual bool load(std::string file, std::string name, afBulletMultiBody* mB);
    virtual void addChildBody(afRigidBody* childBody, afJoint* jnt);

    std::vector<afJoint*> m_joints;
    std::vector<afRigidBody*> m_childrenBodies;
    std::vector<afRigidBody*> m_parentBodies;

    void setAngle(double &angle, double dt);
    void setAngle(std::vector<double> &angle, double dt);
    static void setSurfaceProperties(const afRigidBody* a_body, const afRigidBodySurfaceProperties *a_surfaceProps);

protected:

    double m_scale;
    double m_total_mass;
    std::string m_mesh_name;
    cMultiMesh m_lowResMesh;
    cVector3d pos;
    cMatrix3d rot;
    std::vector<afRigidBody*>::const_iterator m_bodyIt;
    double K_lin, D_lin;
    double K_ang, D_ang;
    bool _lin_gains_computed = false;
    bool _ang_gains_computed = false;
    void computeGains();
    void createAFObject(std::string a_object_name);

protected:

    void addParentBody(afRigidBody* a_body);
    void populateParentsTree(afRigidBody* a_body, afJoint* a_jnt);
    static afRigidBodySurfaceProperties m_surfaceProps;
    static cMaterial m_mat;

};

///
/// \brief The afJoint class
///
class afJoint{
    friend class afRigidBody;

public:

    afJoint();
    virtual ~afJoint();
    virtual bool load (std::string file, std::string name, afBulletMultiBody* mB, std::string name_remapping_idx = "");
    void commandTorque(double &cmd);
    void commandPosition(double &cmd);

private:

    std::string m_name;
    std::string m_parent_name, m_child_name;
    std::string m_joint_name;
    btVector3 m_axisA, m_axisB;
    btVector3 m_pvtA, m_pvtB;
    bool enable_motor;
    double jnt_lim_low, jnt_lim_high, max_motor_impluse;
    btRigidBody *bodyA, *bodyB;
    void assignVec(std::string name, btVector3* v, YAML::Node* node);
    void printVec(std::string name, btVector3* v);

protected:

    btHingeConstraint* m_hinge;

};

///
/// \brief The afSoftBody class
///
class afSoftBody: public cBulletSoftMultiMesh{

    friend class afBulletMultiBody;

public:

    afSoftBody(cBulletWorld* a_chaiWorld);
    virtual void updateCmdFromROS(double dt){}
    virtual bool load(std::string file, std::string name, afBulletMultiBody* mB);
    virtual void addChildBody(afSoftBody* childBody, afJoint* jnt){}

    std::vector<afJoint*> m_joints;
    std::vector<afSoftBody*> m_childrenBodies;
    std::vector<afSoftBody*> m_parentBodies;

    void setAngle(double &angle, double dt);
    void setAngle(std::vector<double> &angle, double dt);
    static void setConfigProperties(const afSoftBody* a_body, const afSoftBodyCongifProperties *a_configProps);

protected:

    double m_scale;
    double m_total_mass;
    std::string m_mesh_name;
    cMultiMesh m_lowResMesh;
    cVector3d pos;
    cMatrix3d rot;
    std::vector<afRigidBody*>::const_iterator m_bodyIt;
    double K_lin, D_lin;
    double K_ang, D_ang;
    bool _lin_gains_computed = false;
    bool _ang_gains_computed = false;
    void computeGains();
    void createAFObject(std::string a_object_name);

protected:

    void addParentBody(afRigidBody* a_body);
    void populateParentsTree(afRigidBody* a_body, afJoint* a_jnt);
    static afSoftBodyCongifProperties m_configProps;
    static cMaterial m_mat;
};

///
/// \brief The afWorld class
///
class afWorld: public afConfigHandler{

    friend class afBulletMultiBody;

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
/// \brief The afBulletMultiBody class
///
class afBulletMultiBody: public afWorld{

    friend class afRigidBody;
    friend class afSoftBody;
    friend class afJoint;

public:

    afBulletMultiBody();
    afBulletMultiBody(cBulletWorld* a_chaiWorld){m_chaiWorld = a_chaiWorld;}
    virtual ~afBulletMultiBody();
    virtual afRigidBody* loadMultiBody(std::string a_multibody_config = "");
    afRigidBody* getRidigBody(std::string a_name);
    afSoftBody* getSoftBody(std::string a_name);
    inline std::string getHighResPath(){return high_res_path;}
    inline std::string getLowResPath(){return low_res_path;}
    inline const cSoftBodyMap* getSoftBodyMap(){return &m_softBodyMap;}
    inline const cRigidBodyMap* getRigidBodyMap(){return &m_rigidBodyMap;}

protected:

    cRigidBodyMap m_rigidBodyMap;
    cSoftBodyMap m_softBodyMap;
    cJointMap m_jointMap;
    std::string high_res_path, low_res_path;

protected:

    cMaterial mat;
    std::string remapBodyName(std::string a_body_name);
    std::string remapJointName(std::string a_joint_name);
    void remapName(std::string &name, std::string remap_idx_str);
};

}

#endif
