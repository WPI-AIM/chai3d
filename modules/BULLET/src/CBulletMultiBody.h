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
#include "CBulletWorld.h"
#include "chai3d.h"
#include <yaml-cpp/yaml.h>

namespace chai3d {

class afBulletMultiBody;
class afRigidBody;
class afJoint;

typedef std::shared_ptr<afBulletMultiBody> cBulletMultiBodyPtr;
typedef std::shared_ptr<afRigidBody> cMultiBodyBodyPtr;
typedef std::shared_ptr<afJoint> cMultiBodyJointPtr;
typedef std::map<std::string, afRigidBody*> cBodyMap;
typedef std::map<std::string, afJoint*> cJointMap;

///
/// \brief The afConfigHandler class
///
class afConfigHandler{

public:

    afConfigHandler();
    std::string get_config_file(std::string a_config_name);
    std::string get_puzzle_config();
    std::string get_color_config();
    std::string get_world_config();
    std::vector<double> get_color_rgba(std::string a_color_name);
    std::string get_gripper_config(std::string a_gripper_name);
    bool load_yaml(std::string file);

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
struct afBodySurfaceProperties{
public:
    afBodySurfaceProperties(){
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

    void set_angle(double &angle, double dt);
    void set_angle(std::vector<double> &angle, double dt);
    static void set_surface_properties(const afRigidBody* a_body, const afBodySurfaceProperties *a_surfaceProps);

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
    void compute_gains();
    void createAFObject(std::string a_object_name);

protected:

    void add_parent_body(afRigidBody* a_body);
    void populate_parents_tree(afRigidBody* a_body, afJoint* a_jnt);
    static afBodySurfaceProperties m_surfaceProps;
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
    void command_torque(double &cmd);
    void command_position(double &cmd);

private:

    std::string m_name;
    std::string m_parent_name, m_child_name;
    std::string m_joint_name;
    btVector3 m_axisA, m_axisB;
    btVector3 m_pvtA, m_pvtB;
    bool enable_motor;
    double jnt_lim_low, jnt_lim_high, max_motor_impluse;
    btRigidBody *bodyA, *bodyB;
    void assign_vec(std::string name, btVector3* v, YAML::Node* node);
    void print_vec(std::string name, btVector3* v);

protected:

    btHingeConstraint* m_hinge;

};

///
/// \brief The afWorld class
///
class afWorld: public afConfigHandler{

    friend class afBulletMultiBody;

public:
    afWorld(cBulletWorld *bulletWorld);
    virtual ~afWorld(){}
    virtual bool load_world(std::string a_world_config = "");
    double get_enclosure_length();
    double get_enclosure_width();
    double get_enclosure_height();
    void get_enclosure_extents(double &length, double &width, double &height);
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
    friend class afJoint;

public:

    afBulletMultiBody();
    afBulletMultiBody(cBulletWorld* a_chaiWorld){m_chaiWorld = a_chaiWorld;}
    virtual ~afBulletMultiBody();
    virtual afRigidBody* load_multibody(std::string a_multibody_config = "");
    afRigidBody* get_body(std::string a_name);

protected:

    cBodyMap m_bodyMap;
    cJointMap m_jointMap;
    std::string high_res_path, low_res_path;

protected:

    cMaterial mat;
    std::string get_body_name_remapping(std::string a_body_name);
    std::string get_joint_name_remapping(std::string a_joint_name);
    void remap_name(std::string &name, std::string remap_idx_str);
};

}

#endif
