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

class cBulletMultiBody;
class cBulletWorld;
class Link;
class Joint;

typedef std::shared_ptr<cBulletMultiBody> cBulletMultiBodyPtr;
typedef std::shared_ptr<Link> cMultiBodyLinkPtr;
typedef std::shared_ptr<Joint> cMultiBodyJointPtr;
typedef std::map<std::string, Link*> cLinkMap;
typedef std::map<std::string, Joint*> cJointMap;

///
/// \brief The ConfigHandler class
///
class ConfigHandler{

public:

    ConfigHandler();
    std::string get_config_file(std::string a_config_name);
    std::string get_puzzle_config();
    std::string get_color_config();
    std::vector<double> get_color_rgba(std::string a_color_name);
    std::string get_gripper_config(std::string a_gripper_name);
    bool load_yaml(std::string file);

private:
    std::string m_path;
    std::string m_color_config;
    std::string m_puzzle_config;
    static std::map<std::string, std::string> m_gripperConfigFiles;

protected:

    static YAML::Node m_colorsNode;

};

///
/// \brief The LinkSurfaceProperties struct
///
struct LinkSurfaceProperties{
public:
    LinkSurfaceProperties(){
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
/// \brief The Link class
///
class Link : public cBulletMultiMesh{
    friend cBulletMultiBody;


public:

    Link(cBulletWorld* a_chaiWorld);
    virtual void updateCmdFromROS(double dt);
    virtual bool load (std::string file, std::string name, cBulletMultiBody* mB, std::string name_remapping_idx= "");
    virtual void add_child_link(Link* childLink, Joint* jnt);

    std::vector<Joint*> m_joints;
    std::vector<Link*> m_childrenLinks;
    std::vector<Link*> m_parentLinks;

    void set_angle(double &angle, double dt);
    void set_angle(std::vector<double> &angle, double dt);
    static void set_surface_properties(const Link* a_link, const LinkSurfaceProperties *a_surfaceProps);

protected:

    double m_scale;
    double m_total_mass;
    std::string m_mesh_name;
    cMultiMesh m_lowResMesh;
    cVector3d pos;
    cMatrix3d rot;
    std::vector<Link*>::const_iterator m_linkIt;
    double K_lin, D_lin;
    double K_ang, D_ang;
    bool _lin_gains_computed = false;
    bool _ang_gains_computed = false;
    void compute_gains();
    void create_af_object(std::string a_object_name);

protected:

    void add_parent_link(Link* a_link);
    void populate_parents_tree(Link* a_link, Joint* a_jnt);
    static LinkSurfaceProperties m_surfaceProps;
    static cMaterial m_mat;

};

///
/// \brief The Joint class
///
class Joint{
    friend Link;

public:

    Joint();
    virtual ~Joint();
    virtual bool load (std::string file, std::string name, cBulletMultiBody* mB, std::string name_remapping_idx = "");
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
/// \brief The cBulletMultiBody class
///
class cBulletMultiBody: public ConfigHandler{
public:
    friend Link;
    friend Joint;
    cBulletMultiBody(cBulletWorld *bulletWorld);
    virtual ~cBulletMultiBody();
    virtual Link* load_multibody(std::string a_multibody_config = "");

protected:

    cBulletWorld *m_chaiWorld;
    cLinkMap m_linkMap;
    cJointMap m_jointMap;
    std::string high_res_path, low_res_path;

protected:

    cMaterial mat;
    std::string get_link_name_remapping(std::string a_link_name);
    std::string get_joint_name_remapping(std::string a_joint_name);
    void remap_name(std::string &name, std::string remap_idx_str);
};

}

#endif
