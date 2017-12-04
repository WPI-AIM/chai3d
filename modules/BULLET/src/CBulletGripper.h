#ifndef CBulletGripper_H
#define CBulletGripper_H

#include "chai3d.h"
#include "CBulletMultiMesh.h"
#include "CBullet.h"

namespace chai3d {

struct GripperSurfaceProperties{
public:
    GripperSurfaceProperties(){
        lin_damping = 0.0;
        ang_damping = 0.0;
        friction = 0.0;
        rolling_friction = 0.0;
    }
    void set_default(){
        lin_damping = 0.2;
        ang_damping = 1.0;
        friction = 0.5;
        rolling_friction = 0.5;
    }

    double lin_damping;
    double ang_damping;
    double friction;
    double rolling_friction;
};

class cBulletWorld;
class cBulletGripper:public cBulletMultiMesh{
public:
    cBulletGripper(cBulletWorld *bulletWorld);
    ~cBulletGripper(){}

public:
    void set_gripper_angle(const double &angle);
    void set_surface_props(GripperSurfaceProperties &props);
    void set_scale(double scale);
    void build();
public:
    cBulletMultiMesh* bulletMeshGripperL2;
    btHingeConstraint* bulletHinge;

private:
    btVector3 axisA, axisB, pvtA, pvtB;
    cMaterial mat;
};

}

#endif
