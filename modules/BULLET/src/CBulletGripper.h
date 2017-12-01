#ifndef CBulletGripper_H
#define CBulletGripper_H

#include "chai3d.h"
#include "CBulletMultiMesh.h"
#include "CBullet.h"

namespace chai3d {
class cBulletWorld;
class cBulletGripper:public cBulletMultiMesh{
public:
    cBulletGripper(cBulletWorld *bulletWorld);
    ~cBulletGripper(){}

public:
    void set_gripper_angle(const double &angle);
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
