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
    virtual void buildDynamicModel();
    virtual void setLocalPos(const cVector3d& a_position);
    virtual void setLocalRot(const cMatrix3d& a_rotation);
    virtual void updatePositionFromDynamics();
public:
    cBulletMultiMesh* bulletMeshGripperL1;
    cBulletMultiMesh* bulletMeshGripperL2;
    btHingeConstraint* bulletHinge;

private:
    btVector3 axisA, axisB, pvtA, pvtB;
};

}

#endif
