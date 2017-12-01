#include "CBulletGripper.h"
#include <string.h>

// root resource path
std::string resourceRoot;
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+std::string(p)).c_str())

namespace chai3d {

cBulletGripper::cBulletGripper(cBulletWorld *bulletWorld):cBulletMultiMesh(bulletWorld){
    bulletMeshGripperL1 = new cBulletMultiMesh(bulletWorld);
    bulletMeshGripperL2 = new cBulletMultiMesh(bulletWorld);

    bulletMeshGripperL1->loadFromFile(RESOURCE_PATH("../resources/models/gripper/gripper_L1.3ds"));
    bulletMeshGripperL2->loadFromFile(RESOURCE_PATH("../resources/models/gripper/gripper_L1.3ds"));
    bulletMeshGripperL1->scale(0.1);
    bulletMeshGripperL2->scale(0.05);
    bulletMeshGripperL1->setLocalPos(0.0,0.0,0.0);
    cMatrix3d mat;
    mat.setAxisAngleRotationDeg(0,0,1,180);
    bulletMeshGripperL1->setLocalRot(mat);
}

void cBulletGripper::buildDynamicModel(){
    bulletMeshGripperL1->setMass(0.1);
    bulletMeshGripperL2->setMass(0.05);

    bulletMeshGripperL1->buildContactTriangles(0.001);
    bulletMeshGripperL2->buildContactTriangles(0.001);

    bulletMeshGripperL1->setShowFrame(true);
    bulletMeshGripperL2->setShowFrame(true);

    bulletMeshGripperL1->estimateInertia();
    bulletMeshGripperL2->estimateInertia();

    bulletMeshGripperL1->buildDynamicModel();
    bulletMeshGripperL2->buildDynamicModel();

    m_dynamicWorld->addChild(bulletMeshGripperL1);
    m_dynamicWorld->addChild(bulletMeshGripperL2);

    axisA.setValue(0.0,0.0,1.0);
    axisB = -axisA;
    pvtA.setValue(0.0,0.0,0.0);
    pvtB = pvtA;
    bulletHinge = new btHingeConstraint(*bulletMeshGripperL1->m_bulletRigidBody,
                                        *bulletMeshGripperL2->m_bulletRigidBody,
                                        pvtA, pvtB, axisA, axisB);

    m_dynamicWorld->m_bulletWorld->addConstraint(bulletHinge);
    bulletHinge->enableMotor(true);
    bulletHinge->setLimit(1.57, 3.14);

}

void cBulletGripper::set_gripper_angle(const double &angle){
    bulletHinge->setMotorTarget(angle,0.001);
}

void cBulletGripper::setLocalPos(const cVector3d &a_position){
    m_localPos = a_position;

    // get transformation matrix of object
    btTransform trans;
    btVector3 pos;
    btQuaternion q;

    // set new position
    pos[0] = m_localPos(0);
    pos[1] = m_localPos(1);
    pos[2] = m_localPos(2);

    // set new orientation
    cQuaternion quaternion;
    quaternion.fromRotMat(m_localRot);

    q.setW(quaternion.w);
    q.setX(quaternion.x);
    q.setY(quaternion.y);
    q.setZ(quaternion.z);

    // set new transform
    trans.setOrigin(pos);
    trans.setRotation(q);
    if (m_bulletMotionState)
        m_bulletMotionState->setWorldTransform(trans);
    if (m_bulletRigidBody)
        m_bulletRigidBody->setCenterOfMassTransform(trans);
}

void cBulletGripper::setLocalRot(const cMatrix3d &a_rotation){
    m_localRot = a_rotation;

    // get transformation matrix of object
    btTransform trans;
    btVector3 pos;
    btQuaternion q;

    // set new position
    pos[0] = m_localPos(0);
    pos[1] = m_localPos(1);
    pos[2] = m_localPos(2);

    // set new orientation
    cQuaternion quaternion;
    quaternion.fromRotMat(m_localRot);

    q.setW(quaternion.w);
    q.setX(quaternion.x);
    q.setY(quaternion.y);
    q.setZ(quaternion.z);

    // set new transform
    trans.setOrigin(pos);
    trans.setRotation(q);
    if (m_bulletMotionState)
        m_bulletMotionState->setWorldTransform(trans);
    if (m_bulletRigidBody)
        m_bulletRigidBody->setCenterOfMassTransform(trans);
}

void cBulletGripper::updatePositionFromDynamics(){
    if (m_bulletRigidBody)
    {
        // get transformation matrix of object
        btTransform trans;
        m_bulletRigidBody->getMotionState()->getWorldTransform(trans);

        btVector3 pos = trans.getOrigin();
        btQuaternion q = trans.getRotation();

       // set new position
        m_localPos.set(pos[0],pos[1],pos[2]);

        // set new orientation
        cQuaternion quaternion(q.getW(), q.getX(), q.getY(), q.getZ());
        quaternion.toRotMat(m_localRot);

        // orthogonalize frame
        m_localRot.orthogonalize();
    }
}
}
