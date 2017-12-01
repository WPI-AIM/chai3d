#include "CBulletGripper.h"
#include <string.h>

// root resource path
std::string resourceRoot;
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+std::string(p)).c_str())

namespace chai3d {

cBulletGripper::cBulletGripper(cBulletWorld *bulletWorld):cBulletMultiMesh(bulletWorld){


    loadFromFile(RESOURCE_PATH("../resources/models/gripper/gripper_L1.3ds"));
    scale(0.1);
    setLocalPos(0.0,-0.2,0.0);

    bulletMeshGripperL2 = new cBulletMultiMesh(bulletWorld);
    bulletMeshGripperL2->loadFromFile(RESOURCE_PATH("../resources/models/gripper/gripper_L1.3ds"));
    bulletMeshGripperL2->scale(0.1);
    bulletMeshGripperL2->setLocalPos(0.0,0.2,0.0);
    cMatrix3d rotMat;
    rotMat.setAxisAngleRotationDeg(1,0,0,180);
    bulletMeshGripperL2->setLocalRot(rotMat);
}

void cBulletGripper::build(){
    setMass(0.03);
    buildContactTriangles(0.001);
    setShowFrame(true);
    estimateInertia();
    buildDynamicModel();
    m_dynamicWorld->addChild(this);


    bulletMeshGripperL2->setMass(0.03);
    bulletMeshGripperL2->buildContactTriangles(0.001);
    bulletMeshGripperL2->setShowFrame(true);
    bulletMeshGripperL2->estimateInertia();
    bulletMeshGripperL2->buildDynamicModel();
    m_dynamicWorld->addChild(bulletMeshGripperL2);

    axisA.setValue(0.0,0.0,1.0);
    axisB = -axisA;
    pvtA.setValue(0.2,0.1,0.0);
    pvtB.setValue(0.2,0.1,0.0);
    bulletHinge = new btHingeConstraint(*this->m_bulletRigidBody,
                                        *bulletMeshGripperL2->m_bulletRigidBody,
                                        pvtA, pvtB, axisA, axisB, true);

    m_dynamicWorld->m_bulletWorld->addConstraint(bulletHinge, true);
    bulletHinge->enableMotor(true);
    bulletHinge->setMaxMotorImpulse(0.3);
    bulletHinge->setLimit(2.2, 3.139);

    GripperSurfaceProperties props;
    props.set_default();
    set_surface_props(props);

    mat.setBlueLightSteel();
    setMaterial(mat);
    mat.setBlueMidnight();
    bulletMeshGripperL2->setMaterial(mat);
}

void cBulletGripper::set_gripper_angle(const double &angle){
    bulletHinge->setMotorTarget(angle, 0.001);
}

void cBulletGripper::set_surface_props(GripperSurfaceProperties &props){

    this->m_bulletRigidBody->setDamping(props.lin_damping, props.ang_damping);
    this->m_bulletRigidBody->setFriction(props.friction);
    this->m_bulletRigidBody->setRollingFriction(props.rolling_friction);

    bulletMeshGripperL2->m_bulletRigidBody->setDamping(props.lin_damping, props.ang_damping);
    bulletMeshGripperL2->m_bulletRigidBody->setFriction(props.friction);
    bulletMeshGripperL2->m_bulletRigidBody->setRollingFriction(props.rolling_friction);

}

}
