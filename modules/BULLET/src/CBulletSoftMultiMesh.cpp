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

    \author    <http://www.chai3d.org>
    \author    Adnan Munawar
    \version   3.2.0 $Rev: 2161 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "CBulletSoftMultiMesh.h"
//------------------------------------------------------------------------------
#include "CBulletWorld.h"
//------------------------------------------------------------------------------
#include "chai3d.h"
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------


//==============================================================================
/*!
    This method assigns a desired position to the object.

    \param  a_position  New desired position.
*/
//==============================================================================
void cBulletSoftMultiMesh::setLocalPos(const cVector3d& a_position)
{
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


//==============================================================================
/*!
    This method assigns a desired rotation to the object.

    \param  a_rotation  New desired orientation.
*/
//==============================================================================
void cBulletSoftMultiMesh::setLocalRot(const cMatrix3d& a_rotation)
{
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


//==============================================================================
/*!
    This method updates the position and orientation data from the Bullet 
    representation to the CHAI3D representation.
*/
//==============================================================================
void cBulletSoftMultiMesh::updatePositionFromDynamics()
{
    if (m_bulletSoftBody)
    {
        // create collision detector for each mesh
        std::vector<cMesh*>::iterator it;
        for (it = m_meshes->begin(); it < m_meshes->end(); it++)
        {
            cMesh* mesh = (*it);

            // read number of triangles of the object
            unsigned int nVerts = m_bulletSoftBody->m_nodes.size();
            // add all triangles to Bullet model
            m_counter++;
            if(m_counter % 2 ==0){
                for (unsigned int i=0; i<nVerts; i++)
                {
                    btVector3 v = m_bulletSoftBody->m_nodes[i].m_x;
                    for(unsigned int j=0 ; j < 3 ; j++){
                        mesh->m_vertices->setLocalPos(i,v.x(), v.y(), v.z());
                    }
                }

            }

        }
        markForUpdate(true);
    }

    // update Transform data for m_rosObj
    if(m_afObjPtr.get() != nullptr){
        m_afObjPtr->cur_position(m_localPos.x(), m_localPos.y(), m_localPos.z());
        cQuaternion q;
        q.fromRotMat(m_localRot);
        m_afObjPtr->cur_orientation(q.x, q.y, q.z, q.w);
    }
}

struct TVPair{
    std::list<unsigned int> triangleIdx;
    std::list<unsigned int> vertexIdx;
} g_tvPair;

std::map<unsigned int, TVPair> g_vertexTVMap;

void updateMins(double &x, double &y, double &z, cVector3d &v){
    x = cMin(x,v.x());
    y = cMin(y,v.y());
    z = cMin(z,v.z());
}

void updateMins(cVector3d &vMin, cVector3d &v){
    vMin.x(cMin(vMin.x(),v.x()));
    vMin.y(cMin(vMin.y(),v.y()));
    vMin.z(cMin(vMin.z(),v.z()));
}

void updateMaxs(double &x, double &y, double &z, cVector3d &v){
    x = cMax(x,v.x());
    y = cMax(y,v.y());
    z = cMax(z,v.z());
}

void updateMaxs(cVector3d &vMax, cVector3d &v){
    vMax.x(cMax(vMax.x(),v.x()));
    vMax.y(cMax(vMax.y(),v.y()));
    vMax.z(cMax(vMax.z(),v.z()));
}

//bool isPresentInGrid(unsigned int cntIdx, cVector3d &v, cVector3d &vMin, cVector3d &vBounds, bool* vtxCheckGrid, int* vtxIdxGrid){
//    unsigned int xIdx, yIdx, zIdx;
//    xIdx = (v.x() + vMin.x()) / vBounds.x();
//    yIdx = (v.y() + vMin.y()) / vBounds.y();
//    zIdx = (v.z() + vMin.z()) / vBounds.z();

//    if (vtxCheckGrid[xIdx + yIdx + zIdx] == false){

//    }
//}


//==============================================================================
/*!
    This method creates a Bullet collision model for this object.
*/
//==============================================================================
void cBulletSoftMultiMesh::buildContactTriangles(const double a_margin, cMultiMesh* lowResMesh)
{
    // create compound shape
    btCompoundShape* compound = new btCompoundShape();
    m_bulletCollisionShape = compound;

    std::vector<cMesh*> *v_meshes;
    if (lowResMesh ){
        v_meshes = lowResMesh->m_meshes;
    }
    else{
        v_meshes = m_meshes;
    }

    // create collision detector for each mesh
    std::vector<cMesh*>::iterator it;
    for (it = v_meshes->begin(); it < v_meshes->end(); it++)
    {
        cMesh* mesh = (*it);

        // read number of triangles of the object
        unsigned int numTriangles = mesh->m_triangles->getNumElements();
        unsigned int numVertices = mesh->m_vertices->getNumElements();

        printf("# Triangles %d, # Vertices %d \n", numTriangles, numVertices);



        bool vertexCheckGrid [numVertices][numVertices][numVertices];
        unsigned int vertexIdxGrid [numVertices][numVertices][numVertices];
        double sbTriangleArray[numTriangles * 3];
        std::vector<double> sbVertexArray;
        cVector3d vMin(9999,9999,9999);
        cVector3d vMax(-9999,-9999,-9999);
        cVector3d vBounds;

        for (unsigned int x = 0 ; x < numVertices ; x++){
            cVector3d v = mesh->m_vertices->getLocalPos(x);
            updateMins(vMin, v);
            updateMaxs(vMax, v);
            for (unsigned int y = 0 ; y < numVertices ; y++){
                for (unsigned int z = 0 ; z < numVertices ; z++){
                    vertexCheckGrid[x][y][z] = false;
                    vertexIdxGrid[x][y][z] = -1;
                }
            }
        }
        vBounds = vMax - vMin;
        printf("***************************************\n");
        printf("Vmin = [%f, %f, %f] \n", vMin.x(), vMin.y(), vMin.z());
        printf("Vmax = [%f, %f, %f] \n", vMax.x(), vMax.y(), vMax.z());
        printf("VBounds = [%f, %f, %f] \n", vBounds.x(), vBounds.y(), vBounds.z());
        printf("***************************************\n");
        int unique = 0;
        int duplicate = 0;
        for (unsigned int tNum = 0 ; tNum < numTriangles ; tNum ++){
            for (unsigned int vNum = 0 ; vNum < 3 ; vNum++){

                auto vIdx = mesh->m_triangles->getVertexIndex(tNum, vNum);
                auto vPos = mesh->m_vertices->getLocalPos(vIdx);
//                printf("Triangle # %d, Vertex # %d, Idx # %d \n", tNum, vNum, vIdx);

                unsigned int xKey = (double) (numVertices - 1) * ((vPos.x() - vMin.x()) / vBounds.x());
                unsigned int yKey = (double) (numVertices - 1) * ((vPos.y() - vMin.y()) / vBounds.y());
                unsigned int zKey = (double) (numVertices - 1) * ((vPos.z() - vMin.z()) / vBounds.z());

//                printf("vPos = [%f, %f, %f] \n", vPos.x(), vPos.y(), vPos.z());
                printf("Keys = [%d, %d, %d] ", xKey, yKey, zKey);

                if (vertexCheckGrid[xKey][yKey][zKey] == false){
                    vertexCheckGrid[xKey][yKey][zKey] = true;
                    vertexIdxGrid[xKey][yKey][zKey] = 3*tNum + vNum;
                    sbTriangleArray[tNum + vNum] = sbVertexArray.size();
                    sbVertexArray.push_back(vPos.x());
                    sbVertexArray.push_back(vPos.y());
                    sbVertexArray.push_back(vPos.z());
                    TVPair tvPair;
                    tvPair.triangleIdx.push_back(tNum);
                    tvPair.vertexIdx.push_back(vIdx);
                    g_vertexTVMap[tNum] = tvPair;
                    printf("New Vertex \n");
                    unique++;

                }
                else{
                    unsigned int matchedVtxIdx = vertexIdxGrid[xKey][yKey][zKey];
                    g_vertexTVMap[matchedVtxIdx].triangleIdx.push_back(tNum);
                    g_vertexTVMap[matchedVtxIdx].vertexIdx.push_back(vIdx);
                    sbTriangleArray[tNum + vNum] = matchedVtxIdx;
                    printf("Duplicate Vertex \n");
                    duplicate++;
                }
            }
        }

        for (int i = 0; i < numVertices ; i++){
            printf("Vertex %d, TV Map Size %d \n", i, g_vertexTVMap[i].triangleIdx.size());
        }
        printf("Unique Vertices Found = %d, Duplicate Vertices Found = %d\n", unique, duplicate);


//        for (int i = 0 ; i < numVertices ; i ++){
//            auto p = mesh->m_vertices->getLocalPos(i);
//            printf("Vertices %d = [%f, %f, %f]\n", i, p.x(), p.y(), p.z());
//        }

//        // bullet mesh
//        m_trianglesPtr = new int[numTriangles*3];
//        m_verticesPtr = new btScalar[numTriangles*9];

//        // add all triangles to Bullet model
//        for (unsigned int i=0; i<numTriangles; i++)
//        {
//            m_trianglesPtr[3*i + 0] = mesh->m_triangles->getVertexIndex0(i);
//            m_trianglesPtr[3*i + 1] = mesh->m_triangles->getVertexIndex1(i);
//            m_trianglesPtr[3*i + 2] = mesh->m_triangles->getVertexIndex2(i);

//            for(unsigned int j=0 ; j < 3 ; j++){
//                cVector3d vertex = mesh->m_vertices->getLocalPos(m_trianglesPtr[3*i + j]);
//                m_verticesPtr[ (9*i) + (3*j) + 0] = vertex.x();
//                m_verticesPtr[ (9*i) + (3*j) + 1] = vertex.y();
//                m_verticesPtr[ (9*i) + (3*j) + 2] = vertex.z();
//            }
//        }
//        // create mesh collision model
//        m_bulletSoftBody = btSoftBodyHelpers::CreateFromTriMesh(*m_dynamicWorld->m_bulletSoftBodyWorldInfo,
//                                                                          m_verticesPtr, m_trianglesPtr, numTriangles);

//        // add to compound object
//        btTransform localTrans;
//        btVector3 pos;
//        btQuaternion q;

//        // set new position
//        cVector3d posMesh = mesh->getLocalPos();
//        pos[0] = posMesh(0);
//        pos[1] = posMesh(1);
//        pos[2] = posMesh(2);

//        // set new orientation
//        cMatrix3d rotMesh = mesh->getLocalRot();
//        cQuaternion quaternion;
//        quaternion.fromRotMat(rotMesh);

//        q.setW(quaternion.w);
//        q.setX(quaternion.x);
//        q.setY(quaternion.y);
//        q.setZ(quaternion.z);

//        // set new transform
//        localTrans.setOrigin(pos);
//        localTrans.setRotation(q);

//        // Apply the inertial transform offset
//        localTrans *= m_inertialOffsetTransform.inverse();

//        // add collision shape to compound
//        m_bulletSoftBody->getCollisionShape()->setUserPointer(m_bulletSoftBody);
////        m_bulletSoftBody->setTotalMass(50,true);
//        btSoftRigidDynamicsWorld *softWorld = (btSoftRigidDynamicsWorld*) m_dynamicWorld->m_bulletWorld;
//        softWorld->addSoftBody(m_bulletSoftBody);
//        m_dynamicWorld->m_bulletSoftBodyWorldInfo->m_sparsesdf.Reset();
    }
//    if(lowResMesh){
//        lowResMesh->m_meshes->clear();
//    }
}


//==============================================================================
/*!
    This method creates a Bullet collision model for this object.
*/
//==============================================================================
void cBulletSoftMultiMesh::buildContactConvexTriangles(const double a_margin)
{
    // create compound shape
    btCompoundShape* compound = new btCompoundShape();
    m_bulletCollisionShape = compound;

    // create collision detector for each mesh
    std::vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        cMesh* mesh = (*it);

        // bullet mesh
        btTriangleMesh* bulletMesh = new btTriangleMesh();

        // read number of triangles of the object
        unsigned int numTriangles = mesh->m_triangles->getNumElements();

        // add all triangles to Bullet model
        for (unsigned int i=0; i<numTriangles; i++)
        {
            unsigned int vertexIndex0 = mesh->m_triangles->getVertexIndex0(i);
            unsigned int vertexIndex1 = mesh->m_triangles->getVertexIndex1(i);
            unsigned int vertexIndex2 = mesh->m_triangles->getVertexIndex2(i);

            cVector3d vertex0 = mesh->m_vertices->getLocalPos(vertexIndex0);
            cVector3d vertex1 = mesh->m_vertices->getLocalPos(vertexIndex1);
            cVector3d vertex2 = mesh->m_vertices->getLocalPos(vertexIndex2);

            bulletMesh->addTriangle(btVector3(vertex0(0), vertex0(1), vertex0(2)), 
                btVector3(vertex1(0), vertex1(1), vertex1(2)), 
                btVector3(vertex2(0), vertex2(1), vertex2(2)));
        }

        // create mesh collision model
        btConvexTriangleMeshShape* collisionShape = new btConvexTriangleMeshShape(bulletMesh);

        // assign settings
        collisionShape->setMargin(a_margin);

        // add to compound object
        btTransform localTrans;
        btVector3 pos;
        btQuaternion q;

        // set new position
        cVector3d posMesh = mesh->getLocalPos();
        pos[0] = posMesh(0);
        pos[1] = posMesh(1);
        pos[2] = posMesh(2);

        // set new orientation
        cMatrix3d rotMesh = mesh->getLocalRot();
        cQuaternion quaternion;
        quaternion.fromRotMat(rotMesh);

        q.setW(quaternion.w);
        q.setX(quaternion.x);
        q.setY(quaternion.y);
        q.setZ(quaternion.z);

        // set new transform
        localTrans.setOrigin(pos);
        localTrans.setRotation(q);

        // Apply the inertial transform offset
        localTrans *= m_inertialOffsetTransform.inverse();

        // add collision shape to compound
        compound->addChildShape(localTrans, collisionShape);
    }
}


//==============================================================================
/*!
    This method creates a Bullet collision model for this object.
*/
//==============================================================================
void cBulletSoftMultiMesh::buildContactHull(const double a_margin)
{
    // create compound shape
    btCompoundShape* compound = new btCompoundShape();
    m_bulletCollisionShape = compound;

    // create collision detector for each mesh
    std::vector<cMesh*>::iterator it;
    for (it = m_meshes->begin(); it < m_meshes->end(); it++)
    {
        cMesh* mesh = (*it);

        // create convex hull
        btConvexHullShape* collisionShape = new btConvexHullShape((double*)(&mesh->m_vertices->m_localPos[0]), mesh->m_vertices->getNumElements(), sizeof(cVector3d));

        // set margin
        collisionShape->setMargin(a_margin);

        // add to compound object
        btTransform localTrans;
        btVector3 pos;
        btQuaternion q;

        // set new position
        cVector3d posMesh = mesh->getLocalPos();
        pos[0] = posMesh(0);
        pos[1] = posMesh(1);
        pos[2] = posMesh(2);

        // set new orientation
        cMatrix3d rotMesh = mesh->getLocalRot();
        cQuaternion quaternion;
        quaternion.fromRotMat(rotMesh);

        q.setW(quaternion.w);
        q.setX(quaternion.x);
        q.setY(quaternion.y);
        q.setZ(quaternion.z);

        // set new transform
        localTrans.setOrigin(pos);
        localTrans.setRotation(q);

        // Apply the inertial transform offset
        localTrans *= m_inertialOffsetTransform.inverse();

        // add collision shape to compound
        compound->addChildShape(localTrans, collisionShape);
    }
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
