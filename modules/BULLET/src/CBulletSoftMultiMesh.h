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
    \version   3.2.1 $Rev: 2017 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CBulletSoftMultiMeshH
#define CBulletSoftMultiMeshH
//------------------------------------------------------------------------------
#include "CBulletGenericObject.h"
#include "chai3d.h"
#include "CGELMesh.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

///
/// \brief The TVPair struct: A tree
///
struct VertexTree{
    std::vector<int> triangleIdx;
    std::vector<int> vertexIdx;
};

//==============================================================================
/*!
    \file       CBulletSoftMultiMesh.h

    \brief
    <b> Bullet Module </b> \n 
    Bullet Soft MultiMesh Object.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cBulletSoftMultiMesh
    \ingroup    Bullet

    \brief
    This class implements a Bullet dynamic soft multi-mesh.

    \details
    cBulletSoftMultiMesh models a dynamic soft multi-mesh object.
*/
//==============================================================================
class cBulletSoftMultiMesh : public cGELMesh, public cBulletGenericObject
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cBulletSoftMultiMesh.
    cBulletSoftMultiMesh(cBulletWorld* a_world, std::string a_objName = "") : cBulletGenericObject(a_world, a_objName) {
    }

    //! Destructor of cBulletSoftMultiMesh.
    virtual ~cBulletSoftMultiMesh() {}


    //--------------------------------------------------------------------------
    // IMPORT:
    //--------------------------------------------------------------------------

public:

    //! import base class overloaded virtual and non-virtual methods
    using cGenericObject::setLocalPos;
    using cGenericObject::setLocalRot;


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - TRANSLATION AND ORIENTATION:
    //--------------------------------------------------------------------------

public:

    //! This method sets the local position of object.
    virtual void setLocalPos(const cVector3d& a_position);

    //! This method sets the orientation of this object.
    virtual void setLocalRot(const cMatrix3d& a_rotation);

    //! This method update the CHAI3D position representation from the Bullet dynamics engine.
    virtual void updatePositionFromDynamics();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - CONTACT MODEL:
    //--------------------------------------------------------------------------

public:
    //! This method loads a mesh from a file.
//    virtual loadFromFile(std::string a_filename);

    //! This method updates the skeletal model of GEL from Bullet's softBody.
    virtual void updateGELSkeletonFrombtSoftBody();

    virtual void render(cRenderOptions &a_options);

    //! This method creates a Bullet collision model for this object.
    virtual void buildContactConvexTriangles(const double a_margin = 0.01);

    //! This method creates a Bullet collision model for this object.
    virtual void buildContactTriangles(const double a_margin = 0.01, cMultiMesh *lowResMesh = NULL) ;

    //! This method creates a Bullet collision model for this object.
    virtual void buildContactHull(const double a_margin = 0.01);

    //! Create CGEL Links and Nodes based on bulletSoftBody
    virtual void createGELSkeleton();

    inline btSoftBody* getSoftBody(){return m_bulletSoftBody;}

    inline void setSoftBody(btSoftBody* a_softBody){m_bulletSoftBody = a_softBody;}


private:
    //! Ptr to scalar vertex arrays of the sofy body
    std::vector<btScalar> m_verticesPtr;
    //! Ptr to Triangles arrays referring to vertices by indices
    std::vector<int> m_trianglesPtr;
     //! Ptr to vector vertex arrays of the sofy body
    std::vector<btVector3> m_verticesVecPtr;
    //! Vertex Tree containing vtx idx's that are repeated for a given vtx
    std::vector<VertexTree> m_vertexTree;
    //! Function to detect, index and store repeat vertices
    void computeUniqueVerticesandTriangles(cMesh* mesh, std::vector<btScalar>* outputVertices, std::vector<int>* outputTriangles, bool print_debug_info=false);

    unsigned int m_counter = 0;

};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
