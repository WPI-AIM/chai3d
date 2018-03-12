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
    \author    Francois Conti, Adnan Munawar
    \version   3.2.0 $Rev: 2015 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "CBulletWorld.h"
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cBulletWorld.
*/
//==============================================================================
cBulletWorld::cBulletWorld(std::string a_worldName)
{
    if(!a_worldName.empty()){
        m_rosWorldPtr.reset(new chai_env::World(a_worldName));
    }
    // reset simulation time
    m_simulationTime = 0.0;

    // integration time step
    m_integrationTimeStep = 0.001;

    // maximum number of iterations
    m_integrationMaxIterations = 5;

    // setup broad phase collision detection
    m_bulletBroadphase = new btDbvtBroadphase();

    // setup the collision configuration
    m_bulletCollisionConfiguration = new btDefaultCollisionConfiguration();

    // setup the collision dispatcher
    m_bulletCollisionDispatcher = new btCollisionDispatcher(m_bulletCollisionConfiguration);

    // register GIMPACT collision detector for GIMPACT objects
    btGImpactCollisionAlgorithm::registerAlgorithm(m_bulletCollisionDispatcher);

    // setup the actual physics solver
    m_bulletSolver = new btSequentialImpulseConstraintSolver;

    // setup the dynamic world
    m_bulletWorld = new btDiscreteDynamicsWorld(m_bulletCollisionDispatcher, m_bulletBroadphase, m_bulletSolver, m_bulletCollisionConfiguration);

    // assign gravity constant
    m_bulletWorld->setGravity(btVector3( 0.0, 0.0,-9.81));
}


//==============================================================================
/*!
    Destructor of cBulletWorld.
*/
//==============================================================================
cBulletWorld::~cBulletWorld()
{
    // clear all bodies
    m_bodies.clear();

    // delete resources
    delete m_bulletWorld;
    delete m_bulletSolver;
    delete m_bulletCollisionDispatcher;
    delete m_bulletCollisionConfiguration;
    delete m_bulletBroadphase;
}


//==============================================================================
/*!
    This methods defines a gravity field passed as argument.

    \param  a_gravity  Gravity field vector.
*/
//==============================================================================
void cBulletWorld::setGravity(const cVector3d& a_gravity)
{
    m_bulletWorld->setGravity(btVector3(a_gravity(0), a_gravity(1), a_gravity(2)));
}


//==============================================================================
/*!
    This methods returns the current gravity field vector.

    \return Current gravity field vector.
*/
//==============================================================================
cVector3d cBulletWorld::getGravity()
{
    btVector3 gravity = m_bulletWorld->getGravity();
    cVector3d result(gravity[0], gravity[1], gravity[2]);
    return (result);
}


//==============================================================================
/*!
    This methods updates the simulation over a time interval passed as
    argument.

    \param  a_interval  Time increment.
*/
//==============================================================================
void cBulletWorld::updateDynamics(double a_interval, double a_wallClock, double a_loopFreq, int a_numDevices)
{
    // sanity check
    if (a_interval <= 0) { return; }

    m_wallClock = a_wallClock;

    if(m_rosWorldPtr.get() != nullptr){
        while (!m_rosWorldPtr->step_sim()){
            usleep(1);
        }
    }
    // apply wrench from ROS
    list<cBulletGenericObject*>::iterator i;

    for(i = m_bodies.begin(); i != m_bodies.end(); ++i)
    {
        cBulletGenericObject* nextItem = *i;
        nextItem->updateCmdFromROS(a_interval);
    }

    // integrate simulation during an certain interval
    m_bulletWorld->stepSimulation(a_interval, m_integrationMaxIterations, m_integrationTimeStep);

    // add time to overall simulation
    m_simulationTime = m_simulationTime + a_interval;

    if (m_rosWorldPtr.get() != nullptr){
        m_rosWorldPtr->set_chai_sim_time(m_simulationTime);
        m_rosWorldPtr->set_chai_wall_time(m_wallClock);
        m_rosWorldPtr->set_loop_freq(a_loopFreq);
        m_rosWorldPtr->set_num_devices(a_numDevices);
    }

    // update CHAI3D positions for of all object
    updatePositionFromDynamics();
}


//==============================================================================
/*!
    This methods updates the position and orientation from the Bullet models 
    to CHAI3D models.
*/
//==============================================================================
void cBulletWorld::updatePositionFromDynamics()
{
    list<cBulletGenericObject*>::iterator i;

    for(i = m_bodies.begin(); i != m_bodies.end(); ++i)
    {
        cBulletGenericObject* nextItem = *i;
        nextItem->updatePositionFromDynamics();
        nextItem->updateROSMessageTime(&m_wallClock, &m_simulationTime);
    }
}

//==============================================================================
/*!
    This methods returns the current simulation time.
*/
//==============================================================================
double cBulletWorld::getSimulationTime(){
    return m_simulationTime;
}

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
