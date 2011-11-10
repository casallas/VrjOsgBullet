/*
 * (C) Copyright 2010-2013
 * Iowa State University
 * Arts et Métiers Paristech - Institut Image 
 *
 * Author: Juan Sebastian Casallas
 *
 * Based on OsgNav.cpp from the osgNav sample of vrjuggler2 (LGPL)
 * and the Hinge example from  osgBullet (LGPL)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * (COPYING) and the GNU Lesser General Public License (COPYING.LESSER)
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * app-specific includes
 */
#include "VrjOsgBulletApp.h"

/**
 * osg includes
 */
#include <osg/Math>
#include <osg/Geode>
#include <osg/Material>
#include <osg/Vec3>
#include <osgUtil/Optimizer>
#include <osgDB/ReadFile>

/**
 * vrj includes
 */
#include <gmtl/Vec.h>
#include <gmtl/Coord.h>
#include <gmtl/Xforms.h>
#include <gmtl/Math.h>

/**
 * osgBullet includes
 */
#include <osgwTools/AbsoluteModelTransform.h>

#include <osgbDynamics/GroundPlane.h>
#include <osgbDynamics/MotionState.h>
#include <osgbDynamics/RigidBody.h>
#include <osgbCollision/RefBulletObject.h>
#include <osgbCollision/Utils.h>

#include <sstream>

VrjOsgBulletApp::VrjOsgBulletApp(vrj::Kernel* kern, int& argc, char** argv)
: vrj::osg::App(kern)
, mFileToLoad("")
{	
}

VrjOsgBulletApp::~VrjOsgBulletApp()
{
}

void VrjOsgBulletApp::exit()
{
#ifdef VOB_PHYSICS_THREAD
	mPhysicsThread->stopPhysics();
	mPhysicsThread->join();
	delete mPhysicsThread;
#endif
}

void VrjOsgBulletApp::updatePhysics(float time_delta)
{
#ifdef VOB_DEBUG_DRAW
	//Start drawing on the debug drawer (if any)
	if( mDbgDraw != NULL )
		mDbgDraw->BeginDraw();
#endif
	
#ifndef VOB_PHYSICS_THREAD
	//Step simulation according to the given time_delta
	mDynamicsWorld->stepSimulation( time_delta );
#else
	//Update the motion states in the triple buffer
	TripleBufferMotionStateUpdate( mMotionStates, &mTripleBuffer );
#endif

#ifdef VOB_DEBUG_DRAW
	//Stop drawing on the debug drawer (if any)
	if( mDbgDraw != NULL )
	{
		mDynamicsWorld->debugDrawWorld();
		mDbgDraw->EndDraw();
	}
#endif
}

void VrjOsgBulletApp::latePreFrame()
{
	gmtl::Matrix44f world_transform;
	gmtl::invertFull(world_transform, mNavigator.getCurPos());
	// Update the scene graph
	osg::Matrix osg_current_matrix;
	osg_current_matrix.set(world_transform.getData());
	mNavTrans->setMatrix(osg_current_matrix);
	
	// Finish updating the scene graph.
	vrj::osg::App::latePreFrame();
}

void VrjOsgBulletApp::preFrame()
{
	vpr::Interval cur_time = mWand->getTimeStamp();
	vpr::Interval diff_time(cur_time-mLastPreFrameTime);
	if (mLastPreFrameTime.getBaseVal() >= cur_time.getBaseVal())
	{  
		diff_time.secf(0.0f);
	}
	
	float time_delta = diff_time.secf();
	
	mLastPreFrameTime = cur_time;
		
	// Update physics
	updatePhysics(time_delta);
	
	// Update the navigation using the time delta between
	mNavigator.update(time_delta);
}

void VrjOsgBulletApp::bufferPreDraw()
{
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClear(GL_COLOR_BUFFER_BIT);
}

void VrjOsgBulletApp::initScene()
{
	// Initialize devices
	const std::string wand("VJWand");
	const std::string vjhead("VJHead");
	
	mWand.init(wand);
	mHead.init(vjhead);
	
	// Init physics
	mDynamicsWorld = initPhysics();
	// Init the scenegraph
	initSceneGraph();
}

btDynamicsWorld* VrjOsgBulletApp::initPhysics()
{
    btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher * dispatcher = new btCollisionDispatcher( collisionConfiguration );
    btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;
	
    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );
	
	btDynamicsWorld * dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );
	
    dynamicsWorld->setGravity( btVector3( 0, -9.8, 0 ) );

#ifdef VOB_PHYSICS_THREAD
	mTripleBuffer.resize( 16384 );
    mPhysicsThread = new osgbDynamics::PhysicsThread( dynamicsWorld, &mTripleBuffer );
	
	mPhysicsThread->setProcessorAffinity( 0 );
    mPhysicsThread->start();
#endif

    return( dynamicsWorld );
}

void VrjOsgBulletApp::initSceneGraph()
{
	// mRootNode
	//         \-- mNavTrans -- mModelTrans -- (AbsoluteTransform) -- mModel
	//						\-- (ground plane)
	
	//The top level nodes of the tree
	mRootNode = new osg::Group();
	mNavTrans = new osg::MatrixTransform();
	
	mNavigator.init();
	
	mRootNode->addChild(mNoNav.get());
	mRootNode->addChild(mNavTrans.get());
	
	//Make a floor and add it to the navTrans (non-rotated)
	mNavTrans->addChild( 
	 osgbDynamics::generateGroundPlane( osg::Vec4( 0.f, 1.f, 0.f, 0.f ), mDynamicsWorld )
	 );
	
	// Transform node for the model
	mModelTrans  = new osg::MatrixTransform();
	//This can be used if the model orientation needs to change
	mModelTrans->preMult(osg::Matrix::rotate(gmtl::Math::deg2Rad(-90.0f),
											 1.0f, 0.0f, 0.0f));

	//Load the model
	std::cout << "Attempting to load file: " << mFileToLoad << "... ";
	mModel = osgDB::readNodeFile(mFileToLoad);
	std::cout << "done." << std::endl;
		
	if ( ! mModel.valid() )
	{
		std::cout << "ERROR: Could not load file: " << mFileToLoad << std::endl;
	}
	else
	{
		//Place the model 10mt above the ground
		osg::Matrix m( osg::Matrix::translate( 0.f, 0.f, 10.f ) );
		
		//Create rigid body from the model and add it to the transform
		mDynamicsWorld->addRigidBody(createObject( mModel.get(), mModelTrans, m ) );
	}
	
	// Add the transform to the tree
	mNavTrans->addChild(mModelTrans.get());
	
#ifdef VOB_DEBUG_DRAW
	//Create a debug display and add it to the root
	mDbgDraw = new osgbCollision::GLDebugDrawer();
	mDbgDraw->setDebugMode( ~btIDebugDraw::DBG_DrawText );
	mDynamicsWorld->setDebugDrawer( mDbgDraw );
	mRootNode->addChild( mDbgDraw->getSceneGraph() );
#endif
}

//Creates a rigid object from the given node, attachs it to a new AbsoluteModelTransform and attachs the latter to the given parent
btRigidBody* VrjOsgBulletApp::createObject( osg::Node* node, osg::Group* parent, const osg::Matrix& m )
{
	//Bullet should control the absolute transform
    osgwTools::AbsoluteModelTransform* mt = new osgwTools::AbsoluteModelTransform;
	parent->addChild( mt );
    mt->addChild( node );
	
    osg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
    //No need to set the center of mass on the cow
	//cr->setCenterOfMass( osg::Vec3(0,0,0) );
    cr->_sceneGraph = mt;
	cr->_shapeType = CONVEX_HULL_SHAPE_PROXYTYPE;
    //The parent transform should be the world transform of the parent premultiplied by the initial transform of the object
	cr->_parentTransform = m*parent->getWorldMatrices()[0];
	//Set some mass, it's value shouldn't really matter as long as its possitive
	cr->_mass = 1.f;
	cr->_restitution = 1.f;
    btRigidBody* rb = osgbDynamics::createRigidBody( cr.get() );
	
#ifdef VOB_PHYSICS_THREAD
	// Set up for multithreading and triple buffering.
    osgbDynamics::MotionState* motion = static_cast< osgbDynamics::MotionState* >( rb->getMotionState() );
    motion->registerTripleBuffer( &mTripleBuffer );
    mMotionStates.insert( motion );
#endif
		
	//Save the rigid body as userData of the absoluteModelTransform
    mt->setUserData( new osgbCollision::RefRigidBody( rb ) );
    std::ostringstream id;
    id << std::hex << mt;
    
    return( rb );
}
