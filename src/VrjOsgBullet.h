/*
 * (C) Copyright 2010-2013
 * Iowa State University
 * Arts et Métiers Paristech - Institut Image 
 *
 * Author: Juan Sebastian Casallas
 *
 * Based on OsgNav.h from the osgNav sample of vrjuggler2 (LGPL)
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
#ifndef _OSG_NAV_
#define _OSG_NAV_

#include <vrj/vrjConfig.h>

#include <iostream>
#include <iomanip>
#include <math.h>

#include <gadget/Type/PositionInterface.h>
#include <gadget/Type/AnalogInterface.h>
#include <gadget/Type/DigitalInterface.h>

//OSG  includes
#include <osg/Matrix>
#include <osg/Transform>
#include <osg/MatrixTransform>

#include <osgUtil/SceneView>

#ifdef TWEEK_HAVE_CXX
#include <tweek/CORBA/CorbaManager.h>
#endif

#include "nav.h"

#include <vrj/Draw/OSG/App.h>

#include <btBulletDynamicsCommon.h>
#include <osgbCollision/GLDebugDrawer.h>

/**
 * Demonstration vrjuggler+osgBullet application class.
 */
class VrjOsgBullet : public vrj::osg::App
{
public:
	VrjOsgBullet(vrj::Kernel* kern, int& argc, char** argv);
	
	virtual ~VrjOsgBullet();
	
	/**
	 * Execute any initialization needed before the API is started.
	 *
	 * This is called once before OSG is initialized.
	 */
	virtual void initScene();
	
	///Initializes the scenegraph
	void initSceneGraph();
	
	///Initializes the dynamics world
	btDynamicsWorld* initPhysics();
	
	///Updates the debug drawer and does a simulation step
	void updatePhysics(float time_delta);
	
	///Creates a rigid body from the given node and transform m.
	btRigidBody* createObject( osg::Node* node, osg::Group* parent, const osg::Matrix& m );
	
	virtual osg::Group* getScene()
	{
		return mRootNode.get();
	}
	
	virtual void configSceneView(osgUtil::SceneView* newSceneViewer)
	{
		vrj::osg::App::configSceneView(newSceneViewer);
		
		newSceneViewer->getLight()->setAmbient(osg::Vec4(0.3f,0.3f,0.3f,1.0f));
		newSceneViewer->getLight()->setDiffuse(osg::Vec4(0.9f,0.9f,0.9f,1.0f));
		newSceneViewer->getLight()->setSpecular(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
	}
	
	void bufferPreDraw();
	
	// ----- Drawing Loop Functions ------
	//
	//  The drawing loop will look similar to this:
	//
	//  while (drawing)
	//  {
	//        preFrame();
	//       <Application Data Syncronization>
	//        latePreFrame();
	//       draw();
	//        intraFrame();     // Drawing is happening while here
	//       sync();
	//        postFrame();      // Drawing is now done
	//
	//       UpdateTrackers();
	//  }
	//------------------------------------
	
	/**
	 * Function called after tracker update but before start of drawing.
	 *
	 * Called once before every frame.
	 */
	virtual void preFrame();
	
	/** Function called after ApplicationData syncronization but before draw() */
	virtual void latePreFrame();
	
	/**
	 * Function called after drawing has been triggered but BEFORE it
	 * completes.
	 *
	 * Called once during each frame.
	 */
	virtual void intraFrame()
	{
		// Put your intra frame computations here.
	}
	
	/**
	 * Function called before updating trackers but after the frame is drawn.
	 *
	 * Called once after every frame.
	 */
	virtual void postFrame()
	{
		// Put your post frame computations here.
	}
	
	void setModelFileName(std::string filename)
	{
		mFileToLoad = filename;
	}
	
protected:
	osg::ref_ptr<osg::Group>           mRootNode;
	osg::ref_ptr<osg::Group>           mNoNav;
	osg::ref_ptr<osg::MatrixTransform> mNavTrans;
	osg::ref_ptr<osg::MatrixTransform> mModelTrans;
	osg::ref_ptr<osg::Node>            mModel;
	
	OsgNavigator  mNavigator;       /**< Navigation class */
	
	std::string mFileToLoad;
	
	/** Time of the start of the last preframe */
	vpr::Interval mLastPreFrameTime;
	
	//Physics Specifics
	btDynamicsWorld* mDynamicsWorld;	
	///Allows to see if the dynamics world matches the scene graph
	osgbCollision::GLDebugDrawer* mDbgDraw;
	bool mDebugDisplay;
	
#ifdef TWEEK_HAVE_CXX
	tweek::CorbaManager mCorbaManager;
#endif
	
public:
	gadget::PositionInterface  mWand;     /**< the wand */
	gadget::PositionInterface  mHead;     /**< the head */
};


#endif
