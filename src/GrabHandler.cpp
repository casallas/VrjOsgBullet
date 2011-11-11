/*
* (C) Copyright 2010-2013
* Iowa State University
* Arts et M�tiers Paristech - Institut Image 
*
* Author: Juan Sebastian Casallas
*
* Based on osgbInteraction/DragHandler from  osgBullet (LGPL)
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
#include "GrabHandler.h"
#include "VrjOsgBulletApp.h"
#include <osgbCollision/RefBulletObject.h>
#include <osgbCollision/Utils.h>
#include <osgbDynamics/MotionState.h>
#include <osgbDynamics/PhysicsThread.h>

#include <osgGA/GUIEventHandler>
#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/IntersectionVisitor>

#include <btBulletDynamicsCommon.h>

#include <osg/io_utils>
#include <sstream>


namespace ii_vrac
{
	GrabHandler::GrabHandler( btDynamicsWorld* dw, VrjOsgBulletApp* mainApp )
		: _dw( dw ),
		_mainApp( mainApp ),
		_constraint( NULL ),
		_constrainedMotionState( NULL ),
		_pt( NULL )
	{
	}
	GrabHandler::~GrabHandler()
	{
	}

	bool GrabHandler::handle(  )
	{
		if( _mainApp->mGrabBtn->getData() == gadget::Digital::ON )
		{
			/// Get the wand matrix in the units of this application.
			const gmtl::Matrix44f wand_mat(_mainApp->mWand->getData(_mainApp->getDrawScaleFactor()));
			//std::cout << "scale factor: " << getDrawScaleFactor() << std::endl;

			// Get the point in space where the wand is located.
			gmtl::Point3f wand_pt = gmtl::makeTrans<gmtl::Point3f>(wand_mat);

			// Get the wand's orientation
			gmtl::Quatf wand_ori = gmtl::makeRot<gmtl::Quatf>(wand_mat);

			if (!_constraint)
			{
				const bool picked = intersects( btVector3(wand_pt[0], wand_pt[1], wand_pt[2]) );

				if( picked )
					_constraint->getRigidBodyA().activate( true );

				return( picked );
			}
			else
			{
				osg::Vec3 wand_point = osg::Vec3( wand_pt[0], wand_pt[1], wand_pt[2] );
				osg::notify( osg::DEBUG_FP ) << "  OSG point " << wand_point << std::endl;

				if( _pt != NULL )
					_pt->pause( true );

				osg::Matrix ow2bw;
				if( _constrainedMotionState != NULL )
					ow2bw = _constrainedMotionState->computeOsgWorldToBulletWorld();
				osg::Vec3d bulletPoint = wand_point * ow2bw;
				osg::notify( osg::DEBUG_FP ) << "    bullet point " << bulletPoint << std::endl;

				_constraint->setPivotB( osgbCollision::asBtVector3( bulletPoint ) );

				if( _pt != NULL )
					_pt->pause( false );

				return( true );
			}
		}
		else if( _constraint && _mainApp->mGrabBtn->getData() != gadget::Digital::ON )
		{

			if( _constraint == NULL )
				return( false );

			if( _pt != NULL )
				_pt->pause( true );

			_dw->removeConstraint( _constraint );

			if( _pt != NULL )
				_pt->pause( false );

			delete _constraint;
			_constraint = NULL;
			_constrainedMotionState = NULL;
			return( true );
		}

		return( false );
	}

	void GrabHandler::setThreadedPhysicsSupport( osgbDynamics::PhysicsThread* pt )
	{
		_pt = pt;
	}

	bool GrabHandler::intersects( const btVector3& wand_pos )
	{
		bool picked = false;

		//Create a ray from the wand's previous position to its current one
		btCollisionWorld::ClosestRayResultCallback rayCallback(_lastPos,wand_pos);
		_dw->rayTest(_lastPos,wand_pos,rayCallback);
		if (rayCallback.hasHit())
		{
			btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
			if (body)
			{
				// Save the MotionState for this rigid body. We'll use it during the DRAG events.
				_constrainedMotionState = dynamic_cast< osgbDynamics::MotionState* >( body->getMotionState() );
				osg::Matrix ow2col;
				if( _constrainedMotionState != NULL )
					ow2col = _constrainedMotionState->computeOsgWorldToCOLocal();

				//osg::Vec3d pickPointBulletOCLocal = pickPointWC * ow2col;
				//osg::notify( osg::DEBUG_FP ) << "pickPointWC: " << pickPointWC << std::endl;
				//osg::notify( osg::DEBUG_FP ) << "pickPointBulletOCLocal: " << pickPointBulletOCLocal << std::endl;

				// We now have the intersetionPoint and a pointer to the btRigidBody.
				// Make a Bullet point-to-point constraint, so we can drag it around.
				_constraint = new btPoint2PointConstraint( *body, wand_pos );

				if( _pt != NULL )
					_pt->pause( true );

				_dw->addConstraint( _constraint );

				if( _pt != NULL )
					_pt->pause( false );

				picked = true;
			}
		}
		_lastPos = wand_pos;
		return picked;
	}
}//ii_vrac
