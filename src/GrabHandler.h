/*
 * (C) Copyright 2010-2013
 * Iowa State University
 * Arts et Métiers Paristech - Institut Image 
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
#ifndef GRAB_HANDLER
#define GRAB_HANDLER

#include <osgbDynamics/MotionState.h>
#include <btBulletDynamicsCommon.h>
#include <osg/Vec4>

// Forward declaring
class VrjOsgBulletApp;

namespace osgbDynamics {
	class PhysicsThread;
}


namespace ii_vrac
{


	/** \class DragHandler DragHandler.h <osgbInteraction\DragHandler.h>
	\brief An event handler for selecting and dragging rigid bodies.

	To use this event handler, simply create an instance of it and add it to your
	osgViewer::Viewer.

	During a ctrl-leftmouse click, DragHandler does an intersection test with
	the \c scene. The test succeeds if the picked object has a Node in its NodePath
	containing a RefRigidBody stored in the Node's UserData. DragHandler then adds
	the picked btRigidBody to a new btPoint2PointConstraint and adds it to the
	dynamics world. DragHandler also computes a drag plane, orthogonal to the view
	direction and containing the intersection point.

	On subsequent ctrl-leftmouse drag events, DragHandler back-transform the mouse
	position to create a world space ray, and intersects it with the DragPlane.
	DragHandler then sets this intersection point in the constraint.

	On a leftmouse release event, DragHandler removes the constraint and deletes it.
	*/
	class GrabHandler
	{
	public:
		/** \brief Constructor.
		\param dw The Bullet dynamics world. When the DragHandler creates a
		btPoint2PointConstraint, it adds it to this dynamics world.
		\param scene Scene graph used for picking. \c scene must be a Camera node
		to allow DragHandler to properly convert from window to world coordinates
		during selection and dragging. */
		GrabHandler( btDynamicsWorld* dw, VrjOsgBulletApp* mainApp );

		/** \brief Handle events.

		Controls:
		\li ctrl-left-mouse Select and drag a rigid body.
		*/
		virtual bool handle( );

		/** \brief Support for running the Bullet physics simultation in a separate thread.

		Call this function to specify the osgbDynamics::PhysicsThread. DragHandler pauses
		and unpauses the thread during constraint creation, modification, and deletion. */
		void setThreadedPhysicsSupport( osgbDynamics::PhysicsThread* pt );

	protected:
		~GrabHandler();

		/** \brief Picking support.
		\param wx Normalized (-1.0 to 1.0) x mouse position
		\param wy Normalized (-1.0 to 1.0) Y mouse position
		*/
		bool pick( float wx, float wy, float wz );

		btDynamicsWorld* _dw;
		VrjOsgBulletApp* _mainApp;

		btPoint2PointConstraint* _constraint;
		osgbDynamics::MotionState* _constrainedMotionState;

		osgbDynamics::PhysicsThread* _pt;
	};
}//ii_vrac


//GRAB_HANDLER
#endif