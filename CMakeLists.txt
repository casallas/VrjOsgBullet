# Minimal VR Juggler+OpenSourceGraph+osgBullet Build
# CMake cross-platform build system recipe
#
# Based on vrjuggler-osgnav-example:
# 2009 Ryan Pavlik <rpavlik@iastate.edu>
# http://academic.cleardefinition.com
# https://github.com/rpavlik/cmake-modules
#
# Modified for VrjOsgNav:
# 2012 Juan Sebastian Casallas
# <casallas@iastate.edu>
#
# Iowa State University HCI Graduate Program/VRAC

cmake_minimum_required(VERSION 2.6)

# Set package properties
project(osgNav)

###
# CMake Modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(UseBackportedModules)
include(SearchProgramFilesForOpenSceneGraph)
include( FindBulletHelper )
include(CreateLaunchers)
include(GetDirectoryList)

###
# Dependencies

# VR Juggler
find_package(VRJuggler 3.0 REQUIRED)
include_directories(${VRJUGGLER_INCLUDE_DIRS})
list(APPEND RUNTIME_LIBRARY_DIRS ${VRJUGGLER_RUNTIME_LIBRARY_DIRS})
add_definitions(${VRJUGGLER_DEFINITIONS})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${VRJUGGLER_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${VRJUGGLER_CXX_FLAGS}")

# OpenSceneGraph
find_package(OpenSceneGraph REQUIRED COMPONENTS osgUtil osgDB)
include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})
get_directory_list(OPENSCENEGRAPH_RUNTIME_LIBRARY_DIRS ${OPENSCENEGRAPH_LIBRARIES})
list(APPEND RUNTIME_LIBRARY_DIRS ${OPENSCENEGRAPH_RUNTIME_LIBRARY_DIRS})
if(WIN32)
	list(APPEND RUNTIME_LIBRARY_DIRS ${OPENSCENEGRAPH_RUNTIME_LIBRARY_DIRS}/../bin)
endif()

# osgWorks
FIND_PACKAGE( OSGWorks REQUIRED )
include_directories(${OSGWORKS_INCLUDE_DIR})
#list(APPEND RUNTIME_LIBRARY_DIRS ${OSGWORKS_LIBRARIES})

# Bullet
include_directories(
    ${BULLET_INCLUDE_DIRS}
)
set(BULLET_LIBRARIES
            ${BULLET_COLLISION_LIBRARY}
            ${BULLET_MATH_LIBRARY}
            ${BULLET_DYNAMICS_LIBRARY}
)


###
# Build the project
###
set(SOURCES
	src/main.cpp
	src/nav.h
	src/VrjOsgBulletApp.cpp
	src/VrjOsgBulletApp.h)

add_executable(VrjOsgBulletApp ${SOURCES})
target_link_libraries(VrjOsgBulletApp
	${OPENSCENEGRAPH_LIBRARIES}
	${OSGWORKS_LIBRARIES}
	${BULLET_LIBRARIES}
	${VRJUGGLER_LIBRARIES})
create_default_target_launcher(osgnav
	FORWARD_ARGS
	RUNTIME_LIBRARY_DIRS
	${RUNTIME_LIBRARY_DIRS}
	ENVIRONMENT
	${VRJUGGLER_ENVIRONMENT})
