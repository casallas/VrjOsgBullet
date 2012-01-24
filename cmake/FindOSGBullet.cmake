# Locate osgBullet.
# Requires osgWorks 2.0 (FindOSGWorks.cmake)
# and Bullet v2.76 - v2.79 (FindBulletHelper.cmake)
#
# This script defines:
#   OSGBULLET_FOUND, set to 1 if found
#   OSGBULLET_LIBRARIES
#   OSGBULLET_INCLUDE_DIR
#   OSGBCOLLISION_LIBRARY
#   OSGBDYNAMICS_LIBRARY
#   OSGBINTERACTION_LIBRARY
#   OSGDB_OSGBDYNAMICS_LIBRARY
#   OSGDB_SGB_LIBRARY
#
# This script will look in standard locations for installed osgBullet. However, if you
# install osgBullet into a non-standard location, you can use the OSGBULLET_ROOT
# variable (in environment or CMake) to specify the location.
#
# You can also use osgBullet out of a source tree by specifying OSGBULLET_SOURCE_DIR
# and OSGBULLET_BUILD_DIR (in environment or CMake).
#
# Based on FindOSGWorks.cmake from osgBullet
# osgBullet is (C) 2009-2012 by Kenneth Mark Bryden
# licensed under the GNU LGPL v2.1 software license.
# http://code.google.com/p/osgbullet
#
# Modified for osgBullet 2.0 by
# Juan Sebastian Casallas:
# <casallas@iastate.edu>


set( _osgBulletSearchPathS
    /usr/local
    /usr
    /sw/ # Fink
    /opt/local # DarwinPorts
    /opt/csw # Blastwave
    /opt
    "C:/Program Files/osgBullet"
    "C:/Program Files (x86)/osgBullet"
    ~/Library/Frameworks
    /Library/Frameworks
)

find_path( OSGBULLET_INCLUDE_DIR
    osgbDynamics/MotionState.h
    HINTS
        ${OSGBULLET_ROOT}
        $ENV{OSGBULLET_ROOT}
        ${OSGBULLET_SOURCE_DIR}
        $ENV{OSGBULLET_SOURCE_DIR}
    PATH_SUFFIXES
        include
    PATHS
        ${_osgBulletSearchPathS}
)
mark_as_advanced( OSGBULLET_INCLUDE_DIR )
# message( STATUS ${OSGBULLET_INCLUDE_DIR} )



macro( FIND_OSGBULLET_LIBRARY MYLIBRARY MYLIBRARYNAME )
    mark_as_advanced( ${MYLIBRARY} )
    mark_as_advanced( ${MYLIBRARY}_debug )
    find_library( ${MYLIBRARY}
        NAMES
            ${MYLIBRARYNAME}
        HINTS
            ${OSGBULLET_ROOT}
            $ENV{OSGBULLET_ROOT}
            ${OSGBULLET_BUILD_DIR}
            $ENV{OSGBULLET_BUILD_DIR}
        PATH_SUFFIXES
            lib
            bin
            bin/Release
        PATHS
            ${_osgBulletSearchPathS}
    )
    find_library( ${MYLIBRARY}_debug
        NAMES
            ${MYLIBRARYNAME}d
        HINTS
            ${OSGBULLET_ROOT}
            $ENV{OSGBULLET_ROOT}
            ${OSGBULLET_BUILD_DIR}
            $ENV{OSGBULLET_BUILD_DIR}
        PATH_SUFFIXES
            lib
            bin
            bin/Debug
        PATHS
            ${_osgBulletSearchPathS}
    )
#    message( STATUS ${${MYLIBRARY}} ${${MYLIBRARY}_debug} )
#    message( STATUS ${MYLIBRARYNAME} )
    if( ${MYLIBRARY} )
        list( APPEND OSGBULLET_LIBRARIES
            "optimized" ${${MYLIBRARY}}
        )
    endif()
    if( ${MYLIBRARY}_debug )
        list( APPEND OSGBULLET_LIBRARIES
            "debug" ${${MYLIBRARY}_debug}
        )
    endif()
#    message( STATUS ${OSGBULLET_LIBRARIES} )
endmacro()

unset( OSGWORKS_LIBRARIES )
FIND_OSGBULLET_LIBRARY( OSGBCOLLISION_LIBRARY osgbCollision )
FIND_OSGBULLET_LIBRARY( OSGBDYNAMICS_LIBRARY osgbDynamics )
FIND_OSGBULLET_LIBRARY( OSGBINTERACTION_LIBRARY osgbInteraction )
FIND_OSGBULLET_LIBRARY( OSGDB_OSGBDYNAMICS_LIBRARY osgdb_osgbdynamics )
FIND_OSGBULLET_LIBRARY( OSGDB_SGB_LIBRARY osgdb_sgb )

# handle the QUIETLY and REQUIRED arguments and set FMOD_FOUND to TRUE if all listed variables are TRUE
include( FindPackageHandleStandardArgs )
find_package_handle_standard_args(
    OSGBullet
    DEFAULT_MSG 
    OSGBULLET_LIBRARIES 
    OSGBULLET_INCLUDE_DIR
)
