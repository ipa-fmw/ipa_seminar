cmake_minimum_required(VERSION 2.8.3)
project(kr16_deployment)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
  	rospy
)

# protected region additional user defined REQUIREMENTS on begin #
# protected region additional user defined REQUIREMENTS end #


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(common/include)
include_directories(common/src)
include_directories(${catkin_INCLUDE_DIRS})


## Declare a cpp executable

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

## Specify libraries to link a library or executable target against

# protected region additional user defined BUILD STATEMENTS on begin #
# protected region additional user defined BUILD STATEMENTS end #


#############
## Install ##
#############

## Mark executables and/or libraries for installation

# protected region user Cmake macros on begin #
# protected region user Cmake macros end #
