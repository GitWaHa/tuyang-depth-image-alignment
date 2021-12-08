cmake_minimum_required(VERSION 2.8.3)
project(tuyang-depth-image-alignment)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  sensor_msgs
  message_filters
)

# PCL 
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ========================================
# === OpenCV
# ========================================
find_package(OpenCV REQUIRED)
if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV library not found")
else()
    include_directories(${OpenCV_INCLUDE_DIRS})
    include_directories(${OpenCV2_INCLUDE_DIRS})
    # if(CMAKE_BUILD_TYPE STREQUAL Debug)
    # link_directories(${OpenCV_LIB_DIR_DBG})
    # else()
    # link_directories(${OpenCV_LIB_DIR_OPT})
    # endif()
    link_directories(${OpenCV_LIB_DIR})
endif()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tuyang-depth-image-alignment
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if (NOT TARGET tycam)
    #only build samples
    # option (OpenCV_STATIC OFF)
    # set(INCLUDE_PATH ../include)
    # include_directories(${INCLUDE_PATH})
    set(ABSOLUTE_TARGET_LIB tycam)
    add_library(${ABSOLUTE_TARGET_LIB} SHARED IMPORTED)
    if (MSVC)#for windows
        # set (LIB_ROOT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../lib/win/hostapp/)
        # if(CMAKE_CL_64) #x64
        #     set_property(TARGET ${ABSOLUTE_TARGET_LIB} PROPERTY IMPORTED_LOCATION ${LIB_ROOT_PATH}/x64/tycam.dll)
        #     set_property(TARGET ${ABSOLUTE_TARGET_LIB} PROPERTY IMPORTED_IMPLIB  ${LIB_ROOT_PATH}/x64/tycam.lib)
        # else()
        #     set_property(TARGET ${ABSOLUTE_TARGET_LIB} PROPERTY IMPORTED_LOCATION ${LIB_ROOT_PATH}/x86/tycam.dll)
        #     set_property(TARGET ${ABSOLUTE_TARGET_LIB} PROPERTY IMPORTED_IMPLIB ${LIB_ROOT_PATH}/x86/tycam.lib)
        # endif()
    else()
      if(ARCH)
          set_property(TARGET ${ABSOLUTE_TARGET_LIB} PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../lib/linux/lib_${ARCH}/libtycam.so)
      else()
          set(ABSOLUTE_TARGET_LIB -ltycam)
      endif()
    endif()
endif()

# ========================================
# === common, build static lib to speed up
# ========================================
set(COMMON_SOURCES include/common/MatViewer.cpp include/common/TYThread.cpp)

add_library(sample_common STATIC ${COMMON_SOURCES})
if(UNIX)
  target_link_libraries(sample_common pthread)
endif()

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/tuyang-depth-image-alignment.cpp
# )

add_executable(depth2rgb_align src/test.cpp
)
add_dependencies(depth2rgb_align sample_common ${TARGET_LIB})
target_link_libraries(depth2rgb_align sample_common ${catkin_LIBRARIES} ${ABSOLUTE_TARGET_LIB} ${OpenCV_LIBS})

add_executable(align_server src/alignment_server.cpp
  src/alignment.cpp
)
add_dependencies(align_server sample_common ${TARGET_LIB})
target_link_libraries(align_server sample_common ${catkin_LIBRARIES} ${ABSOLUTE_TARGET_LIB} ${OpenCV_LIBS} ${CLOUD_VIEWER})

add_executable(pub_pointcloud src/pub_pointcloud.cpp
)
target_link_libraries(pub_pointcloud
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)
