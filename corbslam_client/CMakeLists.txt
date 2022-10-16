cmake_minimum_required(VERSION 2.8)
project(corbslam_client)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

### find and configure Boost
find_package(Boost COMPONENTS serialization system filesystem REQUIRED)

include_directories(include
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/Examples/RGB-D
)

## Declare a C++ library

cs_add_library(${PROJECT_NAME} 
        src/System.cc
        src/Tracking.cc
        src/LocalMapping.cc
        src/LoopClosing.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/FrameDrawer.cc
        src/Converter.cc
        src/MapPoint.cc
        src/KeyFrame.cc
        src/Map.cc
        src/Optimizer.cc
        src/PnPsolver.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Sim3Solver.cc
        src/Initializer.cc
        src/Cache.cc
        src/LightKeyFrame.cc
        src/LightMapPoint.cc
        src/DataDriver.cc
        src/TransPose.cpp)

add_dependencies (${PROJECT_NAME} g2o DBoW2)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(orbslam_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable

# Build examples

cs_add_executable(corbslam_client_rgbd_ros
        Examples/ROS/ORB_SLAM2/src/ros_rgbd.cc)

target_link_libraries(corbslam_client_rgbd_ros
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
        )

#add_executable(orbslam_client_node_mono_kitti
#        Examples/Monocular/mono_kitti.cc)
#target_link_libraries(orbslam_client_node_mono_kitti ${PROJECT_NAME})
#
#add_executable(orbslam_client_node_mono_euroc
#        Examples/Monocular/mono_euroc.cc)
#target_link_libraries(orbslam_client_node_mono_euroc ${PROJECT_NAME})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark executables and/or libraries for installation

#

cs_export()
cs_install()

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_orbslam_client.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
