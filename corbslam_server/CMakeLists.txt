cmake_minimum_required(VERSION 2.8.3)
project(corbslam_server)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        corbslam_client
        corbslam_msgs
        dbow2_catkin
        g2o_catkin
        opencv2_catkin
        eigen_catkin
        pcl_ros
        roscpp
        rospy
        std_msgs
        message_generation
        tf
        )


#find_package(corbslam_client REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost COMPONENTS serialization system filesystem REQUIRED)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
)

cs_add_library(${PROJECT_NAME}
        src/MapFusion.cpp
        src/ServerMap.cpp
        src/PubToClient.cpp
        src/GlobalOptimize.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )

cs_add_executable(corbslam_server_node
        src/main.cpp
)

target_link_libraries(corbslam_server_node
        ${PROJECT_NAME}
        )

cs_export()
cs_install()