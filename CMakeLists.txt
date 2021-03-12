cmake_minimum_required(VERSION 3.0.2)
project(cam_radar_fusion)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        image_geometry 
        pcl_conversions
        pcl_ros
        roscpp
        tf
        )

find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)

find_package(Eigen3 QUIET)

if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

catkin_package(CATKIN_DEPENDS
        cv_bridge
        image_transport
        pcl_conversions
        pcl_ros
        roscpp
        tf
        )

set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -g -Wall ${CMAKE_CXX_FLAGS}")

#fusion Library
add_library(cam_radar_fusion_lib SHARED
        src/cam_radar_fusion.cpp
        include/cam_radar_fusion/cam_radar_fusion.h
        )

if (OPENMP_FOUND)
    set_target_properties(cam_radar_fusion_lib PROPERTIES
            COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
            LINK_FLAGS ${OpenMP_CXX_FLAGS}
            )
endif ()

target_include_directories(cam_radar_fusion_lib PRIVATE
        ${OpenCV_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        include
        )

target_link_libraries(cam_radar_fusion_lib
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Qt5Core_LIBRARIES}
        )

add_dependencies(cam_radar_fusion_lib
        ${catkin_EXPORTED_TARGETS}
        )

#Fusion node
add_executable(cam_radar_fusion
        src/cam_radar_fusion_node.cpp
        )
target_include_directories(cam_radar_fusion PRIVATE
        ${OpenCV_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        include
        )

target_link_libraries(cam_radar_fusion
        cam_radar_fusion_lib)

install(TARGETS cam_radar_fusion
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
