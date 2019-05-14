cmake_minimum_required(VERSION 2.8.3)
project(sparse_bundle_adjustment)

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    roscpp
)
find_package(Eigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} )
add_definitions(${EIGEN_DEFINITIONS})

INCLUDE_DIRECTORIES(
    include
    ${Eigen_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${SuiteSparse_INCLUDE_DIRS}
    ${LAPACK_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_definitions(-DSBA_CHOLMOD)

catkin_package(
    LIBRARIES sba
    INCLUDE_DIRS include
    DEPENDS Eigen SuiteSparse
)

# SBA library
add_library(sba src/sba.cpp src/spa.cpp src/spa2d.cpp src/csparse.cpp src/proj.cpp src/node.cpp src/sba_file_io.cpp)
#rosbuild_add_compile_flags(sba ${SSE_FLAGS})
target_link_libraries(sba blas lapack cholmod cxsparse)


#add_library(sba_add src/nodes/sba_add.cpp)
add_executable(sba_add src/nodes/sba_add.cpp src/visualization.cpp)
target_link_libraries(sba_add sba ${catkin_LIBRARIES})


install(DIRECTORY include/sparse_bundle_adjustment/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS sba
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
