cmake_minimum_required(VERSION 3.9)
project(VIO_Learning)

set(CMAKE_CXX_STANDARD 11)
find_package( Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV QUIET)
#find_package(Eigen3 REQUIRED)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${Sophus_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}

)

set(Sophus_LIBRARIES libSophus.so)

set(Sophus_LIBRARIES libSophus.so)
add_executable(L1Q2 L1Q2.cpp)
target_link_libraries( L1Q2 ${Sophus_LIBRARIES}  ${Pangolin_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBS})



