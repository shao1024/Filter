cmake_minimum_required(VERSION 3.0)
project(Filter)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 11)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# find_package(Eigen3
# )


include_directories(
    ${PROJECT_SOURCE_DIR}/include
    "/usr/include/eigen3"
)


add_executable(PF_CV_RUN test/PF_RUN.cpp src/particleFilter_CV.cpp)
target_link_libraries(PF_CV_RUN )

add_executable(EKF_CV_RUN test/EKF_RUN.cpp src/KF_CV.cpp)
target_link_libraries(EKF_CV_RUN)

add_executable(PF_GenData test/PF_GenData.cpp)
target_link_libraries(PF_GenData )