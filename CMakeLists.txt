cmake_minimum_required(VERSION 3.16)
project(ORB_SLAM2)

# Compiler settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Release)

# Output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# Compiler flags
add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)

# Dependencies
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)
find_package(yaml-cpp REQUIRED)

# DBoW2 and g2o from Thirdparty folders
set(DBoW2_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/include")
set(DBoW2_LIBRARIES "${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so")

set(g2o_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/Thirdparty/g2o/include")
set(g2o_LIBRARIES "${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so")

# Create shared library
add_library(${PROJECT_NAME} SHARED
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
        src/MapDrawer.cc
        src/Optimizer.cc
        src/PnPsolver.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Sim3Solver.cc
        src/Initializer.cc
        src/Viewer.cc
)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${DBoW2_INCLUDE_DIRS}
    ${g2o_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    Eigen3::Eigen
    ${YAML_CPP_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${DBoW2_LIBRARIES}
    ${g2o_LIBRARIES}
)

# Example executables
add_executable(vslamlab_orbslam2_mono Examples/mono/vslamlab_orbslam2_mono.cpp)
target_link_libraries(vslamlab_orbslam2_mono ${PROJECT_NAME})

# Install targets and headers
install(TARGETS ${PROJECT_NAME} vslamlab_orbslam2_mono
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)