cmake_minimum_required(VERSION 3.16)
project(ORB_SLAM2)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

find_package(OpenCV REQUIRED )
find_package(Eigen3  REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)
find_package(yaml-cpp REQUIRED)


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

#if(EXISTS "${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2")
set(DBoW2_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/include")
set(DBoW2_LIBRARIES "${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so")

#else()
    #find_package(DBoW2 REQUIRED)
#endif()

#if(EXISTS "${PROJECT_SOURCE_DIR}/Thirdparty/g2o")
set(g2o_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/Thirdparty/g2o/include")
set(g2o_LIBRARIES "${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so")

#    g2o_INCLUDE_DIRS="${PROJECT_SOURCE_DIR}/Thirdparty/g2o/include"
#    g2o_LIBRARIES="${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so"
#else()
    find_package(g2o REQUIRED)
#endif()

target_include_directories(${PROJECT_NAME} PUBLIC
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${DBoW2_INCLUDE_DIRS}
        #${g2o_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        Eigen3::Eigen
        ${YAML_CPP_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${DBoW2_LIBRARIES}
        ${g2o_LIBRARIES}
)

# Build examples
add_executable(mono Examples/mono/mono_vslamlab.cpp)
target_link_libraries(mono ${PROJECT_NAME})

add_executable(rgbd Examples/RGB-D/rgbd_vslamlab.cpp)
target_link_libraries(rgbd ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} mono rgbd
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)