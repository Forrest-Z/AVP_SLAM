cmake_minimum_required(VERSION 3.5)
project(AVP_SLAM)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -w  -O0 -march=native -g -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w   -O0 -march=native -g -std=c++11")

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})


include_directories(
    modules/HdMap/include
    modules/AVP_SLAM/include

)
include_directories(${PCL_INCLUDE_DIRS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
    modules/HdMap/src/hdMap.cpp
    modules/HdMap/src/Motion.cpp
    modules/AVP_SLAM/src/Mapping.cpp
    modules/AVP_SLAM/src/LocalMapper.cpp
    modules/AVP_SLAM/src/Localizer.cpp
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(AVP_SIM Examples/AVP_SIM.cpp)
target_link_libraries(AVP_SIM ${PROJECT_NAME})

add_executable(AVP_MAPPING Examples/AVP_MAPPING.cpp)
target_link_libraries(AVP_MAPPING ${PROJECT_NAME})

add_executable(AVP_LOCALIZING Examples/AVP_LOCALIZING.cpp)
target_link_libraries(AVP_LOCALIZING ${PROJECT_NAME})

