cmake_minimum_required(VERSION 3.5)
project(AVP_SLAM)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -w  -O0 -march=native -g -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w   -O0 -march=native -g -std=c++11")

find_package(OpenCV REQUIRED)

include_directories(
    modules/AVP_SIM/include
    modules/AVP_SLAM/include
    modules/Messages/include
    3rdParty/lcm.include
)

link_directories("3rdParty/lcm/lib")

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
    modules/AVP_SIM/src/Map.cpp
    modules/AVP_SIM/src/System.cpp
    modules/AVP_SIM/src/Motion.cpp
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} lcm)

add_executable(AVP_SIM Examples/AVP_SIM.cpp)
target_link_libraries(AVP_SIM ${PROJECT_NAME})

add_executable(AVP_MAPPING Examples/AVP_MAPPING.cpp)
target_link_libraries(AVP_MAPPING ${PROJECT_NAME})
