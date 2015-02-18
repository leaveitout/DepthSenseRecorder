cmake_minimum_required(VERSION 2.8.4)

project(DepthSenseRecorder)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules/")

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(Boost REQUIRED)                   ## Removed this dependency, using C++11 threads.
find_package(OpenCV REQUIRED)
find_package(DepthSense REQUIRED)
#find_package(PCL REQUIRED)
find_package(PCL REQUIRED COMPONENTS io common visualization sample_consensus segmentation)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(/usr/include)
include_directories({Boost_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIR})
include_directories(${DEPTHSENSE_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${CMAKE_SOURCE_DIR}/lib)
link_directories(/usr/lib)
#link_directories(${Boost_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

if(APPLE)
  include_directories(/usr/local/include)
  link_directories(/usr/local/lib)
elseif(UNIX)
  include_directories(/usr/local/include)
  link_directories(/usr/local/lib)
endif()

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)
#set(CMAKE_CXX_FLAGS "-std=c++0x -fpermissive -Wno-deprecated -Wno-sign-compare")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_LINK_FLAGS_DEBUG "-pg")

ADD_DEFINITIONS(${PCL_DEFINITIONS})

set(SOURCE_FILES
    #src/ColorCamera.cpp src/DepthCamera.cpp src/DS325.cpp src/DS325Capture.cpp)
    #src/ColorCamera.cpp src/DepthCamera.cpp src/DS325.cpp src/DS325Record.cpp)
    src/ColorCamera.cpp src/DepthCamera.cpp src/DS325.cpp src/DS325PlanarSegmentation.cpp)

set(LIB_EXTERNAL
    ${Boost_LIBRARIES} ${OpenCV_LIBS} ${DEPTHSENSE_LIBRARIES} ${PCL_LIBS})
    #${OpenCV_LIBS} ${DEPTHSENSE_LIBRARIES} ${PCL_LIBS})

add_executable(DepthSenseRecorder ${SOURCE_FILES})

#target_link_libraries(DepthSenseRecorder
#  ${Boost_FILESYSTEM_LIBRARY}
#  ${Boost_SYSTEM_LIBRARY}
#  ${Boost_MATH_LIBRARY}
#  ${Boost_REGEX_LIBRARY}
#)

target_link_libraries(DepthSenseRecorder ${LIB_EXTERNAL})

target_link_libraries(DepthSenseRecorder ${PCL_COMMON_LIBRARY})
target_link_libraries(DepthSenseRecorder ${PCL_VISUALIZATION_LIBRARY})
target_link_libraries(DepthSenseRecorder ${PCL_SEGMENTATION_LIBRARY})


