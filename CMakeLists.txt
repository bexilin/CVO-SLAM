cmake_minimum_required(VERSION 3.5.1)
project(cvo_slam)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
      message(STATUS "Using C++ 17")
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -march=native -qopenmp -O3 -vec")
   if(CMAKE_CXX_COMPILER_ID MATCHES "Intel")
      message(STATUS "Using intel compiler.")
   else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -msse -msse2 -msse3 -msse4 -mavx2 -ftree-vectorize")
      message(STATUS "Using other compiler.")
   endif()
   add_definitions(-DCOMPILEDWITHC17)
   message(STATUS "Using flag -std=c++17.")
elseif(COMPILER_SUPPORTS_CXX11)
   if(CMAKE_CXX_COMPILER_ID MATCHES "Intel")
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -mkl -vec -DMKL_ILP64 -I${MKLROOT}/include")
   else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -msse -msse2 -msse3 -msse4 -mavx2 -ftree-vectorize")
   endif()
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   if(CMAKE_CXX_COMPILER_ID MATCHES "Intel")
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -march=native -mkl -vec -DMKL_ILP64 -I${MKLROOT}/include")
   else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -msse -msse2 -msse3 -msse4 -mavx2 -ftree-vectorize")
   endif()
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

#find package
#find_package(catkin REQUIRED COMPONENTS cvo roscpp)
#find_package(cvo REQUIRED)
#find_package(PCL REQUIRED COMPONENTS common filters)
# find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(G2O REQUIRED)
#find_package(Boost REQUIRED timer chrono)
find_package(OpenCV 3.0.0 REQUIRED)
find_package(TBB REQUIRED)
# find_package(PythonLibs 3.5 EXACT)
find_package(Boost REQUIRED COMPONENTS system)

# Message(STATUS "G2O root path: ${G2O_ROOT}")
# Message(STATUS "G2O include directory: ${G2O_INCLUDE_DIR}")

#catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES cvo_slam
#)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/cvo)

add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/ORB_SLAM2)

#include directories
include_directories(
  "include"
  #${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/thirdparty/cvo/include
  ${PROJECT_SOURCE_DIR}/thirdparty/cvo/thirdparty
  ${PROJECT_SOURCE_DIR}/thirdparty/ORB_SLAM2
  ${PROJECT_SOURCE_DIR}/thirdparty/ORB_SLAM2/include

#   # modify it if it's not your local directory
#   /usr/include/python3.5
#   /usr/include/python3.5/numpy

  #${cvo_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
#   ${PCL_INCLUDE_DIRS}
  
)

#link_directories(${cvo_LIBRARY_DIRS})

#add library
add_library(cvo_slam SHARED
  src/keyframe_tracker.cpp
  src/local_map.cpp
  src/local_tracker.cpp
  src/keyframe_graph.cpp
  src/vertex_and_edge.cpp
  src/ORBmatcher.cpp
  src/ORBextractor.cpp
)

target_link_libraries(cvo_slam
 cvo 
 ORB_SLAM2
)

#add executable
add_executable(run_TUM src/run_TUM.cpp)
add_executable(run_TartanAir src/run_TartanAir.cpp)

#target link libraries
target_link_libraries(run_TUM 
cvo_slam 
${EIGEN3_LIBS} 
${G2O_CORE_LIBRARY} 
${G2O_SOLVER_DENSE} 
${G2O_SOLVER_EIGEN} 
${G2O_TYPES_SLAM3D} 
${Boost_GENERAL} 
${Boost_LIBRARIES} 
${OpenCV_LIBS} 
# ${PCL_LIBRARIES} 
tbb 
# ${PYTHON_LIBRARIES}
)

target_link_libraries(run_TartanAir 
cvo_slam 
${EIGEN3_LIBS} 
${G2O_CORE_LIBRARY} 
${G2O_SOLVER_DENSE} 
${G2O_SOLVER_EIGEN} 
${G2O_TYPES_SLAM3D} 
${Boost_GENERAL} 
${Boost_LIBRARIES} 
${OpenCV_LIBS} 
tbb 
)

set_target_properties(cvo_slam PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
