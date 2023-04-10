cmake_minimum_required(VERSION 3.5.1)
project(ORB_SLAM2)

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

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
#find_package(PCL REQUIRED)
#find_package(G2O REQUIRED)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${PCL_INCLUDE_DIRS}
#${G2O_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED
# src/ORBextractor.cc
#src/ORBmatcher.cc
src/Converter.cc
#src/vertex_and_edge.cpp
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
DBoW2
${PCL_LIBRARIES}
#${G2O_CORE_LIBRARY}
#${G2O_TYPES_SLAM3D}
)

set_target_properties(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

