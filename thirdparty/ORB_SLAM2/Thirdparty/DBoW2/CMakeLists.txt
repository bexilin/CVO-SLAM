cmake_minimum_required(VERSION 3.5.1)
project(DBoW2)

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

set(HDRS_DBOW2
  DBoW2/BowVector.h
  DBoW2/FORB.h 
  DBoW2/FClass.h       
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h   
  DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp      
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})

set_target_properties(DBoW2 PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

