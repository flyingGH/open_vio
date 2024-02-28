cmake_minimum_required(VERSION 3.10.2)
project(open_vio CXX)

IF (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING
    "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
ENDIF()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized -fno-omit-frame-pointer")

option(VIO_DEBUG "option for debug" ON)
if (VIO_DEBUG) 
  add_definitions(-DVIO_DEBUG)
endif(VIO_DEBUG)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

include_directories(
  ${PROJECT_SOURCE_DIR}/camera_models/include
  ${PROJECT_SOURCE_DIR}/vio/src
)

list(APPEND thirdparty_libraries
  pthread
  ${CERES_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

option(XSLAM_DEBUG "option for debug" ON)
if (XSLAM_DEBUG) 
  add_definitions(-DXSLAM_DEBUG)
endif(XSLAM_DEBUG)

add_subdirectory(camera_models)
add_subdirectory(vio)

add_executable(run_euroc test/run_euroc.cpp)
target_link_libraries(run_euroc vins_lib)