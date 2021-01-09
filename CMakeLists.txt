cmake_minimum_required(VERSION 3.1)
project(MyOrbSLAM VERSION 0.0.1)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

# Check C++11 support
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF)
cmake_policy(SET CMP0057 NEW) # enable IN_LIST operator
if(cxx_std_11 IN_LIST CMAKE_CXX_COMPILE_FEATURES)
  add_definitions(-DCOMPILEDWITHC11)
endif()

# Compiler specific options for gcc
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
  # set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-deprecated -O3 -march=native ")
  # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated -O3 -march=native")
  # reduce export symbols
  add_compile_options(-fvisibility=hidden)
endif()
# Compiler specific options for MSVC
if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3")
  # use the static CRT if other libraries are compiled with /MT
  # (CMake sets /MD by default, but for example Pangolin uses /MT by default)
  #set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
  #set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
endif()

# fix big obj file fatal error 
add_compile_options(-bigobj)

# fix stack overflow error
if (MSVC)
   set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /STACK:10000000")
else ()
   set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--stack,10000000")
endif()

MESSAGE(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 4 QUIET)
  if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
  endif()
endif()
MESSAGE(STATUS "OpenCV VERSION: ${OpenCV_VERSION}")

find_package(Eigen3 3.1.0 REQUIRED)

find_package(Pangolin REQUIRED)

find_package(Boost REQUIRED COMPONENTS serialization)
MESSAGE(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")

set(OPENSSL_USE_STATIC_LIBS TRUE)
find_package(OpenSSL REQUIRED) # for crypto library

# set config file
set(CONFIG_SLAM_VOC_FILE "${PROJECT_SOURCE_DIR}/config/ORBvoc.bin")
set(CONFIG_SLAM_SETTING_FILE "${PROJECT_SOURCE_DIR}/config/EuRoC.yaml")
set(CONFIG_USE_VIEWER true)
configure_file("${PROJECT_SOURCE_DIR}/config.h.in"
               "${PROJECT_SOURCE_DIR}/config.h")

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/CameraModels
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIR}
)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_library(${PROJECT_NAME} SHARED
  src/System.cpp
  src/Tracking.cpp
  src/LocalMapping.cpp
  src/LoopClosing.cpp
  src/ORBextractor.cpp
  src/ORBmatcher.cpp
  src/FrameDrawer.cpp
  src/Converter.cpp
  src/MapPoint.cpp
  src/KeyFrame.cpp
  src/Atlas.cpp
  src/Map.cpp
  src/MapDrawer.cpp
  src/Optimizer.cpp
  src/PnPsolver.cpp
  src/Frame.cpp
  src/KeyFrameDatabase.cpp
  src/Sim3Solver.cpp
  src/Initializer.cpp
  src/Viewer.cpp
  src/ImuTypes.cpp
  src/G2oTypes.cpp
  src/CameraModels/Pinhole.cpp
  src/CameraModels/KannalaBrandt8.cpp
  src/OptimizableTypes.cpp
  src/MLPnPsolver.cpp
  include/System.h
  include/Tracking.h
  include/LocalMapping.h
  include/LoopClosing.h
  include/ORBextractor.h
  include/ORBmatcher.h
  include/FrameDrawer.h
  include/Converter.h
  include/MapPoint.h
  include/KeyFrame.h
  include/Atlas.h
  include/Map.h
  include/MapDrawer.h
  include/Optimizer.h
  include/PnPsolver.h
  include/Frame.h
  include/KeyFrameDatabase.h
  include/Sim3Solver.h
  include/Initializer.h
  include/Viewer.h
  include/ImuTypes.h
  include/G2oTypes.h
  include/CameraModels/GeometricCamera.h
  include/CameraModels/Pinhole.h
  include/CameraModels/KannalaBrandt8.h
  include/OptimizableTypes.h
  include/MLPnPsolver.h
  include/TwoViewReconstruction.h
  src/TwoViewReconstruction.cpp
  include/vcc_zxm_mslam.h
  include/vcc_slam_interface.h
  src/vcc_zxm_mslam.cpp
  config.h
)

add_subdirectory(Thirdparty/DBoW2)

add_subdirectory(Thirdparty/g2o)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  DBoW2
  g2o
  ${Boost_LIBRARIES}
  ${OPENSSL_CRYPTO_LIBRARY}
)

# Build examples
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/examples)
#add_executable(demo demo.cpp)
#target_link_libraries(demo ${PROJECT_NAME})