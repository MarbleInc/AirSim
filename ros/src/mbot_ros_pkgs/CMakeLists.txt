cmake_minimum_required(VERSION 3.10.0)
project(mbot_ros_pkgs)
set (CMAKE_CXX_FLAGS "-g")

# set this to path to AirSim root folder if you want your catkin workspace in a custom directory
set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../)

#add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" mbot_rpclib_wrapper)
#add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" mbot_AirLib)

set(CMAKE_CXX_STANDARD 11)
set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs
-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
-lm -lc -lgcc_s -lgcc
-lstdc++fs -fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel")

set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include")
set(RPC_LIB rpc) # name of .a file with lib prefix
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

# todo eigen3 in AirLib already
# find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  nav_msgs
  # nodelet
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  mbot_base
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES airsim_ros
  CATKIN_DEPENDS message_runtime roscpp std_msgs  # nodelet
  DEPENDS OpenMP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(mbot_sim_node
    src/nwu_transform.cpp
    src/airsim_settings_parser.cpp
    src/gps.cpp
    src/sensor.cpp
    src/lidar.cpp
    src/capture_devices.cpp
    src/airsim_client_factory.cpp
    src/mbot_sim.cpp
    src/mbot_sim_node.cpp
)

target_compile_options(mbot_sim_node PRIVATE ${OpenMP_FLAGS})
add_dependencies(mbot_sim_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(mbot_sim_node
    airsim_settings_parser
    ${catkin_LIBRARIES}
    ${OpenMP_LIBS}
    AirLib
)

# for shipping binaries. run `$ catin_make install`
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # PATTERN ".git" EXCLUDE
)
