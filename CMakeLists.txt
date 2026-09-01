cmake_minimum_required(VERSION 3.8)
project(fast_lio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")

# --- perf experiment knob -----------------------------------------------------
# The ONLY OpenMP parallel region in this codebase is the residual/nearest-
# neighbour loop in h_share_model() (src/laserMapping.cpp), which is the
# dominant per-scan cost and runs max_iteration times per scan. The
# architecture test below enables it for x86 only, so on aarch64 -- a Jetson --
# MP_EN stays undefined and that loop runs on ONE core.
#
# The loop writes only to index-i slots (feats_down_world[i], Nearest_Points[i],
# point_selected_surf[i] (uint8_t, not vector<bool>), normvec[i], res_last[i]);
# total_residual is accumulated in a separate sequential loop afterwards. So
# parallelising it is race-free and deterministic.
#
# Default OFF so the stock behaviour is unchanged and the A/B is explicit:
#   colcon build --cmake-args -DFASTLIO_ENABLE_OPENMP_MP=ON -DFASTLIO_MP_PROC_NUM=4
# See perf/README.md, experiment E5.
option(FASTLIO_ENABLE_OPENMP_MP
       "Enable the OpenMP residual loop regardless of CPU architecture" OFF)
set(FASTLIO_MP_PROC_NUM "4" CACHE STRING
    "Threads for the OpenMP residual loop when FASTLIO_ENABLE_OPENMP_MP=ON")

if(FASTLIO_ENABLE_OPENMP_MP)
  add_definitions(-DMP_EN)
  add_definitions(-DMP_PROC_NUM=${FASTLIO_MP_PROC_NUM})
  message(STATUS "perf: OpenMP residual loop FORCED ON with ${FASTLIO_MP_PROC_NUM} threads")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")

  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

# ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  pcl_ros
  pcl_conversions
  livox_ros_driver2
)

# Thirdparty libraries
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

message(Eigen: ${EIGEN3_INCLUDE_DIR})
message(STATUS "PCL: ${PCL_INCLUDE_DIRS}")

set(msg_files
  "msg/Pose6D.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(fastlio_mapping src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
target_include_directories(fastlio_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
# Header-only, opt-in performance/crash probe (perf/instrumentation/perf_probe.hpp).
# It compiles to no-ops unless FASTLIO_PERF_LOG is set at runtime, so it is safe
# to leave enabled in the build. See perf/README.md.
target_include_directories(fastlio_mapping PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/perf/instrumentation
)
target_link_libraries(fastlio_mapping ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} Eigen3::Eigen)
target_include_directories(fastlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})

list(APPEND EOL_LIST "foxy" "galactic" "eloquent" "dashing" "crystal")

if($ENV{ROS_DISTRO} IN_LIST EOL_LIST)
  # Custommsg to support foxy & galactic
  rosidl_target_interfaces(fastlio_mapping
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(fastlio_mapping ${cpp_typesupport_target})
endif()

ament_target_dependencies(fastlio_mapping ${dependencies})

# ---------------- Install --------------- #
install(TARGETS fastlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
