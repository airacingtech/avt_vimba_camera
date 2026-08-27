cmake_minimum_required(VERSION 3.5)
project(avt_vimba_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PCAP REQUIRED libpcap)

# Isaac ROS stays a build_depend so colcon still orders it ahead of this package, but it is taken
# out of the list ament_auto sees. Two reasons, and the REQUIRED keyword only covers the first:
#
#  - a bare ament_auto_find_build_dependencies() resolves every build_depend as REQUIRED, which
#    fails configure on a host without Isaac ROS -- before the host-path fallback can be selected;
#  - it also find_package()s every build_depend whether or not it is in the REQUIRED list, and
#    ament_auto_add_library() then links everything it found. That silently linked GXF into the
#    host path whenever the stack happened to be installed -- a "host-only" build that would not
#    load on a machine without it, which is precisely the machine it exists for.
#
# So the GPU packages are resolved only by the explicit probe below, and linked only by the
# ament_target_dependencies() call on that path.
ament_package_xml()
set(AVT_VIMBA_CAMERA_OPTIONAL_DEPENDS
  isaac_ros_gxf
  isaac_ros_managed_nitros
  isaac_ros_nitros_image_type
)
list(REMOVE_ITEM ${PROJECT_NAME}_BUILD_DEPENDS ${AVT_VIMBA_CAMERA_OPTIONAL_DEPENDS})

ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

# Isaac ROS alone decides whether the GPU frame path is built. A CUDA device is NOT enough and
# must not be part of the decision: the GPU path publishes NITROS handles through the GXF runtime,
# so a host with CUDA but no Isaac ROS -- or only part of it built -- has to produce the host-only
# path. Every package that path uses is therefore required, isaac_ros_gxf included, because a
# partial Isaac ROS build otherwise compiles a GPU path that cannot load at runtime.
#
# CUDA is then REQUIRED rather than probed: once this is the GPU path, a missing toolkit is a
# broken workspace to report, not a reason to silently switch paths.
#
# ART_NITROS overrides the detection, because a verdict that cannot be overridden is impossible to
# test: OFF exercises the host path on a machine that has the whole stack, and ON turns a
# silent degrade on a machine that is SUPPOSED to have it into a build failure that says what is
# missing. AUTO (the default) detects, which is what every normal build does.
#
#   colcon build --cmake-args -DART_NITROS=OFF
set(ART_NITROS "AUTO" CACHE STRING
  "NITROS/GPU path: AUTO detects Isaac ROS, ON requires it, OFF forces the host path")
set_property(CACHE ART_NITROS PROPERTY STRINGS AUTO ON OFF)

set(AVT_VIMBA_CAMERA_NITROS_DEPENDS
  isaac_ros_gxf
  isaac_ros_managed_nitros
  isaac_ros_nitros_image_type
)

# Normalised so -DART_NITROS=0/false/OFF and =1/true/ON all mean what they look like.
if(ART_NITROS STREQUAL "AUTO")
  set(AVT_VIMBA_CAMERA_NITROS_MODE "AUTO")
elseif(ART_NITROS)
  set(AVT_VIMBA_CAMERA_NITROS_MODE "ON")
else()
  set(AVT_VIMBA_CAMERA_NITROS_MODE "OFF")
endif()

set(AVT_VIMBA_CAMERA_NITROS_MISSING "")
if(AVT_VIMBA_CAMERA_NITROS_MODE STREQUAL "OFF")
  set(AVT_VIMBA_CAMERA_WITH_NITROS FALSE)
else()
  set(AVT_VIMBA_CAMERA_WITH_NITROS TRUE)
  foreach(dependency IN LISTS AVT_VIMBA_CAMERA_NITROS_DEPENDS)
    find_package(${dependency} QUIET)
    if(NOT ${dependency}_FOUND)
      set(AVT_VIMBA_CAMERA_WITH_NITROS FALSE)
      list(APPEND AVT_VIMBA_CAMERA_NITROS_MISSING ${dependency})
    endif()
  endforeach()

  if(NOT AVT_VIMBA_CAMERA_WITH_NITROS AND AVT_VIMBA_CAMERA_NITROS_MODE STREQUAL "ON")
    string(REPLACE ";" ", " AVT_VIMBA_CAMERA_NITROS_MISSING_TEXT
      "${AVT_VIMBA_CAMERA_NITROS_MISSING}")
    message(FATAL_ERROR
      "[avt_vimba_camera]: ART_NITROS=ON was requested but Isaac ROS is incomplete here (missing "
      "${AVT_VIMBA_CAMERA_NITROS_MISSING_TEXT}). Build those packages, or pass ART_NITROS=AUTO to "
      "fall back to the host path instead of failing.")
  endif()
endif()

if(AVT_VIMBA_CAMERA_WITH_NITROS)
  # Restored for ament_auto_package()'s dependency export, which reads FOUND_BUILD_DEPENDS.
  list(APPEND ${PROJECT_NAME}_FOUND_BUILD_DEPENDS ${AVT_VIMBA_CAMERA_NITROS_DEPENDS})
  find_package(CUDAToolkit REQUIRED)
  message(STATUS "[avt_vimba_camera]: Isaac ROS found, building the GPU frame path")
elseif(AVT_VIMBA_CAMERA_NITROS_MODE STREQUAL "OFF")
  message(STATUS
    "[avt_vimba_camera]: building the host-only frame path -- forced by ART_NITROS=OFF. Frames "
    "stay on the host and are published as raw Bayer on ~/image.")
else()
  string(REPLACE ";" ", " AVT_VIMBA_CAMERA_NITROS_MISSING_TEXT
    "${AVT_VIMBA_CAMERA_NITROS_MISSING}")
  message(STATUS
    "[avt_vimba_camera]: building the host-only frame path -- Isaac ROS is incomplete here "
    "(missing ${AVT_VIMBA_CAMERA_NITROS_MISSING_TEXT}). Frames stay on the host and are published "
    "as raw Bayer on ~/image; a CUDA device does not change that, the GPU path needs GXF.")
endif()

# Get architecture
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(TargetArchitecture)
target_architecture(ARCH)

###########
## Build ##
###########

include_directories(
  include
  ${PCAP_INCLUDE_DIRS}
)

function(add_dependencies_and_linkings arg)
  if("${ARCH}" STREQUAL x86_64)
    target_link_libraries(${arg}
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaC.so
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaCPP.so
      ${PCAP_LIBRARIES}
    )
  elseif("${ARCH}" STREQUAL armv7)
    target_link_libraries(${arg}
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_32bit/libVimbaC.so
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_32bit/libVimbaCPP.so
      ${PCAP_LIBRARIES}
    )
  elseif("${ARCH}" STREQUAL armv8)
    target_link_libraries(${arg}
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_64bit/libVimbaC.so
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_64bit/libVimbaCPP.so
      ${PCAP_LIBRARIES}
    )
  else()
    message(FATAL_ERROR "[libvimba]: Architecture ${ARCH} not suported. Exiting...")
  endif()
endfunction(add_dependencies_and_linkings)

# Nodes
set(mono_camera_sources
  src/mono_camera_node.cpp
  src/avt_vimba_camera.cpp
  src/frame_observer.cpp
  src/pcap_reader.cpp
)
if(AVT_VIMBA_CAMERA_WITH_NITROS)
  list(APPEND mono_camera_sources src/gpu_frame_publisher.cpp src/nvenc_h264_encoder.cpp)
endif()

ament_auto_add_library(mono_camera_node SHARED ${mono_camera_sources})

add_dependencies_and_linkings(mono_camera_node)

if(AVT_VIMBA_CAMERA_WITH_NITROS)
  target_compile_definitions(mono_camera_node PUBLIC AVT_VIMBA_CAMERA_WITH_NITROS)
  target_link_libraries(mono_camera_node CUDA::cudart CUDA::nppc CUDA::nppicc CUDA::nppidei CUDA::nppig)
  # In-driver NVENC uplink: nvEncodeAPI.h from /usr/local/include (nv-codec-headers),
  # libnvidia-encode + libcuda from the display driver.
  find_library(NVIDIA_ENCODE_LIB nvidia-encode REQUIRED)
  target_link_libraries(mono_camera_node ${NVIDIA_ENCODE_LIB} CUDA::cuda_driver)
  ament_target_dependencies(mono_camera_node ${AVT_VIMBA_CAMERA_NITROS_DEPENDS})
  set_target_properties(mono_camera_node PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    BUILD_RPATH_USE_ORIGIN TRUE
    INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()

rclcpp_components_register_nodes(mono_camera_node 
  "avt_vimba_camera::MonoCameraNode")

ament_auto_add_library(trigger_node SHARED
  src/trigger_node.cpp
)

add_dependencies_and_linkings(trigger_node)

rclcpp_components_register_nodes(trigger_node
   "trigger::TriggerNode")
   
# Executables
ament_auto_add_executable(mono_camera_exec
  src/exec/mono_camera_exec.cpp
)

target_link_libraries(mono_camera_exec
  mono_camera_node
)

ament_auto_add_executable(trigger_exec
  src/exec/trigger_exec.cpp
)

target_link_libraries(trigger_exec
  trigger_node
)

#############
## Install ##
#############

if("${ARCH}" STREQUAL x86_64)
  install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaC.so
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaCPP.so
    DESTINATION lib
  )
elseif("${ARCH}" STREQUAL armv7)
  install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_32bit/libVimbaC.so
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_32bit/libVimbaCPP.so
    DESTINATION lib
  )
elseif("${ARCH}" STREQUAL armv8)
  install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_64bit/libVimbaC.so
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_64bit/libVimbaCPP.so
    DESTINATION lib
  )
endif()

install(FILES ../README.md
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package(INSTALL_TO_SHARE
  calibrations
  config
  launch
)
