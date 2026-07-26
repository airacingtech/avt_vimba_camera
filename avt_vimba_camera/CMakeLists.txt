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

ament_auto_find_build_dependencies()

find_package(CUDAToolkit QUIET)
find_package(isaac_ros_managed_nitros QUIET)
find_package(isaac_ros_nitros_image_type QUIET)

if(CUDAToolkit_FOUND AND isaac_ros_managed_nitros_FOUND AND isaac_ros_nitros_image_type_FOUND)
  set(AVT_VIMBA_CAMERA_WITH_NITROS TRUE)
  message(STATUS "[avt_vimba_camera]: CUDA + NITROS found, building the GPU frame path")
else()
  set(AVT_VIMBA_CAMERA_WITH_NITROS FALSE)
  message(STATUS "[avt_vimba_camera]: no CUDA/NITROS, building the host-only frame path")
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
  list(APPEND mono_camera_sources src/gpu_frame_publisher.cpp)
endif()

ament_auto_add_library(mono_camera_node SHARED ${mono_camera_sources})

add_dependencies_and_linkings(mono_camera_node)

if(AVT_VIMBA_CAMERA_WITH_NITROS)
  target_compile_definitions(mono_camera_node PUBLIC AVT_VIMBA_CAMERA_WITH_NITROS)
  target_link_libraries(mono_camera_node CUDA::cudart CUDA::nppc CUDA::nppicc CUDA::nppidei CUDA::nppig)
  ament_target_dependencies(mono_camera_node
    isaac_ros_managed_nitros
    isaac_ros_nitros_image_type
  )
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
