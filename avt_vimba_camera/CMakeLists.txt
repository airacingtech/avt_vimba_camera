cmake_minimum_required(VERSION 3.5)
project(avt_vimba_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Option to build CUDA zero-copy camera node (opt-in)
option(BUILD_CUDA_NODE "Build zero-copy CUDA camera node" ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(avt_vimba_camera_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PCAP REQUIRED libpcap)

ament_auto_find_build_dependencies()

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
      ${OpenCV_LIBS}
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
ament_auto_add_library(mono_camera_node SHARED
  src/mono_camera_node.cpp
  src/avt_vimba_camera.cpp
  src/frame_observer.cpp
  src/pcap_reader.cpp
)

add_dependencies_and_linkings(mono_camera_node)

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

# =============================================================================
# CUDA zero-copy camera node (opt-in via BUILD_CUDA_NODE)
# =============================================================================
if(BUILD_CUDA_NODE)
  find_package(CUDA REQUIRED)

  include_directories(${CUDA_INCLUDE_DIRS})

  # CUDA debayer library
  cuda_add_library(debayer_cuda STATIC
    cuda/debayer.cu
  )
  target_include_directories(debayer_cuda PRIVATE include)
  set_target_properties(debayer_cuda PROPERTIES
    CUDA_STANDARD 14
    POSITION_INDEPENDENT_CODE ON
  )

  # CUDA camera node shared library (component)
  ament_auto_add_library(cuda_camera_node SHARED
    src/cuda_camera_node.cpp
    src/pcap_reader.cpp
  )
  target_include_directories(cuda_camera_node PRIVATE
    ${CUDA_INCLUDE_DIRS}
  )
  target_link_libraries(cuda_camera_node
    debayer_cuda
    ${CUDA_LIBRARIES}
    ${CUDA_CUDART_LIBRARY}
  )
  add_dependencies_and_linkings(cuda_camera_node)

  rclcpp_components_register_nodes(cuda_camera_node
    "avt_vimba_camera::CudaCameraNode")

  # CUDA camera executable
  ament_auto_add_executable(cuda_camera_exec
    src/exec/cuda_camera_exec.cpp
  )
  target_include_directories(cuda_camera_exec PRIVATE
    ${CUDA_INCLUDE_DIRS}
  )
  target_link_libraries(cuda_camera_exec
    cuda_camera_node
  )

  # Install the debayer CUDA library
  install(TARGETS debayer_cuda
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  calibrations
  config
  launch
)
