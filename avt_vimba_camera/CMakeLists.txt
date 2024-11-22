cmake_minimum_required(VERSION 3.5)
project(avt_vimba_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camera_info_manager REQUIRED)

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
)

function(add_dependencies_and_linkings arg)
  if("${ARCH}" STREQUAL x86_64)
    target_link_libraries(${arg}
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaC.so
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaCPP.so
      ${OpenCV_LIBS}
    )
  elseif("${ARCH}" STREQUAL armv7)
    target_link_libraries(${arg}
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_32bit/libVimbaC.so
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_32bit/libVimbaCPP.so
    )
  elseif("${ARCH}" STREQUAL armv8)
    target_link_libraries(${arg}
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_64bit/libVimbaC.so
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_64bit/libVimbaCPP.so
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
)
add_dependencies_and_linkings(mono_camera_node)

ament_auto_add_library(image_subscriber_node SHARED
  src/image_subscriber_node.cpp
)

ament_auto_add_library(trigger_node SHARED
  src/trigger_node.cpp
)
add_dependencies_and_linkings(trigger_node)

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

# Add the intra-process executable
ament_auto_add_executable(intra_process_exec
  src/exec/intra_process_exec.cpp
)
target_link_libraries(intra_process_exec
  mono_camera_node
  image_subscriber_node
)

# Add the image subscriber executable
ament_auto_add_executable(image_subscriber_exec
  src/exec/image_subscriber_exec.cpp
)
target_link_libraries(image_subscriber_exec
  image_subscriber_node
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
