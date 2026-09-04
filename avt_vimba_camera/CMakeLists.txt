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

set(USE_GPU "AUTO" CACHE STRING
  "GPU path: AUTO/True builds it when this system can, REQUIRED demands it, False forces the CPU path")
set_property(CACHE USE_GPU PROPERTY STRINGS AUTO True False REQUIRED)

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

set(AVT_VIMBA_CAMERA_NITROS_DEPENDS
  isaac_ros_gxf
  isaac_ros_managed_nitros
  isaac_ros_nitros_image_type
)

list(REMOVE_ITEM ${PROJECT_NAME}_BUILD_DEPENDS ${AVT_VIMBA_CAMERA_NITROS_DEPENDS})

ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

string(TOUPPER "${USE_GPU}" AVT_USE_GPU)
if(AVT_USE_GPU STREQUAL "REQUIRED")
  set(AVT_VIMBA_CAMERA_GPU_MODE "REQUIRED")
elseif(AVT_USE_GPU STREQUAL "AUTO")
  set(AVT_VIMBA_CAMERA_GPU_MODE "AUTO")
elseif(USE_GPU)
  set(AVT_VIMBA_CAMERA_GPU_MODE "AUTO")
else()
  set(AVT_VIMBA_CAMERA_GPU_MODE "OFF")
endif()

set(AVT_VIMBA_CAMERA_WITH_NITROS FALSE)
set(AVT_VIMBA_CAMERA_MISSING "")

if(NOT AVT_VIMBA_CAMERA_GPU_MODE STREQUAL "OFF")
  set(AVT_VIMBA_CAMERA_WITH_NITROS TRUE)

  set(AVT_PREFIXES ${CMAKE_PREFIX_PATH})
  foreach(avt_path_var AMENT_PREFIX_PATH CMAKE_PREFIX_PATH)
    if(DEFINED ENV{${avt_path_var}})
      string(REPLACE ":" ";" avt_split "$ENV{${avt_path_var}}")
      list(APPEND AVT_PREFIXES ${avt_split})
    endif()
  endforeach()
  if(AVT_PREFIXES)
    list(REMOVE_DUPLICATES AVT_PREFIXES)
  endif()

  set(AVT_CUDA_MAJOR "")
  find_program(AVT_READELF NAMES readelf eu-readelf)
  mark_as_advanced(AVT_READELF)
  foreach(avt_name isaac_ros_common isaac_ros_nitros isaac_ros_nitros_image_type)
    foreach(avt_prefix IN LISTS AVT_PREFIXES)
      set(avt_lib "${avt_prefix}/lib/lib${avt_name}.so")
      if(AVT_CUDA_MAJOR OR NOT EXISTS "${avt_lib}")
        continue()
      endif()
      set(avt_needed "")
      if(AVT_READELF)
        execute_process(COMMAND "${AVT_READELF}" -d "${avt_lib}"
          OUTPUT_VARIABLE avt_needed ERROR_QUIET)
      endif()
      if(NOT avt_needed)
        file(STRINGS "${avt_lib}" avt_needed REGEX "libcudart\\.so\\.[0-9]+" LIMIT_COUNT 4)
      endif()
      if(avt_needed MATCHES "libcudart\\.so\\.([0-9]+)")
        set(AVT_CUDA_MAJOR "${CMAKE_MATCH_1}")
        message(STATUS
          "[avt_vimba_camera]: NITROS links libcudart.so.${AVT_CUDA_MAJOR} (${avt_lib})")
      endif()
    endforeach()
  endforeach()

  set(AVT_ROOTS "")
  if(ART_CUDA_TOOLKIT_ROOT)
    list(APPEND AVT_ROOTS "${ART_CUDA_TOOLKIT_ROOT}")
  endif()
  if(DEFINED ENV{CUDACXX})
    get_filename_component(avt_bin "$ENV{CUDACXX}" DIRECTORY)
    get_filename_component(avt_root "${avt_bin}" DIRECTORY)
    list(APPEND AVT_ROOTS "${avt_root}")
  endif()
  file(GLOB AVT_INSTALLED "/usr/local/cuda-*")
  list(SORT AVT_INSTALLED COMPARE NATURAL ORDER DESCENDING)
  if(AVT_CUDA_MAJOR)
    foreach(avt_root IN LISTS AVT_INSTALLED)
      if(avt_root MATCHES "/usr/local/cuda-${AVT_CUDA_MAJOR}(\\.[0-9]+)?$")
        list(APPEND AVT_ROOTS "${avt_root}")
      endif()
    endforeach()
  endif()
  list(APPEND AVT_ROOTS ${AVT_INSTALLED} "/usr/local/cuda")
  find_program(AVT_NVCC_ON_PATH nvcc)
  mark_as_advanced(AVT_NVCC_ON_PATH)
  if(AVT_NVCC_ON_PATH)
    get_filename_component(avt_bin "${AVT_NVCC_ON_PATH}" DIRECTORY)
    get_filename_component(avt_root "${avt_bin}" DIRECTORY)
    list(APPEND AVT_ROOTS "${avt_root}")
  endif()
  list(REMOVE_DUPLICATES AVT_ROOTS)

  set(AVT_TOOLKIT "")
  foreach(avt_root IN LISTS AVT_ROOTS)
    if(AVT_TOOLKIT OR NOT EXISTS "${avt_root}/bin/nvcc")
      continue()
    endif()
    execute_process(COMMAND "${avt_root}/bin/nvcc" --version
      OUTPUT_QUIET ERROR_QUIET RESULT_VARIABLE avt_rc)
    if(avt_rc EQUAL 0)
      set(AVT_TOOLKIT "${avt_root}")
    endif()
  endforeach()

  if(AVT_TOOLKIT)
    set(CUDAToolkit_ROOT "${AVT_TOOLKIT}")
    set(CUDA_TOOLKIT_ROOT_DIR "${AVT_TOOLKIT}")
    set(ENV{PATH} "${AVT_TOOLKIT}/bin:$ENV{PATH}")
  else()
    set(AVT_VIMBA_CAMERA_WITH_NITROS FALSE)
    list(APPEND AVT_VIMBA_CAMERA_MISSING "a usable CUDA toolkit")
  endif()

  if(AVT_TOOLKIT)
    foreach(avt_dep IN LISTS AVT_VIMBA_CAMERA_NITROS_DEPENDS)
      find_package(${avt_dep} QUIET)
      if(NOT ${avt_dep}_FOUND)
        set(AVT_VIMBA_CAMERA_WITH_NITROS FALSE)
        list(APPEND AVT_VIMBA_CAMERA_MISSING ${avt_dep})
      endif()
    endforeach()
  endif()

  if(AVT_VIMBA_CAMERA_WITH_NITROS)
    find_package(CUDAToolkit QUIET)
    if(NOT CUDAToolkit_FOUND)
      set(AVT_VIMBA_CAMERA_WITH_NITROS FALSE)
      list(APPEND AVT_VIMBA_CAMERA_MISSING "CUDAToolkit")
    endif()
  endif()
endif()

if(AVT_VIMBA_CAMERA_WITH_NITROS)
  # Restored for ament_auto_package()'s dependency export, which reads FOUND_BUILD_DEPENDS.
  list(APPEND ${PROJECT_NAME}_FOUND_BUILD_DEPENDS ${AVT_VIMBA_CAMERA_NITROS_DEPENDS})
  message(STATUS
    "[avt_vimba_camera]: GPU frame path -- CUDA ${CUDAToolkit_VERSION} at ${AVT_TOOLKIT}")
  if(AVT_CUDA_MAJOR AND NOT CUDAToolkit_VERSION_MAJOR STREQUAL "${AVT_CUDA_MAJOR}")
    message(WARNING
      "[avt_vimba_camera]: NITROS links libcudart.so.${AVT_CUDA_MAJOR} but this builds against "
      "CUDA ${CUDAToolkit_VERSION}, so both runtimes end up in the camera container and device "
      "pointers cross between them. Install a CUDA ${AVT_CUDA_MAJOR}.x toolkit, or rebuild the "
      "Isaac ROS stack against CUDA ${CUDAToolkit_VERSION_MAJOR}.")
  endif()
else()
  string(REPLACE ";" ", " AVT_MISSING_TEXT "${AVT_VIMBA_CAMERA_MISSING}")
  if(AVT_VIMBA_CAMERA_GPU_MODE STREQUAL "REQUIRED")
    message(FATAL_ERROR
      "[avt_vimba_camera]: USE_GPU=REQUIRED but this system cannot build the GPU path (missing "
      "${AVT_MISSING_TEXT}). Build those packages, or set USE_GPU=True in race.env to fall back "
      "to the host-only path instead of failing.")
  elseif(AVT_VIMBA_CAMERA_GPU_MODE STREQUAL "OFF")
    message(STATUS
      "[avt_vimba_camera]: host-only frame path -- forced by USE_GPU=${USE_GPU}. Frames stay on "
      "the host and are published as raw Bayer on ~/image.")
  else()
    message(STATUS
      "[avt_vimba_camera]: host-only frame path -- this system is missing ${AVT_MISSING_TEXT}. "
      "Frames stay on the host and are published as raw Bayer on ~/image; a CUDA device here does "
      "not change that, the GPU path needs GXF.")
  endif()
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
