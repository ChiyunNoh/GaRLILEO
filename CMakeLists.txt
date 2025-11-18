cmake_minimum_required(VERSION 3.8)
project(garlileo VERSION 0.1.0 LANGUAGES C CXX)
#######################################################################
# Build flags
#######################################################################
add_compile_options(-Wall -Wextra -Wno-unused-function -O3 -march=native)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#######################################################################
# Dependencies (ament + ROS 2 + 3rd‑party)
#######################################################################
find_package(ament_cmake          REQUIRED)
find_package(rclcpp               REQUIRED)
find_package(sensor_msgs          REQUIRED)
find_package(std_msgs             REQUIRED)
find_package(geometry_msgs        REQUIRED)
find_package(nav_msgs             REQUIRED)
find_package(pcl_ros              REQUIRED)
find_package(pcl_conversions      REQUIRED)
find_package(magic_enum           REQUIRED)
find_package(yaml-cpp             REQUIRED)
find_package(rosbag2_cpp          REQUIRED)
find_package(rosbag2_storage      REQUIRED)
find_package(spot_msgs REQUIRED)


# ─────────────────────────────────────────────────────────────────────
# PCL ↔ qhull alias (Ubuntu 22.04 libqhull-dev ships only Qhull::qhull_r)
# ─────────────────────────────────────────────────────────────────────
if(NOT TARGET QHULL::QHULL)
  find_library(QHULL_LIB qhull_r REQUIRED)
  add_library(QHULL::QHULL SHARED IMPORTED)
  set_target_properties(QHULL::QHULL PROPERTIES IMPORTED_LOCATION ${QHULL_LIB})
endif()

# Local third‑party CMake packages ────────────────────────────────────
set(THIRDPARTY_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)
set(tiny-viewer_DIR ${THIRDPARTY_ROOT}/ctraj/thirdparty/tiny-viewer-install/lib/cmake/tiny-viewer)
find_package(tiny-viewer REQUIRED)

set(ctraj_DIR ${THIRDPARTY_ROOT}/ctraj-install/lib/cmake/ctraj)
find_package(ctraj REQUIRED)
link_directories(${THIRDPARTY_ROOT}/ctraj-install/lib)
link_directories(${THIRDPARTY_ROOT}/ctraj/thirdparty/tiny-viewer-install/lib)


find_package(spdlog REQUIRED)      # apt 버전 찾기
find_package(fmt REQUIRED)

# Disable PCL pre‑compile for custom point types
add_definitions(-DPCL_NO_PRECOMPILE)

#######################################################################
# Source files
#######################################################################
file(GLOB_RECURSE SRC_CORE   CONFIGURE_DEPENDS src/core/*.cpp)
file(GLOB_RECURSE SRC_CONFIG CONFIGURE_DEPENDS src/config/*.cpp)
file(GLOB_RECURSE SRC_SENSOR CONFIGURE_DEPENDS src/sensor/*.cpp)
file(GLOB_RECURSE SRC_VIEW   CONFIGURE_DEPENDS src/view/*.cpp)
set(ALL_SRC ${SRC_CORE} ${SRC_CONFIG} ${SRC_SENSOR} ${SRC_VIEW})

#######################################################################
# Core library (isolated from rosidl auto‑targets)
#######################################################################
add_library(garlileo_core SHARED ${ALL_SRC})

set(PROJECT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_include_directories(garlileo_core PUBLIC
  $<BUILD_INTERFACE:${PROJECT_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
  /usr/local/include/magic_enum)

ament_target_dependencies(garlileo_core PUBLIC
  rclcpp sensor_msgs std_msgs geometry_msgs nav_msgs 
  pcl_ros pcl_conversions yaml-cpp rosbag2_cpp rosbag2_storage magic_enum spdlog fmt spot_msgs)

target_link_libraries(garlileo_core PUBLIC yaml-cpp ctraj)

#######################################################################
# ROS 2 Interfaces (messages)
#######################################################################
# find_package(rosidl_default_generators REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   DEPENDENCIES std_msgs sensor_msgs geometry_msgs)
# ament_export_dependencies(rosidl_default_runtime)

# # Retrieve C++ typesupport target once (keyword style)
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(garlileo_core PUBLIC ${cpp_typesupport_target})

#######################################################################
# Executables
#######################################################################
add_executable(garlileo_node_exe   exe/main.cpp)

foreach(_exe garlileo_node_exe)
  # 1) first link package dependencies (uses *plain* signature internally)
  ament_target_dependencies(${_exe}
    rclcpp sensor_msgs std_msgs geometry_msgs nav_msgs 
    rosbag2_cpp rosbag2_storage spdlog fmt spot_msgs)
  # 2) now use the *plain* signature consistently
  target_link_libraries(${_exe} garlileo_core ${cpp_typesupport_target} fmt::fmt ctraj)
endforeach()

#######################################################################
# Installation
##################### ##################################################
install(
  DIRECTORY ${THIRDPARTY_ROOT}/ctraj-install/lib/
            ${THIRDPARTY_ROOT}/ctraj/thirdparty/tiny-viewer-install/lib/
  DESTINATION lib
)
install(TARGETS
  garlileo_core garlileo_node_exe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
        FILES_MATCHING PATTERN "*.launch.py" PATTERN "*.launch.xml")

# ── 2. 기타 정적파일(config, param) ----------------------------------
set(DATA_DIRS config)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/param)
  list(APPEND DATA_DIRS param)
endif()

set(DATA_DIRS config dataset)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/param)
  list(APPEND DATA_DIRS param)
endif()

install(DIRECTORY ${DATA_DIRS}
        DESTINATION share/${PROJECT_NAME})
#######################################################################
foreach(tgt garlileo_node_exe)
  set_target_properties(${tgt} PROPERTIES
    INSTALL_RPATH "\$ORIGIN/../:\$ORIGIN/../../ctraj-install/lib")
endforeach()

ament_package()
