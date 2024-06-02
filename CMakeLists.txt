cmake_minimum_required(VERSION 3.21)

project(n10c)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(compressed_image_transport REQUIRED)
find_package(std_srvs REQUIRED)

set(GLFW_BUILD_WAYLAND OFF)

include(FetchContent)
FetchContent_Declare(
  GUItar
  GIT_REPOSITORY https://github.com/Scriptor25/GUItar.git
  GIT_TAG dev
)
FetchContent_MakeAvailable(GUItar)

file(GLOB_RECURSE src src/**.cpp include/**.hpp)
add_executable(n10c ${src})
ament_target_dependencies(n10c PUBLIC
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  OpenCV
  cv_bridge
  image_transport
  compressed_image_transport
  std_srvs
)
target_include_directories(n10c PUBLIC "include")
target_link_libraries(n10c PUBLIC GUItar)

install(
  TARGETS n10c
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY res
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
