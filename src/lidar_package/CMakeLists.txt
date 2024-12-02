cmake_minimum_required(VERSION 3.8)
project(lidar_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(RPLIDAR_SDK_PATH "./sdk/")

FILE(GLOB RPLIDAR_SDK_SRC
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
)

FILE(GLOB RPLIDAR_NODE_SRC
  "./src/*.cpp"
)

include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  include/lidar_package
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(rplidar_sdk STATIC ${RPLIDAR_SDK_SRC})
# add_library(rplidar_node_src STATIC ${RPLIDAR_NODE_SRC})

add_executable(rplidar_node ${RPLIDAR_NODE_SRC})
target_link_libraries(rplidar_node rplidar_sdk)
ament_target_dependencies(rplidar_node rclcpp sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  rplidar_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
