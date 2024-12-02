cmake_minimum_required(VERSION 3.5)
project(motor_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

FILE(GLOB MANUALCONTROLSERVER_SRC
  "./src_manualcontrolserver/*.cpp"
)


FILE(GLOB MOTORCLIENT_SRC
  "./src_motorclient/*.cpp"
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)

# Use pkg-config to find the libevdev package
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBEVDEV REQUIRED libevdev)

include_directories(
  include
  ${LIBEVDEV_INCLUDE_DIRS}
)

add_executable(manualcontrolserver ${MANUALCONTROLSERVER_SRC})
target_link_libraries(manualcontrolserver evdev)
ament_target_dependencies(manualcontrolserver rclcpp custom_interfaces)

add_executable(motorclient ${MOTORCLIENT_SRC})
ament_target_dependencies(motorclient rclcpp custom_interfaces)
# if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
# endif()

install(TARGETS
  manualcontrolserver
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  motorclient
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
