cmake_minimum_required(VERSION 3.10)
project(turret)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# USB2CANFD 后端。仓库内的厂商静态库是 x86-64 版本；ARM 平台可通过
# -DDM_U2CANFD_LIBRARY=/path/to/arm/libu2canfd.a 指定同架构版本。
option(TURRET_BUILD_CANFD "Build the USB2CANFD ROS 2 nodes" ON)
set(DM_CANFD_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../DM_USB_to_canfd")
set(DM_U2CANFD_BUNDLED_LIBRARY "${DM_CANFD_SDK_DIR}/lib/libu2canfd.a")
set(DM_U2CANFD_LIBRARY "${DM_U2CANFD_BUNDLED_LIBRARY}" CACHE FILEPATH
  "Path to the Damiao USB2CANFD static library")

if(TURRET_BUILD_CANFD AND
  NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$" AND
  DM_U2CANFD_LIBRARY STREQUAL DM_U2CANFD_BUNDLED_LIBRARY)
  message(WARNING
    "The bundled libu2canfd.a is x86-64, but the target is ${CMAKE_SYSTEM_PROCESSOR}. "
    "CANFD nodes are disabled. Supply the matching library with "
    "-DDM_U2CANFD_LIBRARY=/path/to/libu2canfd.a to enable them.")
  set(TURRET_BUILD_CANFD OFF)
endif()

if(TURRET_BUILD_CANFD)
  find_package(Threads REQUIRED)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

  add_library(dm_canfd_motor STATIC
    ${DM_CANFD_SDK_DIR}/src/protocol/damiao.cpp
  )
  target_include_directories(dm_canfd_motor PUBLIC
    ${DM_CANFD_SDK_DIR}/include
    ${LIBUSB_INCLUDE_DIRS}
  )
  target_link_libraries(dm_canfd_motor
    Threads::Threads
    ${DM_U2CANFD_LIBRARY}
    ${LIBUSB_LIBRARIES}
  )

  set(DM_CANFD_TEST_NODES
    dm_canfd_motor_run
    dm_canfd_motor_rerun
    dm_canfd_set_zero_position
  )
  foreach(test_node IN LISTS DM_CANFD_TEST_NODES)
    add_executable(${test_node} src/moter_test/${test_node}.cpp)
    target_include_directories(${test_node} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${DM_CANFD_SDK_DIR}/include
    )
    target_link_libraries(${test_node}
      dm_canfd_motor
      Threads::Threads
      ${DM_U2CANFD_LIBRARY}
      ${LIBUSB_LIBRARIES}
    )
    ament_target_dependencies(${test_node} rclcpp)
  endforeach()

  add_executable(dm_canfd_dev_sn ${DM_CANFD_SDK_DIR}/src/dev_sn.cpp)
  target_include_directories(dm_canfd_dev_sn PRIVATE ${LIBUSB_INCLUDE_DIRS})
  target_link_libraries(dm_canfd_dev_sn ${LIBUSB_LIBRARIES})
endif()

if(TURRET_BUILD_CANFD)
  install(TARGETS
    ${DM_CANFD_TEST_NODES}
    dm_canfd_dev_sn
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


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

ament_package()

