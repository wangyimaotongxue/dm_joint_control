cmake_minimum_required(VERSION 3.16)
project(dm_main)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)


# 在 CMakeLists.txt 的任何位置
message("CMAKE_CURRENT_SOURCE_DIR = ${CMAKE_CURRENT_SOURCE_DIR}")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                        ${CMAKE_CURRENT_SOURCE_DIR}/lib
                        ${LIBUSB_INCLUDE_DIRS})
                        

# 公共电机控制库：damiao.cpp + usb_class.cpp 只编译一次，供多个可执行文件复用
add_library(damiao STATIC
    src/protocol/damiao.cpp
    src/protocol/usb_class.cpp
)

target_link_libraries(damiao PUBLIC
                        ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/libdm_device.so
                        ${LIBUSB_LIBRARIES})


add_executable(${PROJECT_NAME}
    src/main.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE damiao)


add_executable(dm_set_zero_position src/dm_set_zero_position.cpp)

target_link_libraries(dm_set_zero_position PRIVATE damiao)


add_executable(dev_sn
    src/dev_sn.cpp
)

# 链接库到可执行文件
target_link_libraries(dev_sn PRIVATE
    ${LIBUSB_LIBRARIES}
)


# 原始报文嗅探器：打印总线上的每一帧，用于排查电机是否回包、回包的 ID/通道
add_executable(can_sniff
    src/can_sniff.cpp
)

target_link_libraries(can_sniff PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/libdm_device.so
    ${LIBUSB_LIBRARIES}
)
