cmake_minimum_required(VERSION 3.10)

project (dm_Linux_Drive)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)

# add_executable(dm_Linux_Drive
#     src/test_damiao.cpp
# )

add_executable(dm_motor_run src/dm_motor_run.cpp)
add_executable(dm_motor_rerun src/dm_motor_rerun.cpp)
add_executable(dm_set_zero_position src/dm_set_zero_position.cpp)
add_executable(test_damiao src/test_damiao.cpp)

# target_include_directories(dm_Linux_Drive PRIVATE
#     ${CMAKE_SOURCE_DIR}/include
# )

include_directories(${CMAKE_SOURCE_DIR}/include)