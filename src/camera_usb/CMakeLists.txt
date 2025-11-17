cmake_minimum_required(VERSION 3.10)
project(VideoStreamExample)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找OpenCV包
find_package(OpenCV REQUIRED)

# 包含头文件目录
include_directories(${OpenCV_INCLUDE_DIRS})

# 添加可执行文件 - 确保名称一致
add_executable(video_stream src/image.cpp)

# 链接OpenCV库 - 使用正确的目标名称
target_link_libraries(video_stream ${OpenCV_LIBS})

# 设置输出目录
set_target_properties(video_stream PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# 打印配置信息
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
message(STATUS "OpenCV include dir: ${OpenCV_INCLUDE_DIRS}")