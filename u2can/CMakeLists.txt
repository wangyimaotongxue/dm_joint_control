cmake_minimum_required(VERSION 3.10)

project (dm_Linux_Drive)

add_executable(dm_Linux_Drive
    src/test_damiao.cpp
)

# set(SOURCES
#     damiao.h
#     SerialPort.h
#     test_damiao.cpp
# )

# add_executable(DM_Linux_Drive ${SOURCES})

target_include_directories(dm_Linux_Drive PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

