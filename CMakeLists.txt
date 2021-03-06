cmake_minimum_required(VERSION 3.10)

# set project
project(MotionProcessing VERSION 0.1.0)
set(CMAKE_CXX_COMPLIER "g++")
set(CMAKE_CXX_STANDARD 17)

# set output directory
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# aOpenGL
set(aOpenGL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../aOpenGL)
find_package(aOpenGL REQUIRED)

# aLibTorch
set(aLibTorch_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../aLibTorch)
find_package(aLibTorch REQUIRED)

# add code
add_executable(template
    ${CMAKE_CURRENT_SOURCE_DIR}/code/train_template.cpp
)

add_executable(Stitching 
    ${CMAKE_CURRENT_SOURCE_DIR}/code/stitching.cpp
)

add_executable(RootRelative 
    ${CMAKE_CURRENT_SOURCE_DIR}/code/root_relative.cpp
)

add_executable(Test 
    ${CMAKE_CURRENT_SOURCE_DIR}/code/test.cpp
)

target_link_libraries(template aOpenGL aLibTorch)
target_link_libraries(Stitching aOpenGL)
target_link_libraries(RootRelative aOpenGL)
target_link_libraries(Test aOpenGL)