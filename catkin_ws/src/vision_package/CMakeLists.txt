cmake_minimum_required(VERSION 3.0.2)
project(codes)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
list(APPEND CMAKE_PREFIX_PATH "/home/surya/Downloads/libtorch-cxx11-abi-shared-with-deps-1.8.1+cu111/libtorch")
find_package(OpenCV)
find_package(Torch REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(chessBoard src/chessBoard.cpp)
add_executable(example-app src/example-app.cpp)
add_executable(pytorchbas src/pytorchbas.cpp)
add_executable(simplenn src/simplenn.cpp)

target_link_libraries(chessBoard
   ${OpenCV_LIBS}
 )

target_link_libraries (example-app 
   ${TORCH_LIBRARIES}
)

target_link_libraries (pytorchbas
    ${TORCH_LIBRARIES}
)
target_link_libraries (simplenn
    ${TORCH_LIBRARIES}
)

set_property(TARGET example-app PROPERTY CXX_STANDARD 14)
