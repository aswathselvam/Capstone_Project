CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
PROJECT(tfcc)
set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV)
find_package(Eigen3)

include_directories(
~/Downloads/tensorflow/bazel-out/k8-opt/bin
~/Downloads/tensorflow
~/Downloads/tensorflow/bazel-tensorflow/external/eigen_archive
~/Downloads/tensorflow/bazel-tensorflow/external/com_google_protobuf/src
~/Downloads/tensorflow/bazel-tensorflow/external/com_google_absl
~/Downloads/flatbuffers/include
/home/surya/Downloads/tensorflow/tensorflow/lite
${OpenCV_INCLUDE_DIRS}
#${EIGEN3_INCLUDE_DIR}/home/surya/Downloads/tensorflow/tensorflow/lite
)

add_executable(tfcc tfcc.cpp)
add_executable(tflite tflite.cpp)

set(LIBS
~/Downloads/tensorflow/bazel-out/k8-opt/bin/tensorflow/libtensorflow_cc.so
#~/Downloads/tensorflow/bazel-out/k8-opt/bin/tensorflow/libtensorflow_framework.so.2.4.1
${OpenCV_LIBS}
)

target_link_libraries(tfcc
${LIBS}
)

target_link_libraries(tflite
tensorflow-lite
${LIBS}
)