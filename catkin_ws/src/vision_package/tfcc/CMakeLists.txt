CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
PROJECT(tfcc)
set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV)
find_package(Eigen3)

include_directories(
~/Downloads/tensorflow/bazel-out/k8-opt/bin
~/Downloads/tensorflow/
#~/Downloads/tensorflow/tensorflow
#~/Downloads/tensorflow/third_party
/home/aswath/.cache/bazel/_bazel_aswath/39c920a04757f746432c7b023a8d9b9f/external/com_google_protobuf/src
/home/aswath/.cache/bazel/_bazel_aswath/39c920a04757f746432c7b023a8d9b9f/external/eigen_archive
/home/aswath/.cache/bazel/_bazel_aswath/39c920a04757f746432c7b023a8d9b9f/external/com_google_absl
${OpenCV_INCLUDE_DIRS}
#${EIGEN3_INCLUDE_DIR}
)

add_executable(tfcc tfcc.cpp)

set(LIBS
~/Downloads/tensorflow/bazel-out/k8-opt/bin/tensorflow/libtensorflow_cc.so
#~/Downloads/tensorflow/bazel-out/k8-opt/bin/tensorflow/libtensorflow_framework.so.2.4.1
${OpenCV_LIBS}
)

target_link_libraries(tfcc
${LIBS}
)