CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
PROJECT(tfcc)
set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV)
#find_package(Eigen3)

set(TENSORFLOW_SOURCE_DIR "" CACHE PATH
  "Directory that contains the TensorFlow project"
)
if(NOT TENSORFLOW_SOURCE_DIR)
  get_filename_component(TENSORFLOW_SOURCE_DIR
    "~/Downloads/tensorflow"
    ABSOLUTE
  )
endif()

add_subdirectory(
  "${TENSORFLOW_SOURCE_DIR}/tensorflow/lite"
  "${CMAKE_CURRENT_BINARY_DIR}/tensorflow-lite"
  EXCLUDE_FROM_ALL
)

include_directories(
~/Downloads/tensorflow/bazel-out/k8-opt/bin
~/Downloads/tensorflow
~/Downloads/tensorflow/bazel-tensorflow/external/eigen_archive
~/Downloads/tensorflow/bazel-tensorflow/external/com_google_protobuf/src
~/Downloads/tensorflow/bazel-tensorflow/external/com_google_absl
~/Downloads/flatbuffers/include
${OpenCV_INCLUDE_DIRS}
#${EIGEN3_INCLUDE_DIR}
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
${CMAKE_DL_LIBS}
)