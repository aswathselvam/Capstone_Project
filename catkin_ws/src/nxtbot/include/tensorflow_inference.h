#include <fstream>
#include <iostream>
#include <utility>
#include <vector>
#include <opencv2/opencv.hpp>
#include <typeinfo>


#include "tensorflow/cc/ops/const_op.h"
#include "tensorflow/cc/ops/image_ops.h"
#include "tensorflow/cc/ops/standard_ops.h"
#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/framework/tensor.h"
#include "tensorflow/core/graph/default_device.h"
#include "tensorflow/core/graph/graph_def_builder.h"
#include "tensorflow/core/lib/core/errors.h"
#include "tensorflow/core/lib/core/stringpiece.h"
#include "tensorflow/core/lib/core/threadpool.h"
#include "tensorflow/core/lib/io/path.h"
#include "tensorflow/core/lib/strings/str_util.h"
#include "tensorflow/core/lib/strings/stringprintf.h"
#include "tensorflow/core/platform/env.h"
#include "tensorflow/core/platform/init_main.h"
#include "tensorflow/core/platform/logging.h"
#include "tensorflow/core/platform/types.h"
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/util/command_line_flags.h"

// These are all common classes it's handy to reference with no namespace.
using tensorflow::Flag;
using tensorflow::int32;
using tensorflow::Status;
using tensorflow::string;
using tensorflow::Tensor;
using tensorflow::tstring;



class SSInference{

  string image;
  string graph;
  string labels;
  int32 input_width;
  int32 input_height;
  float input_mean;
  float input_std;
  string input_layer;
  string output_layer;
  bool self_test;
  std::string username;
  string root_dir;

  std::vector<Tensor> outputs;
  std::chrono::high_resolution_clock::time_point start_inference;
  std::chrono::high_resolution_clock::time_point stop_inference ;
  std::chrono::microseconds time_span;
  tensorflow::Tensor resized_tensor;
  Status run_status;
  std::unique_ptr<tensorflow::Session> session;


public:
  SSInference();
  cv::Mat getMask(cv::Mat img);
  void LoadGraph();
};

