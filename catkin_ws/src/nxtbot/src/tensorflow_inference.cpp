#include "tensorflow_inference.h"

SSInference::SSInference(){
  image = "../assets/myvideo_Label_100.jpg";
  graph = "frozen_graph.pb";
  labels = "imagenet_slim_labels.txt";
  username =  getenv("USER");
  root_dir = "/home/"+username+"/Capstone_Project/catkin_ws/src/vision_package/script/";
  input_width = 128;
  input_height = 128;
  input_mean = 0;
  input_std = 10;
  input_layer = "x:0";
  output_layer = "Identity:0";
  self_test = false;

  resized_tensor = tensorflow::Tensor(tensorflow::DT_FLOAT, tensorflow::TensorShape({1,input_height,input_width,3}) );
  LoadGraph();
}

// Reads a model graph definition from disk, and creates a session object you
// can use to run it.
void SSInference::LoadGraph() {
  tensorflow::GraphDef graph_def;
  
   // First we load and initialize the model.
  string graph_file_name = tensorflow::io::JoinPath(this->root_dir, this->graph);

  Status load_graph_status =
      ReadBinaryProto(tensorflow::Env::Default(), graph_file_name, &graph_def);
  if (!load_graph_status.ok()) {
    //return tensorflow::errors::NotFound("Failed to load compute graph at '", graph_file_name, "'");
  }

  auto options = tensorflow::SessionOptions();
  options.config.mutable_gpu_options()->set_per_process_gpu_memory_fraction(0.2);
  options.config.mutable_gpu_options()->set_allow_growth(true);
  session.reset(tensorflow::NewSession(options));
  Status session_create_status = session->Create(graph_def);
  if (!session_create_status.ok()) {
    LOG(ERROR) << load_graph_status;
    //return session_create_status;
  }
  std::cout<<"Done Loading graph"<<std::endl;
  //return Status::OK();
}

cv::Mat SSInference::getMask(cv::Mat img) {
  // These are the command-line flags the program can understand.
  // They define where the graph and input data is located, and what kind of
  // input the model expects. If you train your own model, or use something
  // other than inception_v3, then you'll need to update these.

    if(img.empty())
      std::cout<<"No frame";
    
    cv::resize(img, img, cv::Size(128,128));
    cv::imshow("img", img);
    // allocate a Tensor
    // get pointer to memory for that Tensor
    float *p = resized_tensor.flat<float>().data();
    int arrsize = input_height * input_height * 3;
    cv::Mat temp;

    img.convertTo(temp, CV_32FC3);
    float* pp = temp.ptr<float>();
    std::copy(pp, pp + arrsize, p );
    
    //std::cout<<"REsized tensor: "<<resized_tensor.shape() <<std::endl;
    auto input_tensor=  resized_tensor.tensor<float, 4>();
    /*
    for(int r=100; r<102;r++){
      for(int c=45; c<50; c++){
          //float f = finalOutputTensor(0, b, i, 0);
          //input_tensor(0, r, c, 0) = 4;
          //mask.at<float>(r, c) = f;

          std::cout << r << "th row and "<<c<<" th col is "<< input_tensor(0, r, c, 1) <<std::endl; 
      }
      }
    */

    // Actually run the image through the model.    
    //clock_t t1 = clock();
    
    start_inference = std::chrono::high_resolution_clock::now();
    run_status = session->Run({{input_layer, resized_tensor}},{output_layer}, {}, &outputs);
    stop_inference = std::chrono::high_resolution_clock::now();
    time_span = std::chrono::duration_cast<std::chrono::microseconds>(stop_inference - start_inference);
    std::cout<<"Time take(us): "<< time_span.count()<<std::endl;
    //auto time  = stop_inference - start_inference;
    //std::cout<<"\nTime taken(ms): "<<time/std::chrono::milliseconds(1)<<std::endl; 

    if (!run_status.ok()) {
      LOG(ERROR) << "Running model failed: " << run_status;
    }else{
      //std::cout<<"Output size: "<< outputs[0].shape() << " "<< outputs.size() << " " <<typeid(outputs[0]).name();
      
      auto finalOutputTensor  = outputs[0].tensor<float, 4>();
      cv::Mat mask = cv::Mat::zeros(128, 128, CV_32FC1);
      float min = 9999;
      for(int r=0; r<128;r++){
      for(int c=0; c<128; c++){
          //float f = finalOutputTensor(0, b, i, 0);
          float f =  outputs[0].tensor<float_t, 4>()(0, r, c, 1);
          mask.at<float>(r, c) = f;
          if(f<min && f>0){
            min=f;
          }
          //std::cout << r << "th output for class "<<c<<" is "<< f <<std::endl; 
      }
      }

      mask = mask*100;
      cv::Mat mask8;
      mask.convertTo(mask8, CV_8UC1);
      return mask8;
    }
    return img;
  }
