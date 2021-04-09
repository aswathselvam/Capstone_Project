'''import onnx
from keras.models import load_model

pytorch_output = '/home/surya/Capstone_Project/catkin_ws/src/codes/model'
keras_model = '/home/surya/Capstone_Project/catkin_ws/src/codes/model/saved_model.h5'
onnx.convert(keras_model,pytorch_output)


python -m tf2onnx.convert \
        --saved-model ./model \
         --output ./model/model1.onnx \
         --opset 7

model/saved_model.h5'''

import numpy as np
import tensorflow as tf
from keras.preprocessing import image
from keras.applications.resnet50 import preprocess_input
import keras2onnx
import onnxruntime


model = tf.keras.models.load_model("model/saved_model.h5")

# convert to onnx model
onnx_model = keras2onnx.convert_keras(model)