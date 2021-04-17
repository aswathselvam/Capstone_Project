cmake_minimum_required(VERSION 3.0.2)
project(mqtt)
#find_package(Threads REQUIRED)
find_package(PahoMqttCpp REQUIRED)

add_executable(mqtt mqtt.cpp)

target_link_libraries(mqtt PahoMqttCpp::paho-mqttpp3)