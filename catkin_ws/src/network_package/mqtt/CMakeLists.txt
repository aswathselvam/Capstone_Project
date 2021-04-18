cmake_minimum_required(VERSION 3.0.2)
project(mqtt)
#find_package(Threads REQUIRED)
find_package(PahoMqttCpp REQUIRED)

add_executable(mqtt mqtt.cpp)
add_executable(mqtt_sub mqtt_subscribe.cpp)

target_link_libraries(mqtt PahoMqttCpp::paho-mqttpp3)
target_link_libraries(mqtt_sub PahoMqttCpp::paho-mqttpp3)