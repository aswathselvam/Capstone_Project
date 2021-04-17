#include<iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>
#include <memory>
#include "mqtt/async_client.h"
#include "mqtt/client.h"

int main(int argc, char* argv[])
{

    const std::string ADDRESS("tcp://broker.hivemq.com:1883");
    const std::string CLIENT_ID("mqtizer-1618668155172");

    const std::string TOPIC { "loc" };
    const std::string PAYLOAD1 { "vit" };

    const char* PAYLOAD2 = "Hi there!";

    // Create a client

    mqtt::client cli(ADDRESS, CLIENT_ID,nullptr);

    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(20);
    connOpts.set_clean_session(true);

    try {
        // Connect to the client

        cli.connect(connOpts);

        // Publish using a message pointer.

        auto msg = mqtt::make_message(TOPIC, PAYLOAD1);
        msg->set_qos(mqtt::GRANTED_QOS_0);

        cli.publish(msg);

        // Now try with itemized publish.

        cli.publish(TOPIC, PAYLOAD2, strlen(PAYLOAD2), 0, false);

        // Disconnect

        cli.disconnect();
    }
    catch (const mqtt::exception& exc) {
        std::cerr << "Error: " << exc.what() << " ["
            << exc.get_reason_code() << "]" << std::endl;
        return 1;
    }

    return 0;
}