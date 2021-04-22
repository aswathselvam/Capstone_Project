
#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>
#include <vector>
#include <functional>
#include "mqtt/client.h"

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
using namespace rapidjson;


using namespace std;
using namespace std::chrono;

const string SERVER_ADDRESS	{ "tcp://broker.hivemq.com:1883" };
const string CLIENT_ID		{ "paho_f3fwefewfw" };

constexpr int QOS_0 = 0;
constexpr int QOS_1 = 1;

/////////////////////////////////////////////////////////////////////////////

// Message table function signature
using handler_t = std::function<bool(const mqtt::message&)>;

// Handler for data messages (i.e. topic "data/#")
int data_handler(const mqtt::message& msg)
{
	cout << msg.get_topic() << ": " << msg.to_string() << endl;
	std::string jsonn = msg.to_string(); 
	const char* json = jsonn.c_str();
	rapidjson::Document d; 
	d.Parse(json);

	// 2. Modify it by DOM.
	assert(d["payload"].IsString());
	std::cout<<d["payload"].IsString();
	Value& s = d["payload"];
	std::string isverified = s.GetString();
	std::cout<<"\nMy received string: "<<isverified<<"\n";
    int x =10;
	return x;
}

/////////////////////////////////////////////////////////////////////////////

int* getLocation()
{
	mqtt::client cli(SERVER_ADDRESS, CLIENT_ID,
					 mqtt::create_options(MQTTVERSION_5));

	auto connOpts = mqtt::connect_options_builder()
		.mqtt_version(MQTTVERSION_5)
		.automatic_reconnect(seconds(2), seconds(30))
		.clean_session(false)
		.finalize();

	try {
		cout << "Connecting to the MQTT server..." << flush;
		mqtt::connect_response rsp = cli.connect(connOpts);
		cout << "OK\n" << endl;

		if (!rsp.is_session_present()) {
			std::cout << "Subscribing to topics..." << std::flush;

			mqtt::subscribe_options subOpts;
			mqtt::properties props1 {
				{ mqtt::property::SUBSCRIPTION_IDENTIFIER, 1 },
			};
			cli.subscribe("sdbAA/loc", QOS_0, subOpts, props1);

			std::cout << "OK" << std::endl;
		}
		else {
			cout << "Session already present. Skipping subscribe." << std::endl;
		}

		// Consume messages

		while (true) {
			auto msg = cli.consume_message();

			// Note: In a real app, you'd want to do a lot more error
			// and bounds checking than this.

			if (msg) {
				// Get the subscription ID from the incoming message
				int subId = mqtt::get<int>(msg->get_properties(),
										   mqtt::property::SUBSCRIPTION_IDENTIFIER);

				// Dispatch to a handler function based on the Subscription ID
				static int d = data_handler(*msg);
				std::cout<<d;
                return &d;
			}
			else if (!cli.is_connected()) {
				cout << "Lost connection" << endl;
				while (!cli.is_connected()) {
					this_thread::sleep_for(milliseconds(250));
				}
				cout << "Re-established connection" << endl;
			}
		}

		// Disconnect

		cout << "\nDisconnecting from the MQTT server..." << flush;
		cli.disconnect();
		cout << "OK" << endl;
	}
	catch (const mqtt::exception& exc) {
		cerr << exc.what() << endl;
        return nullptr;
	}
    return nullptr;
}