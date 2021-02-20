// NXTBot.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "NXT++.h"
#include <stdio.h>
//#pragma comment (lib, "nxt-plus-plus/nxtpp_07/lib/fantom.lib" )

int main()
{
    std::cout << "Helo World!\n";

	printf("Searching for NXT devices... (please wait) \n\n");

	// Get list of NXT Devices
	// connected via Bluetooth or USB
	//
	// Note: This will only find Bluetooth devices that have
	// been paired using the Windows BT Control Panel.
	//
	std::vector<std::vector<std::string> > devicelist;
	devicelist = Comm::ListNXTDevices(true);
	// false = only USB devices will be listed.
	// true = list Bluetooth and USB devices.

	printf("Search complete! \n\n");

	// Iterate through NXT device list
	// and output Device Name and
	// MAC Address to console.
	for (unsigned int i = 0; i < devicelist.size(); i++)
	{
		std::cout << "NXT Device " << i << "   Name: " << devicelist[i][0] << "   MAC Address: " << devicelist[i][1] << std::endl;
	}
	std::cout << "\n\n\n" << std::endl;


	
	// Un-comment this to connect to an NXT Device

	Comm::NXTComm comm1;

	// Try to connect to specific NXT Device
	//
	// The string name should be the Device Name
	// or MAC Address without colon separators.
	//
	if ( NXT::OpenNXTDevice(&comm1, devicelist[0][1], true) )
					// false = only USB devices will be listed.
					// true = list Bluetooth and USB devices.
	{
		printf( "\nConnected to brick... \n" );

		// Get NXT device information
		double protocol = NXT::GetProtocolVersion(&comm1);
		double version = NXT::GetFirmwareVersion(&comm1);
		int battery = NXT::BatteryLevel(&comm1);
		std::string name = NXT::GetName(&comm1);

		// Output information to consol e
		printf( "Protocol version: %f \n", protocol ); // e.g. 1.124000 or
		printf( "Firmware version: %f \n", version ); // e.g. 1.280000 or 1.310000
		printf( "Battery Level: %i \n", battery ); //e.g.  8198, or 6674 when battery low warning
		printf( "Name: %s \n", name.data() ); // NXT device name

		NXT::PlayTone  ( &comm1, 1000, 100 ); // Play high tone from NXT
	}

	

}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
