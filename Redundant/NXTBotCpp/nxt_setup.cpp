#include "nxt_setup.h"

bool setup() {
	std::cout << "Helo World!\n";

	printf("Searching for NXT devices... (please wait) \n\n");

	// Get list of NXT Devices
	// connected via Bluetooth or USB
	//
	// Note: This will only find Bluetooth devices that have
	// been paired using the Windows BT Control Panel.
	//

	//TODO: Find if bluetooth is turned on and then prceed, 
	//otherwise, the app crashes.

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
	if (NXT::OpenNXTDevice(&comm1, devicelist[0][1], true))
		// false = only USB devices will be listed.
		// true = list Bluetooth and USB devices.
	{
		printf("\nConnected to brick... \n");

		// Get NXT device information
		double protocol = NXT::GetProtocolVersion(&comm1);
		double version = NXT::GetFirmwareVersion(&comm1);
		int battery = NXT::BatteryLevel(&comm1);
		std::string name = NXT::GetName(&comm1);

		// Output information to consol e
		printf("Protocol version: %f \n", protocol); // e.g. 1.124000 or
		printf("Firmware version: %f \n", version); // e.g. 1.280000 or 1.310000
		printf("Battery Level: %i \n", battery); //e.g.  8198, or 6674 when battery low warning
		printf("Name: %s \n", name.data()); // NXT device name

		NXT::PlayTone(&comm1, 1000, 100); // Play high tone from NXT

		return true;
	}
	
	return false;
}