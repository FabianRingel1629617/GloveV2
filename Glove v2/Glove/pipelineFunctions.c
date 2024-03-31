#include "variables.h"
#include "pipelineFunctions.h"

void initPackets(uint8_t mode, uint8_t glovieID)
{
	// Packet structure
	//**********************************************************************************************************************************************************************************************************
	// first two bytes of first packet (packet ID = 1) are synchronization bytes
	// packet 1:  2 sync bytes      2 data descriptor bytes    data
	//            0xAB 0xCD         0x.. 0x..                  0x.....................
	// packet 2:  no sync bytes     2 data descriptor bytes    data
	//                              0x.. 0x..                  0x.....................
	// packet 3:  no sync bytes     2 data descriptor bytes    data
	//                              0x.. 0x..                  0x.....................
	//**********************************************************************************************************************************************************************************************************
	
	// Data descriptor structure
	//**********************************************************************************************************************************************************************************************************
	// Node ID						4 bit					former ID1, equals this device's address. Not really required, can be resolved in base station by device address
	// (IMU ID)						3 bit [NOT USED]		former ID2. Which IMU of this device, e.g. thumb or index. Not used, not every IMU sample has an ID field anymore.
	// Device ID					3 bit					is this device a single node, glove v1, or glove v2? May be important, difference is the amount
	//														of different sensors and data alignment in the sent packets. Fixed Node ID could resolve this.
	// Mode							3 bit					quaternion or quaternion + linAcc or maybe acc + rot + mag, Debug info, or similar.
	// Sample ID					2 bit					describes Sample ID of the whole glove at certain time, packet 1, 2, 3 belong together when same Sample ID is the same
	// Packet ID					2 bit					packet 1, 2, or 3. If required it could be extended to packet 4+, but this is not so easy to implement
	
	// num bytes/samples in this packet.					Not really required -> hardcoded
	//**********************************************************************************************************************************************************************************************************
	
	
	// synchronization bytes in first packet
	payload_TX1[0] = 0xAB;
	payload_TX1[1] = 0xCD;
	
	// data descriptor at start of each packet (packet 1 starts after 2 sync bytes)
	payload_TX1[2] = glovieID << 4 | DEVICE_ID;
	payload_TX1[3] = mode << 5 | 0x01;
	
	payload_TX2[0] = glovieID << 4 | DEVICE_ID;
	payload_TX2[1] = mode << 5 | 0x02;
	
	payload_TX3[0] = glovieID << 4 | DEVICE_ID;
	payload_TX3[1] = mode << 5 | 0x03;
	
	payload_TX0[0] = 0xAB;
	payload_TX0[1] = 0xCD;
	payload_TX0[2] = 0xFF;
	payload_TX0[3] = 0xFF;
	payload_TX0[4] = glovieID << 4 | DEVICE_ID | (0 << 3);

}

void updatePacketsSampleID()
{
	// add 1 (i.e. 0x04 = 1 << 2) to previous sample ID and mask out overflow bits
	uint8_t sampleID = (payload_TX1[3] + 0x04) & 0x0C;
	
	payload_TX1[3] = (payload_TX1[3] & 0xF3) | sampleID;
	payload_TX2[1] = (payload_TX2[1] & 0xF3) | sampleID;
	payload_TX3[1] = (payload_TX3[1] & 0xF3) | sampleID;
}

uint8_t modeIsValid(uint8_t mode)
{
	// TODO: enhance (is a bit simplified for now)
	return mode < 2;
}
