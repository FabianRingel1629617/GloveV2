#include "variables.h"
#include "config.h"

// Addresses should have "interesting" bits with variety, not just all bits ones or zeros.
// base station's data address to communicate with single node and receive data. Address is set to (BASE DATA xx)
uint8_t BS_data_address[5] = {0xBA, 0x5E, 0xDA, 0x7A, 0xFF};

// base station's broadcast address to communicate with all nodes. At the moment only used in the beginning to set data address. Address is set to (BASE CAST 3D)
uint8_t BS_broadcast_address[5] = {0xBA, 0x5E, 0xCA, 0x57, 0x3D};


/*****Buffer*****/
												//CMD-Type, CMD (Mode), GloveID, SessionId, Unique Node ID, End Bit
uint8_t payload_Glove_config[PAYLOAD_MAX_LEN] = {0x70, 0xBA, 0x5E, 0x00, UNIQUE_Glove_ID, 0xAA};	//ConfigData

uint8_t payload_RX[PAYLOAD_MAX_LEN] = {0};				//Received Data

uint8_t payload_TX0[PAYLOAD_MAX_LEN] = {0};				//Other Data, like CalibrationData, BatteryState (GloveV1?), ..

uint8_t payload_TX1[PAYLOAD_MAX_LEN] = {0};				//Paket 1
uint8_t payload_TX2[PAYLOAD_MAX_LEN] = {0};				//Paket 2
uint8_t payload_TX3[PAYLOAD_MAX_LEN] = {0};				//Paket 3
