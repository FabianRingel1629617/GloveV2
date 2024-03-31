#ifndef PIPELINE_FUNCTIONS_H_
#define PIPELINE_FUNCTIONS_H_

#include "config.h"

// Addresses should have "interesting" bits with variety, not just all bits ones or zeros.
// base station's data address to communicate with single node and receive data. Address is set to (BASE DATA xx)
extern uint8_t BS_data_address[5];

// base station's broadcast address to communicate with all nodes. At the moment only used in the beginning to set data address. Address is set to (BASE CAST 3D)
extern uint8_t BS_broadcast_address[5];

extern uint8_t payload_Glove_config[PAYLOAD_MAX_LEN];  //ConfigData

extern uint8_t payload_RX[PAYLOAD_MAX_LEN];

extern uint8_t payload_TX0[PAYLOAD_MAX_LEN];           //Other Data, like CalibrationData, BatteryState (GloveV1?), ..

extern uint8_t payload_TX1[PAYLOAD_MAX_LEN];
extern uint8_t payload_TX2[PAYLOAD_MAX_LEN];
extern uint8_t payload_TX3[PAYLOAD_MAX_LEN];

/*****COMMAND_ENUMS*****/
typedef enum {
	COMMAND_TYPE_GENERAL_COMMAND,           // 0
	COMMAND_TYPE_MODE_COMMAND,              // 1
	COMMAND_TYPE_INFO_COMMAND,              // 2
	COMMAND_TYPE_UPDATE_COMMAND,            // 3
	COMMAND_TYPE_BASESTATION_MODE = 254,    // 254. //Ändern des BS-Modus
	COMMAND_TYPE_UNKNOWN                    // 255
} CommandType;

typedef enum {                                      //schauen ob die werte noch stimmen
	GENERAL_COMMAND_QUERY,                  // 0
	GENERAL_COMMAND_IDLE,                   // 1
	GENERAL_COMMAND_RESTART,                // 2
	GENERAL_COMMAND_RECOVER_GLOVE,          // 3
	GENERAL_COMMAND_CONFIGURE_GLOVE,        // 4 //TODO später noch nutzen im Broadcast
	GENERAL_COMMAND_SHOW_ID,                // 5
	GENERAL_COMMAND_UNKNOWN = 255           // 255
} GeneralCommand;                           // CommandType 0

typedef enum {
	MODE_COMMAND_QUAT,                      // 0
	MODE_COMMAND_QUAT_LIN_ACC,              // 1
	MODE_COMMAND_UNKNOWN = 255,             // 255
} ModeCommand;                              // CommandType 1

typedef enum {
	INFO_COMMAND_CALIBRATION_DATA,          // 0
	INFO_COMMAND_GLOVE_CONFIG,              // 1
	INFO_COMMAND_UNKNOWN = 255              // 255
} InfoCommand;                              // CommandType 2

typedef enum {
	UPDATE_COMMAND_SESSION_ID,              // 0
	UPDATE_COMMAND_UNKNOWN = 255            // 255
} UpdateCommand;                            // CommandType 3

typedef enum {
	BASESTATION_MODE_GLOVE_MODE,            // 0 //Get Data von Glove
	BASESTATION_MODE_BS_MODE,               // 1 //Get Data von BS
} BasestationMode;

typedef enum {
	BASESTATION_DATATYPE_NUMBER_OF_NODES,    // 0
	BASESTATION_DATATYPE_SESSION_ID,         // 1
	BASESTATION_DATATYPE_BNO,                // 2
	BASESTATION_DATATYPE_CONFIGURED_NODES,   // 3
	BASESTATION_DATATYPE_ACTIVE_NODES,       // 4
} BasestationDatatype;

typedef enum {
	BNO_DATA_QUAT,                           // 0
	BNO_DATA_QUAT_LIN_ACC,                   // 1
} BasestationBnoData;

typedef enum {
	PACKAGE_Zero,					// 0	Other Data (Glove Config, CalibrationData, ...)
	PACKAGE_One,					// 1	Data Packet 1
	PACKAGE_Two,					// 2	Data Packet 2
	PACKAGE_Three,					// 3	Data Packet 3
} Package;

typedef enum  {
	PIPE_INIT = 0,					//Pipe for Initialization
	PIPE_Broadcast = 0,				//Pipe for Broadcast
	PIPE_Data_Broadcast = 1,		//Pipe for Data	Broadcast
	PIPE_Data = 2,					//Pipe for Data
} Pipe;

#endif /* PIPELINE_FUNCTIONS_H_ */