#include "QueryGloveInfo.h"

#include "BNO055.h"
#include "NRF24L01p.h"

#include <avr/io.h>
#include <util/delay.h>

void query_calibration_data() {
	uint8_t sensorId = 0;
	//payload_TX0[4] = 0xFF;	//NodeID 4 Bit + 1 + xxx?
	payload_TX0[5] = COMMAND_TYPE_INFO_COMMAND;
	payload_TX0[6] = INFO_COMMAND_CALIBRATION_DATA;
	
	//PORTC |= _BV(6);	//Turns ON LED in Port C pin 6
	
	BNO_Read_Calibration_Data(sensorId++, payload_TX0+7);	//BNO0
	BNO_Read_Calibration_Data(sensorId++, payload_TX0+8);	//BNO1
	BNO_Read_Calibration_Data(sensorId++, payload_TX0+9);	//BNO2
	BNO_Read_Calibration_Data(sensorId++, payload_TX0+10);	//BNO3
	BNO_Read_Calibration_Data(sensorId++, payload_TX0+11);	//BN04
	
	#if DEVICE_ID == GLOVE_V1
	BNO_Read_Calibration_Data(sensorId, payload_TX0+12);	//BNO5
	
	nrf_flushRX();
	nrf_writeAckData(PIPE_Data, payload_TX0, 13);
	#elif DEVICE_ID == GLOVE_V2
	BNO_Read_Calibration_Data(sensorId++, payload_TX0+12);	//BNO5
	BNO_Read_Calibration_Data(sensorId, payload_TX0+13);	//BNO6
	
	nrf_flushRX();
	nrf_writeAckData(PIPE_Data, payload_TX0, 14);
	#endif

	//_delay_ms(100);
	//PORTC &= ~_BV(6);	//Turns OFF LED in Port C pin 6
	
}

void query_glove_config() {
	//payload_TX0[4] = 0xFF;	//NodeID 4 Bit + 1 + xxx?
	payload_TX0[5] = COMMAND_TYPE_INFO_COMMAND;
	payload_TX0[6] = INFO_COMMAND_GLOVE_CONFIG;
	for (int i = 0; i < 6; i++) {
		payload_TX0[i+7] = payload_Glove_config[i];
	}
	nrf_writeAckData(PIPE_Data, payload_TX0, 13);
}
