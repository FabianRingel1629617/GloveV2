#include "processingFunctions.h"

#include "BNO055.h"
#include "NRF24L01p.h"

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

void process_quat_linAcc()
{
	uint8_t sensorId = 0;
	
	// packet 1    - remember: before first packet's data, there are two sync bytes, so start data at payload_TX1 + 4
	BNO_Read_Quaternion_LinAcc_Compressed(sensorId++, payload_TX1 + 4, payload_TX1 + 10);
	BNO_Read_Quaternion_LinAcc_Compressed(sensorId++, payload_TX1 + 16, payload_TX1 + 22);

	// flush RX to enable packet sending and write data (glove v1: 28 bytes, glove v2: 28 bytes)
	nrf_flushRX();
	nrf_writeAckData(PIPE_Data, payload_TX1, 28);

	
	// packet 2 (glove v1: 26 bytes, glove v2: 32 bytes)
	BNO_Read_Quaternion_LinAcc_Compressed(sensorId++, payload_TX2 + 2, payload_TX2 + 8);
	BNO_Read_Quaternion_LinAcc_Compressed(sensorId++, payload_TX2 + 14, payload_TX2 + 20);
	
	#if DEVICE_ID == GLOVE_V1
	nrf_writeAckData(PIPE_Data, payload_TX2, 26);
	
	#elif DEVICE_ID == GLOVE_V2
	// split 5th sensor data across TX2 and TX3 packets
	BNO_Read_Quaternion_LinAcc_Compressed(sensorId++, payload_TX2 + 26, payload_TX3 + 2);
	nrf_writeAckData(PIPE_Data, payload_TX2, 32);
	#endif
	
	
	// packet 3 (glove v1: 26 bytes, glove v2: 32 bytes)
	#if DEVICE_ID == GLOVE_V1
	BNO_Read_Quaternion_LinAcc_Compressed(sensorId++, payload_TX3 + 2, payload_TX3 + 8);
	BNO_Read_Quaternion_LinAcc_Compressed(sensorId++, payload_TX3 + 14, payload_TX3 + 20);
	nrf_writeAckData(PIPE_Data, payload_TX3, 26);
	
	#elif DEVICE_ID == GLOVE_V2
	BNO_Read_Quaternion_LinAcc_Compressed(sensorId++, payload_TX3 + 8, payload_TX3 + 14);
	BNO_Read_Quaternion_LinAcc_Compressed(sensorId++, payload_TX3 + 20, payload_TX3 + 26);
	nrf_writeAckData(PIPE_Data, payload_TX3, 32);
	#endif
	
}

void process_quat()
{
	//PORTC &= ~_BV(7);	//Turns OFF LED in Port C pin 7
	uint8_t sensorId = 0;
	
	// packet 1    - remember: before first packet's data, there are two sync bytes, so start data at payload_TX1 + 4
	BNO_Read_Quaternion_Compressed(sensorId++, payload_TX1 + 4);
	BNO_Read_Quaternion_Compressed(sensorId++, payload_TX1 + 10);
	BNO_Read_Quaternion_Compressed(sensorId++, payload_TX1 + 16);
	#if DEVICE_ID == GLOVE_V2
	BNO_Read_Quaternion_Compressed(sensorId++, payload_TX1 + 22);
	#endif
	
	// flush RX to enable packet sending and write data (glove v1: 22 bytes, glove v2: 28 bytes)
	nrf_flushRX();
	#if DEVICE_ID == GLOVE_V1
	nrf_writeAckData(PIPE_Data, payload_TX1, 22);
	#elif DEVICE_ID == GLOVE_V2
	nrf_writeAckData(PIPE_Data, payload_TX1, 28);
	#endif
	
	// packet 2
	BNO_Read_Quaternion_Compressed(sensorId++, payload_TX2 + 2);
	BNO_Read_Quaternion_Compressed(sensorId++, payload_TX2 + 8);
	BNO_Read_Quaternion_Compressed(sensorId++, payload_TX2 + 14);
	
	// flush RX to enable packet sending and write data (20 bytes)
	nrf_flushRX();
	nrf_writeAckData(PIPE_Data, payload_TX2, 20);
		
	//_delay_ms(200);
	//PORTC |= _BV(7);	//Turns ON LED in Port C pin 7
	//_delay_ms(200);
}

void do_nothing(){
	//maybe use a delay from a few ms
}
