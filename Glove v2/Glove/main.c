/*
 * EG_Final.c
 *
 * Created: 15-Mar-19 1:18:24 AM
 * Author : Jochen Kempfle
 */ 

#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <avr/power.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <avr/wdt.h>

#include "config.h"

#include <util/setbaud.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "nrf.h"
#include "NRF24L01p.h"
#include "SPI.h"
#include "BNO055.h"
#include "i2cmaster.h"
#include "variables.h"
#include "pipelineFunctions.h"
#include "processingFunctions.h"
#include "QueryGloveInfo.h"

/*******************************************************************************/
/*******************************************************************************/

/*N�tig, da der Watchdog nicht ausgeschaltet wird sondern mit der kleinsten Watchdog Zeit 15ms aktiv ist und es an anderer Stelle zu sp�t w�re und der jedes Mal Neustarten w�rde, siehe ...*/
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
void get_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));
void get_mcusr(void)
{
	mcusr_mirror = MCUSR;
	MCUSR = 0;
	wdt_disable();
}

/*******************************************************************************/
/*******************************************************************************/

//Function Prototypes
void AVR_Init(void);
void UART_Init(void);
void UART_Tx(unsigned char data);
void UART_Put_String(char *s);


/************************************************************************************
** AVR_Init function:
** - Resets the Clock Prescaler factor to 1x
** - Start-up delay
** - Initializes the I/O peripherals
** - Plays LED sequence
*************************************************************************************/
void AVR_Init(void)
{
	//Set the Clock Prescaler division factor to 1(F_CPU = 8MHz)
	clock_prescale_set (clock_div_1);
	
	DDRD |= _BV(1);		//Set TX as output
	DDRD &= ~(_BV(0));	//Set RX as input

	//Make LED pins as output
	DDRC |= _BV(6);			//Makes PORTC, bit 6 as Output
	DDRC |= _BV(7);			//Makes PORTC, bit 7 as Output

	//Make MUX Select pins as output
	DDRD |= _BV(MUX_S0);		//Makes PORTD, bit 4 as Output
	DDRD |= _BV(MUX_S1);		//Makes PORTD, bit 6 as Output
	DDRD |= _BV(MUX_S2);		//Makes PORTD, bit 7 as Output

	//Start-up LED sequence loop -> implements short pause after BNO055 Power-On Reset (mandatory)
	for (int i = 6; i != 0; i--)
	{
		PORTC &= ~(_BV(6));	//Turns OFF LED in Port C pin 6
		PORTC |= _BV(7);	//Turns ON LED in Port C pin 7
		_delay_ms(100);		//0.1 second delay

		PORTC |= _BV(6);	//Turns ON LED in Port C pin 6
		PORTC &= ~(_BV(7));	//Turns OFF LED in Port C pin 7
		_delay_ms(100);		//0.1 second delay
	}

	PORTC &= ~(_BV(6));		//Turns OFF LED in Port C pin 6
	PORTC &= ~(_BV(7));		//Turns OFF LED in Port C pin 7

	_delay_ms(100);		// again, short pause to ensure mandatory BNO055 Power-On Reset time
}

/************************************************************************************
** USART Reference:
** - ATmega32U4 Datasheet - Rev. CORP072610(Pg.186)
** - AVR Microcontroller and Embedded Systems - Mazidi(Pg.395)
** - Embedded C Programming and the Atmel AVR - Barnett(Pg.132)
*************************************************************************************
** To initialize the UART, the following steps are to be followed:
** - Set the Baud rate(use <util/setbaud.h>, which depends on the macros F_CPU & BAUD)
** - Disable double speed(2x) mode
** - Set the no. of data bits(8/9 bits), stop bit(1/2) and parity bit(None/Odd/Even)
** - Set the USART mode(Synchronous/Asynchronous/Asynchronous 2x)
** - Enable Receiver & Transmitter(Set RXEN & TXEN bits in UCSRB register)
*************************************************************************************/
void UART_Init(void)
{
	//Set the BAUD rate(Ref. ATmega32U4 Datasheet Pg.189, Table 18-1)
	//To hard-code the Baud rate, Ref. Tables 18-9 to 18-12 in Pages 210 - 213
	UBRR1 = ((F_CPU / (16UL * BAUD)) - 1);

	//Disables 2x speed
	UCSR1A &= ~(_BV(U2X1));

	//Enable 8-bit character size, one stop-bit, no parity & asynchronous mode
	UCSR1C |= _BV(UCSZ11) | _BV(UCSZ10);

	//Enable Transmitter & Receiver
	UCSR1B |= _BV(TXEN1) | _BV(RXEN1);
}

/************************************************************************************
** UART_Tx function:
** - Transmits the TWI data via the USB Serial
** - The data is received & displayed in a Hyperterminal
*************************************************************************************/
void UART_Tx(unsigned char data)
{
	loop_until_bit_is_set(UCSR1A, UDRE1);		//Wait until buffer is empty
	UDR1 = data;					//Send TWI data via UART
}

void UART_Put_String(char *s)
{
	//Loop through entire string
	while(*s)
	{
	    UART_Tx(*s);
	    s++;
	}
}



void INT6_Init(void)
{
	EICRB &= ~(1 << ISC60) | (1 << ISC61);	//INT6 active when low
	EIMSK |= (1 << INT6);			//Enable INT6
	sei();					//Enable global interrupts
}

ISR(INT6_vect)
{
	cli();					//Disable global interrupt

	nrf_stopListening();

	uint8_t len, pipe;
	nrf_readRXData(payload_RX, &len, &pipe);

	// RX_Payload_cnt = len;
	// received++;
	
	// add some short delay, otherwise strangely the receiver can not receive data
	_delay_us(100);
	
	// Reset status register
	SPI_Write_Byte(STATUS, (1 << RX_DR));
}


/************************************************************************************
** Main function:
** - Contains an endless loop
** - Sets the BNO055 in NDOF mode and fetches the quaternion data
*************************************************************************************/
int main(void)
{
	// Disable global interrupt
	cli();
	
	//void (*processingFunctions[3])() = {process_quat, process_quat_linAcc, do_nothing};
	//void (*queryFunctions[2])() = {query_calibration_data, query_glove_config};
	
	// initialization
	AVR_Init();
	i2c_init();
	UART_Init();
	SPI_Init();
	nrf_init(0x69, DR_1M, NRF_ADDR_LEN, 1);
	
	// could also just open a dynamic RX pipe, but this way we also have the TX address set and RX pipe will be opened anyway
	//nrf_openDynamicTXPipe(BS_broadcast_address, 1, 0, 0);
	nrf_openDynamicTXPipe(BS_broadcast_address, 1, 0, PIPE_INIT);
	nrf_disableRXAddress(PIPE_Data);			// disable Pipe because of the soft reset. After the soft reset the Address registers will not be reset
	nrf_disableRXAddress(PIPE_Data_Broadcast);	//
	
	//INT6_Init();
	
	// finally initialize BNO (needs power-on reset time + takes a lot of setup time)
	BNO_Init();
	
	
	
	// nrf setup: Configure as receiver
	nrf_setModeRX();
	nrf_maskIRQ(1, 1, 1);
	
	
	
	uint8_t rxLen = 0;
	uint8_t rxPipe;
	uint8_t rx, tx_done, max_retry;
	
	
	uint8_t gloveID = 0;
	uint8_t inRecoveryMode = 0;		//Flag that the node is in the recoverMode
	//uint8_t sessionId = 0;
	
	// operation mode
	// default: quaternion only, mode = 1 -> quaternion + lin. acceleration
	uint8_t *commandType = payload_Glove_config+0;			//0x01;		//BNO-Data
	uint8_t *command = payload_Glove_config+1;				//0x00;		//Quat-Data
	initPackets(*command, gloveID);
	
	// timer
	TCCR1B |= _BV(CS11);
	
	// reset timer
	TCNT1 = 0;
	
	PORTC |= _BV(6);	//Turns ON LED in Port C pin 6
	PORTC |= _BV(7);	//Turns ON LED in Port C pin 7
	_delay_ms(200);
	
	uint8_t doReceive = 1;
	
	nrf_flushAll();
	nrf_resetIRQFlags();
	/*
	payload_glove_config[0] = 0x70;				//CommandType
	payload_Glove_config[1] = 0xBA;				//CMD (Mode)
	payload_Glove_config[2] = 0x5E;				//GloveID	//Data ID
	payload_Glove_config[3] = 0x00;				//SessionID
	payload_Glove_config[4] = UNIQUE_Glove_ID;	//Unique Node ID //In einem Array nutzen und an entsprechende ID schreiben. Beim "broadcast" ebenfalls nutzen und pr�fen ob eine doppelt ist. Bei wiederherstellung und recording nutzen. 
	payload_Glove_config[5] = 0xAA;				//End Byte
	*/
	nrf_writeAckData(PIPE_INIT, payload_Glove_config+3, 3); //Write SessionID UniqueID and End-Byte in TX-Pipe
	
	nrf_startListening();
	nrf_setRetries(500, 3);
	
	_delay_us(150);

	// receive this sensor node's address (which equals its ID)
	while (1)
	{
		rxLen = 0;
		if (nrf_getIRQStatus(&rx, &tx_done, &max_retry))
		{
			nrf_resetIRQFlags();
			if (rx && doReceive)
			{

				while (nrf_dataAvailable())
				{
					nrf_readRXData(payload_RX, &rxLen, &rxPipe);
				}
				
				// check payload and change mode if required
				payload_Glove_config[0] = payload_RX[0]; //CmdType
				payload_Glove_config[1] = payload_RX[1]; //CMD (Mode)
				payload_Glove_config[2] = payload_RX[2]; //GloveID
				payload_Glove_config[3] = payload_RX[3]; //SessionID
				
				//sessionId = payload_RX[2];
				
				doReceive = 0;
				
				TCNT1 = 0;
				
			}
			if (tx_done)
			{
				// last ack packet was received by PTX
				nrf_stopListening();
				break;
			}
			if (TCNT1 > 1000 && !doReceive) {					// If not tx_done, but the tx-fifo is empty, could happen if data send but no ack is received
				doReceive = 1;
				nrf_writeAckData(PIPE_INIT, payload_Glove_config+3, 3);
			}
		}
	}
	
	if (payload_Glove_config[1] != *command && modeIsValid(payload_Glove_config[1]))
	{
		*command = payload_Glove_config[1];
		initPackets(*command, payload_Glove_config[2]);
	} else {
		initPackets(*command, payload_Glove_config[2]);
	}
	gloveID = payload_Glove_config[2];
	
	PORTC &= ~_BV(7);	//Turns OFF LED in Port C pin 7
	_delay_ms(500);
	
	for (int i = 0; i < gloveID + 1; ++i)
	{
		PORTC |= _BV(7);	//Turns ON LED in Port C pin 7
		_delay_ms(200);
		PORTC &= ~_BV(7);	//Turns OFF LED in Port C pin 7
		_delay_ms(200);
	}
	
	PORTC &= ~_BV(6);	//Turns OFF LED in Port C pin 6
	
	
	
	nrf_openDynamicRXPipe(PIPE_Data_Broadcast, BS_data_address, 1, 0);	//Open the nrf-data-broadcast-pipe
	BS_data_address[4] = gloveID;
	nrf_openDynamicRXPipe(PIPE_Data, BS_data_address, 1, 0);			//Open the nrf-data-pip
	//nrf_openDynamicTXPipe(BS_broadcast_address, 1, 0, PIPE_Broadcast); //funktioniert, da aber die BS_data_address hauptsächlich genutzt wird, ist es sinnvoll nur die RX Pipe zu öffnen, wenn es denn dann noch klappt, was es aber sollte
	//nrf_openDynamicRXPipe(PIPE_Broadcast, BS_broadcast_address, 1, 0);	 //noch ungetestet
	
	nrf_flushAll();
	
	nrf_startListening();

	wdt_reset();
	wdt_enable(WDTO_8S);
	
	nrf_flushAll();
	nrf_resetIRQFlags();
	
	PORTC |= _BV(7);	//Turns ON LED in Port C pin 7
	
	while (1) { // Abfangen, wenn kein neuer Status empfangen wurde, dann neustart des Gloves
		if (nrf_getIRQStatus(&rx, &tx_done, &max_retry)) {
			if (rx) {
				nrf_readRXData(payload_RX, &rxLen, &rxPipe);
				if (rxPipe == PIPE_Data) {
					break;
				}
			}
			nrf_resetIRQFlags();
		}
	}
	
	PORTC &= ~_BV(7);	//Turns OFF LED in Port C pin 7
	wdt_disable();
	
	//void (*activeFunction)() = processingFunctions[0];

	// Endless Loop
	nrf_setRetries(500, 1);
	while (1)
	{
		
		//activeFunction();
		if (!inRecoveryMode) {
			if (*commandType == 0x01) { //BNO-CommandType
				if (*command == 1)
				{
				// process quaternions + linear acceleration
					process_quat_linAcc();
				}
				else if (*command == 0)
				{
					// default: only process quaternions
					process_quat();
				}
			} else if (*commandType == 0x00) { //General Command
				if (*command == 0x03) { //Recover Glove
					nrf_writeAckData(PIPE_Data, payload_Glove_config, 6);
				} else if (*command == 0x05) {
					PORTC |= _BV(6);	//Turns OFF LED in Port C pin 6
					_delay_ms(200);
					for (int i = 0; i < gloveID + 1; ++i)
					{
						PORTC |= _BV(7);	//Turns ON LED in Port C pin 7
						_delay_ms(200);
						PORTC &= ~_BV(7);	//Turns OFF LED in Port C pin 7
						_delay_ms(200);
					}
					PORTC &= ~_BV(6);	//Turns OFF LED in Port C pin 6
					_delay_ms(1000);
				}
			} else if (*commandType == 0x02) { //Info Command
				if (*command == 0) {		//Calibration data
					query_calibration_data();
					} else if (*command == 1) {					//Glove Config
					query_glove_config();
				}
			} /* else if (*commandType == 3) { //not useful to use it here or to send a update command more than once, even if it is possible
			
			} */
		}
		
		// increase sample ID to indicate next sample is processed and sent
		updatePacketsSampleID();
		TCNT1 = 0;
		// TODO: maybe just delay the amount of us left (10000 - TCNT1), ensure TCNT1 < 10000
		// wait until 10 ms have passed (to keep sampling time of ~100 Hz)
		while (TCNT1 < 10000)
		{
			rxLen = 0;
			if (nrf_getIRQStatus(&rx, &tx_done, &max_retry))
			{
				nrf_resetIRQFlags();
				if (rx)
				{
					while (nrf_dataAvailable())
					{
						nrf_readRXData(payload_RX, &rxLen, &rxPipe);
						if(rxPipe == PIPE_Data) {
							if (rxLen >= 2) {
								if (payload_RX[0] == 0x00) {				//Restart, query processed data or set idle mode
									if (payload_RX[1] == 0x00) {			//Query				//0x00 0x00
										//delay? or simply do nothing, not even the if statement?
									} else if (payload_RX[1] == 0x01) {		//Set in Idle Mode	//0x00 0x01
										//activeFunction = processingFunctions[2]; 
										//use function do_nothing()
										*commandType = 0; 
										*command = 1;
									} else if(payload_RX[1] == 0x02) {		//Restart Node		//0x00 0x02
										wdt_reset();
										wdt_enable(WDTO_2S);
										while(1){};
									} else if (payload_RX[1] == 0x03) {		//Recover Glove		//0x00 0x03
										nrf_writeAckData(PIPE_Data, payload_Glove_config, 6);
										inRecoveryMode = 1;
										//*commandType = 0;
										//*command = 3;
										//send GloveConfig to recover the node
									} else if (payload_RX[1] == 0x05) {		//Show ID
										if (*command != 5) {		//for the one time execution of the command  //not necessary to check for the command type at the moment because no other command type has a command 5
											PORTC |= _BV(6);	//Turns OFF LED in Port C pin 6
											_delay_ms(200);
											for (int i = 0; i < gloveID + 1; ++i)
											{
												PORTC |= _BV(7);	//Turns ON LED in Port C pin 7
												_delay_ms(200);
												PORTC &= ~_BV(7);	//Turns OFF LED in Port C pin 7
												_delay_ms(200);
											}
											PORTC &= ~_BV(6);	//Turns OFF LED in Port C pin 6
											_delay_ms(1000);
										}
										*commandType = 0; 
										*command = 5;
									}
								} else if (payload_RX[0] == 0x01) {			//Set BNO processing function
									*commandType = 1; 
									uint8_t newMode = payload_RX[1];
									if (newMode != *command && modeIsValid(newMode)) {	//Set new Mode and new processing function	//0x01 0x00 oder 0x01 0x01
										*command = newMode;
										initPackets(*command, gloveID);
										//payload_Glove_config[1] = *command;
									}
									/*if(mode == 0) {					//Quat
										process_quat();
									} else if (mode == 1) {			//LinAcc
										process_quat_linAcc();
									}*/
								} else if (payload_RX[0] == 0x02) {			//Other functions for querying information
									if (payload_RX[1] < 0x02) {				//Check that only functions are used which exist //0x02 0x00 oder 0x02 0x01
										//TODO use the functions for calibration data or glove config
										//activeFunction = queryFunctions[payload_RX[1]];
										if (*command != payload_RX[1]) {  //One time execution of the command
											if (payload_RX[1] == 0) {		//Calibration data
												query_calibration_data();
											} else if (payload_RX[1] == 1) {					//Glove Config
												query_glove_config();
											}
										}
										*commandType = 2;
										*command = payload_RX[1];
									}
								} else if (payload_RX[0] == 0x03) {			//Update Functions	//0x03 0x00
									if (payload_RX[1] == 0x00) {			//Update SessionId
										payload_Glove_config[3] = payload_RX[2];
									} if (payload_RX[1] == 0x01) {			//Update Glove Config e.g. UID, or other data
										payload_Glove_config[4] = payload_RX[2];
									}
								}
							}
							if (inRecoveryMode) {
								if ( (payload_RX[0] != 0x00) | payload_RX[1] != 0x03) { //if the command receieved is no recover command and the glove is still in recoverModes
									inRecoveryMode = 0;
								}
							}
						} else if (rxPipe == PIPE_Data_Broadcast) {
							if (payload_RX[0] == 0x00) {			//General Command
								if(payload_RX[1] == 0x02) {			//Restart Node		//0x00 0x02
									wdt_reset();
									wdt_enable(WDTO_2S);
									while(1){};
								} else if (payload_RX[1] == 0x05) {	//Show ID
									PORTC |= _BV(6);
									_delay_ms(200);
									for (int i = 0; i < gloveID + 1; ++i)
									{
										PORTC |= _BV(7);
										_delay_ms(200);
										PORTC &= ~_BV(7);
										_delay_ms(200);
									}
									PORTC &= ~_BV(6);
									_delay_ms(1000);
								}
							} else if (payload_RX[0] == 0x02) {		//Info Command
								if (payload_RX[1] == 0x00) {		//Calibration Data
									query_calibration_data();
									PORTC |= _BV(6);
									_delay_ms(500);
									PORTC &= ~_BV(6);
								} else if (payload_RX[1] == 0x01) {	//Glove Config
									query_glove_config();
									PORTC |= _BV(7);
									_delay_ms(500);
									PORTC &= ~_BV(7);
								}
							} else if (payload_RX[0] == 0x03) {		//Update Command
								if (payload_RX[1] == 0x00) {		//SessionID
									payload_Glove_config[3] = payload_RX[2];
									PORTC |= _BV(6);
									PORTC |= _BV(7);
									_delay_ms(500);
									PORTC &= ~_BV(6);
									PORTC &= ~_BV(7);
								}
							}
						}
					}
				}
			}
			_delay_ms(1);
		}
		
		// reset timer
		
	}
}