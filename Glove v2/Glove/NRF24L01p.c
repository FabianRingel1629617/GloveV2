/*
 * NRF24L01p.c
 *
 * Created: 26.03.2020 18:21:14
 *  Author: Jochen
 */ 

#include <avr/io.h>

#include "config.h"
#include <util/delay.h>
#include "nrf.h"
#include "NRF24L01p.h"
#include "SPI.h"


	

void nrf_init(uint8_t channel, uint8_t dataRate, uint8_t addressWidth, uint8_t CRCLength)
{
	nrf_powerUp();																												// Wenn das Modul nicht im StandBy-I Modus ist wird diese in den Modus gesetz ander bleibt es in dem Modus
	nrf_setChannel(channel);																									// Setzen des Channels
	nrf_setDataRate(dataRate);																									// Setzen der Datenrate 0=250Kbps, 1=1Mbps, 2=2=2Mbps
	nrf_setRFOutPower(3);																										// Setzen der Output Power
	nrf_setAddressWidth(addressWidth);																							// Setzen der Adressbreite
	nrf_setCRCLength(CRCLength);																								// Aktivieren/Deaktivieren des CRC und gegebenenfalls setzen der Länge des CRC
	nrf_setRetries(500, 15);																									// Setzen der Anzahl der Auto Retransmits und der Verzögerungszeit (Verzögerungszeit, Wiederholversuche)
}

void nrf_powerUp()
{
	if (!nrf_isPoweredUp())
	{
		// CE low - Standby-I
		PORTB &= ~_BV(CE);																										// Chip-Enable auf Low setzen, um  nichts zu senden/empfangen
		SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) | (1 << PWR_UP));															// Lesen der Konfiguration und setzen des PWR-Up Bits auf 1 um in den StandBy-I Modus zu kommen
		// 1.5 ms power up delay w/o external clock, 150 us else.
		// Just to be sure: wait 2 ms (will only be executed at the beginning or very sparsely anyway)
		_delay_ms(2);
	}
}

void nrf_powerDown()
{
	// CE low - Standby-I
	PORTB &= ~_BV(CE);																											// Chip-Enable auf Low setzen, um nichts zu senden/empfangen
	SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) & ~(1 << PWR_UP));																// Lesen der konfiguration und setzen des PWR-Up Bits auf 0 um in den Power Down Modus zu kommen
}

uint8_t nrf_isPoweredUp()
{
	return SPI_Read_Byte(CONFIG) & (1 << PWR_UP);																				//Auslesen des PWR_UP Bits
}


void nrf_setModeRX(void)
{
	// CE low - Standby-I
	PORTB &= ~_BV(CE);																											// Chip-Enable auf Low setzen, um  nichts zu senden/empfangen
	
	SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) | (1 << PRIM_RX));																//
	
	// nrf_flushTX();
	// nrf_flushRX();
	
	// Reset IRQ status
	nrf_resetIRQFlags();
	
	// Mask TX_DR and MAX_RT interrupts
	// SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) | (1 << MASK_TX_DS) | (1 << MASK_MAX_RT));
	
	// PORTB |= _BV(CE);                             //CE high
	// _delay_us(150);
}

void nrf_setModeTX(void)
{
	// CE low - Standby-I
	PORTB &= ~_BV(CE);
	
	SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) & ~(1 << PRIM_RX));
	
	// nrf_flushTX();                     //Flush TX FIFO
	// nrf_flushRX();
	
	nrf_resetIRQFlags();
	
	// Mask TX_DR and MAX_RT interrupts
	// SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) | (1 << MASK_TX_DS) | (1 << MASK_MAX_RT));
	
	// _delay_us(150);
}

void nrf_writeTXData(uint8_t* data, uint8_t len)
{
	nrf_flushTX();
	nrf_resetIRQFlags();
	
	//Transmit payload with ACK enabled
	SPI_Write_Bytes(W_TX_PAYLOAD, data, len);
	
	// TODO: do not send data directly, call this outside the writeData method
	nrf_startSending();
	
	while (nrf_isSending());
}

void nrf_writeTXDataNoAck(uint8_t* data, uint8_t len)
{
	nrf_flushTX();
	nrf_resetIRQFlags();
	
	//Transmit payload with ACK disabled
	SPI_Write_Bytes(W_TX_PAYLOAD_NOACK, data, len);
	
	// TODO: do not send data directly, call this outside the writeData method
	nrf_startSending();
	
	while (nrf_isSending());
}

void nrf_writeAckData(uint8_t pipe, uint8_t* data, uint8_t len) //Schreiben des Ack-Payloads
{
	// flush needed? (I guess not, should be done outside...)
	// nrf_flushTX();
	
	if (pipe > 5)
	{
		return;
	}
	
	SPI_Write_Bytes(W_ACK_PAYLOAD + pipe, data, len);
}

void nrf_startSending(void)
{
	_delay_us(10);        //Need at least 10us before sending
	PORTB |= _BV(CE);             //CE high
	_delay_us(12);        //Hold CE high for at least 10us and not longer than 4ms
	PORTB &= ~_BV(CE);             //CE low
}

void nrf_readRXData(uint8_t* data, uint8_t* len, uint8_t* pipe)
{
	if (nrf_RXFifoEmpty())
	{
		*len = 0;
		*pipe = 0;
		return;
	}
	
	*pipe = nrf_getRXPipeNumber();
	
	if (nrf_hasDynamicPayloadLength(*pipe))
	{
		*len = nrf_getDynamicPayloadLength();
	}
	else
	{
		*len = nrf_getPayloadLength(*pipe);
	}
	
	//Pull down chip select
	PORTB &= ~_BV(CSN);
	
	uint8_t length = *len;
	//Send command to read RX payload
	SPI_Write(R_RX_PAYLOAD);
	
	for (uint8_t i = 0; i < length; ++i)
	{
		data[i] = SPI_Write(0x00);
	}
	
	PORTB |= _BV(CSN);
	_delay_us(1);
}

uint8_t nrf_getStatus(void)
{
	uint8_t status;
	PORTB &= ~_BV(CSN);            //CSN low
	status = SPI_Write(NOP);
	PORTB |= _BV(CSN);            //CSN high
	_delay_us(1);
	return status;
}

void nrf_maskIRQ(uint8_t rx_ready, uint8_t tx_done, uint8_t tx_maxRetry)
{
	uint8_t config = SPI_Read_Byte(CONFIG);
	
	// clear the interrupt flags
	config &= ~(1 << MASK_RX_DR | 1 << MASK_TX_DS | 1 << MASK_MAX_RT);
	// set the specified interrupt flags
	config |= ((rx_ready ? 1:0) << MASK_RX_DR) | ((tx_done ? 1:0) << MASK_TX_DS) | ((tx_maxRetry ? 1:0) << MASK_MAX_RT);
	SPI_Write_Byte(CONFIG, config);
}

uint8_t nrf_getIRQStatus(uint8_t* rx_ready, uint8_t* tx_done, uint8_t* tx_maxRetry)
{
	uint8_t status = nrf_getStatus();

	// Report to the user what happened
	*rx_ready = status & (1 << RX_DR);
	*tx_done = status & (1 << TX_DS);
	*tx_maxRetry = status & (1 << MAX_RT);
	return status & 0x70;
}

void nrf_resetIRQFlags(void)
{
	//Reset IRQ-flags in status register
	SPI_Write_Byte(STATUS, 0x70);
}

void nrf_flushRX(void)
{
	//_delay_us(10);
	PORTB &= ~_BV(CSN);            //CSN low
	//_delay_us(10);
	SPI_Write(FLUSH_RX);
	//_delay_us(10);
	PORTB |= _BV(CSN);            //CSN high
	_delay_us(1);
}

void nrf_flushTX(void)
{
	//_delay_us(10);
	PORTB &= ~_BV(CSN);            //CSN low
	//_delay_us(10);
	SPI_Write(FLUSH_TX);
	//_delay_us(10);
	PORTB |= _BV(CSN);            //CSN high
	_delay_us(1);
}

void nrf_flushAll(void)
{
	nrf_flushRX();
	nrf_flushTX();
}

void nrf_reuseTX(void)
{
	PORTB &= ~_BV(CSN);
	SPI_Write(REUSE_TX_PL);
	PORTB |= _BV(CSN);
	_delay_us(1);
}

uint8_t nrf_isSending(void)
{
	//Read the current status
	uint8_t status = nrf_getStatus();

	// If sending successful (TX_DS) or max retries exceeded (MAX_RT)
	if (status & ((1 << TX_DS) | (1 << MAX_RT)))// || TXFifoEmpty())
	{
		return 0;       // False
	}
	return 1;           // True
}

void nrf_startListening(void)
{
	PORTB |= _BV(CE);
	// ce high to csn low requires minimum 4 us
	_delay_us(10);
}

void nrf_stopListening(void)
{
	PORTB &= ~_BV(CE);
}

uint8_t nrf_RXFifoFull()
{
	return SPI_Read_Byte(FIFO_STATUS) & (1 << RX_FULL);
}

uint8_t nrf_RXFifoEmpty()
{
	return SPI_Read_Byte(FIFO_STATUS) & (1 << RX_EMPTY);
}

uint8_t nrf_TXFifoFull()
{
	return SPI_Read_Byte(FIFO_STATUS) & (1 << FIFO_FULL);
}

uint8_t nrf_TXFifoEmpty()
{
	return SPI_Read_Byte(FIFO_STATUS) & (1 << TX_EMPTY);
}

uint8_t nrf_dataAvailable(void)
{
	return !(SPI_Read_Byte(FIFO_STATUS) & (1 << RX_EMPTY));
}

uint8_t nrf_getRXPipeNumber(void)
{
	return (nrf_getStatus() >> RX_P_NO) & 0x07;
}


// nrf setup methods

void nrf_setChannel(uint8_t channel)
{
	// max channel is 125
	if (channel > 125)
	{
		channel = 125;
	}
	SPI_Write_Byte(RF_CH, channel);
}

uint8_t nrf_getChannel()
{
	return SPI_Read_Byte(RF_CH);
}

void nrf_setAddressWidth(uint8_t width)
{																																//Adressbreite 3 Bytes = 01, 4 Bytes = 10, 3 Bytes = 11
	if (width < 3)						
	{
		width = 3;
	}
	else if (width > 5)							
	{
		width = 5;
	}
	SPI_Write_Byte(SETUP_AW, (width - 2) & 0x03);																				//Adressbreite veringern um auf das Binär-Aquivalent zu kommen und dann mit Maske 03 (0000 0011) setzen
}

uint8_t nrf_getAddressWidth(void)
{
	return SPI_Read_Byte(SETUP_AW) + 2;
}

void nrf_setCRCLength(uint8_t length)
{
	uint8_t config = SPI_Read_Byte(CONFIG) & ~((1 << CRCO) | (1 << EN_CRC));													//Lesen des Config Registers und CRCO und EN_CRC auf 0 setzen

	if (length == 0)
	{
		// Do nothing, we turned it off above.
	}
	else if (length == 1)
	{
		config |= (1 << EN_CRC);																								//setzen des EN_CRC Bits, um CRC zu aktivieren mit 1 Bit CRC
	}
	else
	{
		config |= (1 << EN_CRC);																								//setzen des EN_CRC Bits, um CRC zu aktivieren mit 1 Bit CRC
		config |= (1 << CRCO);																									//setzen de CRCO Bits um 2 Bit CRC zu haben, nicht gesetzt bedeutet 1 Bit CRC
	}
	SPI_Write_Byte(CONFIG, config);
}

uint8_t nrf_getCRCLength(void)
{
	uint8_t result = 0;
	
	uint8_t config = SPI_Read_Byte(CONFIG);

	if (config & (1 << EN_CRC) || SPI_Read_Byte(EN_AA))
	{
		if (config & (1 << CRCO))
		{
			result = 2;
		}
		else
		{
			result = 1;
		}
	}

	return result;
}

void nrf_disableCRC(void)
{
	SPI_Write_Byte(CONFIG, SPI_Read_Byte(CONFIG) & ~(1 << EN_CRC));
}


void nrf_openTXPipe(const uint8_t* address, uint8_t numBytes, uint8_t enAutoAck, uint8_t enDynAck)
{
	nrf_setTXAddress(address, 5);
	
	if (enAutoAck)
	{
		nrf_openRXPipe(0, address, numBytes, enAutoAck, enDynAck);
	}
}

void nrf_openDynamicTXPipe(const uint8_t* address, uint8_t enAckPayload, uint8_t enDynAck, uint8_t pipe)
{
	nrf_setTXAddress(address, 5);
	nrf_openDynamicRXPipe(pipe, address, enAckPayload, enDynAck);
}


void nrf_openRXPipe(uint8_t pipe, const uint8_t* address, uint8_t numBytes, uint8_t enAutoAck, uint8_t enDynAck)
{
	nrf_setRXAddress(pipe, address, 5);
	nrf_enableRXAddress(pipe);
	nrf_disableDynamicPayloadLength(pipe);
	nrf_setPayloadLength(pipe, numBytes);
	
	if (enAutoAck)
	{
		nrf_enableAutoAck(pipe);
		if (enDynAck)
		{
			nrf_enableDynamicAck();
		}
	}
	else
	{
		nrf_disableAutoAck(pipe);
	}
}

void nrf_openDynamicRXPipe(uint8_t pipe, const uint8_t* address, uint8_t enAckPayload, uint8_t enDynAck)
{
	nrf_setRXAddress(pipe, address, 5);																							//setzen der Receive-Adresse und Länge für entsprechende Pipe
	nrf_enableRXAddress(pipe);																									//Aktivieren der Pipe zum Empfangen 
	// just to be sure: set num bytes to be received to 32 (0 means pipe not used according to data sheet)							
	nrf_setPayloadLength(pipe, 32);																								//Festlegen der Payloadlänge für Pipe
	nrf_enableDynamicPayloadLength(pipe);																						//Aktiveren der dynamischen Payloads für Adresse
	
	if (enAckPayload)
	{
		nrf_enableAckPayload();																									//Aktiveren das ein Payload beim Ack Mitgesendet werden kann
	}
	else
	{
		nrf_disableAckPayload();																								//Deaktiveren das ein Payload beim Ack Mitgesendet werden kann
	}
	
	if (enDynAck)
	{
		nrf_enableDynamicAck();																									//Aktiveren des Commandos, dass beim Schreiben kein AutoAck benötigt wird
	}
}

void nrf_closeRXPipe(uint8_t pipe)
{
	nrf_disableRXAddress(pipe);
}



void nrf_setTXAddress(const uint8_t* address, uint8_t addrLen)
{
	uint8_t i;
	PORTB &= ~_BV(CSN);																											//Setzen des CSN-Bits auf Low damit der NRF an dem SPI-Port zuhört
	
	//Setup p0 pipe address for receiving
	SPI_Write(W_REGISTER + TX_ADDR);																							//W_Register = Befehel, dass in die Register geschrieben werden soll  + TX_ADDR = Adresse des Register in das Geschrieben werden soll, hier das Register für die Sende-Adresse, an die die Daten sollen
	
	// write address, LSB first
	for (i = addrLen; i > 0; --i)																								
	{
		SPI_Write(address[i-1]);																								//Senden der Adress aber in Umgekehrter Reihenfalge mit LSB zuerset und MSB am Schluss
	}
	PORTB |= _BV(CSN);																											//Setzen des CSN-Bits auf High damit der NRF nicht mehr an dem SPI-Port zuhört
	_delay_us(1);
}

void nrf_setRXAddress(uint8_t pipe, const uint8_t* address, uint8_t addrLen)
{
	// only pipes 0 and 1 are allowed full addresses, other pipes only have different LSB address
	if (pipe < 2)																												//Nur Pipe 0 und 1 exestieren mit eigener Adresse aller anderen haben adressen wo nur das LSB anders ist als die Adresse von Pipe0
	{
		uint8_t i;
		PORTB &= ~_BV(CSN);																										//Setzen des CSN-Bits auf Low damit der NRF an dem SPI-Port zuhört
		
		//Setup pipe address for receiving
		SPI_Write(W_REGISTER + RX_ADDR_P0 + pipe);																				//W_Register = Befehel, dass in die Register geschrieben werden soll  + RX_ADDR_P0 = Adresse des Register in das Geschrieben werden soll, hier das Register für die Pipe0 Adresse + Pipe Nummer
		
		// write address, LSB first
		for (i = addrLen; i > 0; --i)
		{
			SPI_Write(address[i-1]);																							//Senden der Adress aber in Umgekehrter Reihenfalge mit LSB zuerset und MSB am Schluss
		}
		PORTB |= _BV(CSN);
		_delay_us(1);
	}
	else if (pipe < 6)																											//Schreiben des LBS für Pipes 2-5
	{
		// Setup pipe address for receiving, only write LSB
		SPI_Write_Byte(RX_ADDR_P0 + pipe, address[addrLen-1]);
	}
}


void nrf_setRetries(uint16_t delay_us, uint8_t count)																			//AutoRetransmits
{
	if (delay_us < 250)
	{
		delay_us = 250;
	}
	else if (delay_us > 4000)
	{
		delay_us = 4000;
	}
	if (count > 15)																												//0000 = Keine Retransmits, 0001 = 1 Retransmit, ..., 1111 = 15 Retransmits 
	{
		count = 15;
	}
	
	uint8_t delay = (delay_us / 250) - 1;																						//Berechnen des Binär_Aquivalent zu Zeit in MicroSekunden
	SPI_Write_Byte(SETUP_RETR, (delay << 4) | count);																			//Verschieben des Verzögerung für richtige Position im Register, Verküpfung der Retries mit Verzögerung, schreiben von beidem
}

uint8_t nRF_getRetryCount(void)
{
	return SPI_Read_Byte(OBSERVE_TX) & 0x0F;
}

void nrf_setPayloadLength(uint8_t pipe, uint8_t size)
{
	if (pipe > 5)
	{
		return;
	}
	if (size > 32)
	{
		size = 32;
	}
	SPI_Write_Byte(RX_PW_P0 + pipe, size);																						//Schreiben der Länge des Payload für Pipe in entsprechendes Register, 0 = Nicht genutzt, 1 = 1 Byte, 2 = 2 Byte, ...
}

uint8_t nrf_getPayloadLength(uint8_t pipe)
{
	if (pipe > 5)
	{
		return 0;
	}
	return SPI_Read_Byte(RX_PW_P0 + pipe);
}

uint8_t nrf_getDynamicPayloadLength()
{
	return SPI_Read_Byte(R_RX_PL_WID);
}

void nrf_setDataRate(uint8_t rate)
{
	// read RF setup and mask out data rate bits
	uint8_t dataRate = SPI_Read_Byte(RF_SETUP) & 0xD7; 
	if (rate == 0)																												//D7 = 1101 0111 lässt alle Bits unverändert außer den Bity für die Datenrate (Bit 3 und 5)
	{
		// 250 kBit/s
		dataRate |= 0x20;																										//Setzt das Bit 5 auf 1
	}
	else if (rate == 1)
	{
		// 1 MBit/s
		dataRate |= 0;																											//Lässt alle Bits unverändert
	}
	else
	{
		// 2 MBit/s
		dataRate |= 0x08;																										//Setzt das Bit 3 auf 1
	}
	
	SPI_Write_Byte(RF_SETUP, dataRate);																							//Schreibt die geänderte Datenrate per SPI ins RF_SETUP register
}

uint8_t nrf_getDataRate(void)
{
	// read RF setup and mask out non data rate bits
	uint8_t dataRate = SPI_Read_Byte(RF_SETUP) & 0x28;
	if (dataRate == 0x20)
	{
		// 250 kBit/s
		return 0;
	}
	else if (dataRate == 0)
	{
		// 1 MBit/s
		return 1;
	}
	else // if (dataRate == 0x08)
	{
		// 2 MBit/s
		return 2;
	}
}

void nrf_setRFOutPower(uint8_t power)
{
	if (power > 3)
	{
		power = 3;																												//0 (00) = -18dBm, 1 (01) = -12dBm, 2 (10) = -6dBm, 3 (11) = 0dBm
	}
	SPI_Write_Byte(RF_SETUP, (SPI_Read_Byte(RF_SETUP) & ~0x06) | (power << RF_PWR));											//Lesen der alten Konfiguration, Löschen der Power UP Konfiguration mit 06 (0000 0110), Schiften der neuen Konfiguration um 1 damit an richtiger stelle und dann mit Oder-Verknüpfung setzen
}

uint8_t nrf_getRFOutPower(void)
{
	return (SPI_Read_Byte(RF_SETUP) & 0x06) >> RF_PWR;
}

void nrf_enableAutoAck(uint8_t pipe)
{
	if (pipe > 5)
	{
		return;
	}
	SPI_Write_Byte(EN_AA, SPI_Read_Byte(EN_AA) | (1 << pipe));																	//Lesen des Feature-Registers und setzen des EN_AA Bits auf 1, um AutoAck zuzulassen
}

void nrf_disableAutoAck(uint8_t pipe)
{
	if (pipe > 5)
	{
		return;
	}
	SPI_Write_Byte(EN_AA, SPI_Read_Byte(EN_AA) & ~(1 << pipe));
}


void nrf_enableRXAddress(uint8_t pipe)
{
	if (pipe > 5)
	{
		return;
	}
	SPI_Write_Byte(EN_RXADDR, SPI_Read_Byte(EN_RXADDR) | (1 << pipe));															//Pipe Aktivieren = Lesen des Register EN_RXADDR (enthält Status ob Pipes Aktviert sind = Empfangen können), andern des entsprechenden Bits der Pipe auf 1 und zurückschreiben in EN_RXADDR Register
}

void nrf_disableRXAddress(uint8_t pipe)
{
	if (pipe > 5)
	{
		return;
	}
	SPI_Write_Byte(EN_RXADDR, SPI_Read_Byte(EN_RXADDR) & ~(1 << pipe));
}


void nrf_enableDynamicPayloadLength(uint8_t pipe)
{
	if (pipe > 5)
	{
		return;
	}
	// dynamic payload lengths and auto ack has to be enabled
	nrf_enableDynamicPayloadLengths();																							//Aktivieren von Payload mit dynamischer Länge
	nrf_enableAutoAck(pipe);																									//Aktivieren von AutoAck
	
	SPI_Write_Byte(DYNPD, SPI_Read_Byte(DYNPD) | (1 << pipe));																	//Auslesen des DYNPD-Regsiters (enthält Status für jedes Pipe, ob dynamischer Payload aktiviert ist) und setzen des entsprechenden Bits der Pipe auf 1 zum Aktivieren
}

void nrf_disableDynamicPayloadLength(uint8_t pipe)
{
	if (pipe > 5)
	{
		return;
	}
	SPI_Write_Byte(DYNPD, SPI_Read_Byte(DYNPD) & ~(1 << pipe));
}

uint8_t nrf_hasDynamicPayloadLength(uint8_t pipe)
{
	if (pipe > 5)
	{
		return 0;
	}
	return SPI_Read_Byte(DYNPD) & (1 << pipe);
}

void nrf_enableDynamicPayloadLengths()
{
	SPI_Write_Byte(FEATURE, SPI_Read_Byte(FEATURE) | (1 << EN_DPL));															//Lesen des Feature-Registers und setzen des EN_DPL Bits auf 1, um dynamische Payloadlänge zuzulassen
}

void nrf_disableDynamicPayloadLengths()
{
	SPI_Write_Byte(FEATURE, SPI_Read_Byte(FEATURE) & ~(1 << EN_DPL));

	// Disable dynamic payload on all pipes
	SPI_Write_Byte(DYNPD, 0);
}

uint8_t nrf_hasDynamicPayloadLengths(void)
{
	return SPI_Read_Byte(FEATURE) & (1 << EN_DPL);
}


void nrf_enableAckPayload()
{
	// dynamic payload length has to be enabled
	nrf_enableDynamicPayloadLengths();
	SPI_Write_Byte(FEATURE, SPI_Read_Byte(FEATURE) | (1 << EN_ACK_PAY));														//Lesen des Feature-Registers und setzen des EN_ACK_PAY Bits auf 1,um AutoAck mit Payload zuzulassen (AckPayload + AutoAck)
}

void nrf_disableAckPayload()
{
	SPI_Write_Byte(FEATURE, SPI_Read_Byte(FEATURE) & ~(1 << EN_ACK_PAY));
}


void nrf_enableDynamicAck()
{
	SPI_Write_Byte(FEATURE, SPI_Read_Byte(FEATURE) | (1 << EN_DYN_ACK));
}

void nrf_disableDynamicAck()
{
	SPI_Write_Byte(FEATURE, SPI_Read_Byte(FEATURE) & ~(1 << EN_DYN_ACK));
}

uint8_t nrf_hasDynamicAck()
{
	return SPI_Read_Byte(FEATURE) & (1 << EN_DYN_ACK);
}



uint8_t SPI_Read_Byte(uint8_t reg)
{
	//_delay_us(10);
	PORTB &= ~_BV(CSN);	//CSN low
	//_delay_us(10);
	SPI_Write(R_REGISTER + reg);
	//_delay_us(10);
	reg = SPI_Write(NOP);
	//_delay_us(10);
	PORTB |= _BV(CSN);	//CSN high
	_delay_us(1);
	return reg;
}

void SPI_Write_Byte(uint8_t reg, uint8_t data)
{
	//_delay_us(10);
	PORTB &= ~_BV(CSN);	//CSN low
	_delay_us(1);
	//_delay_us(10);
	SPI_Write(W_REGISTER + reg);
	//_delay_us(10);
	SPI_Write(data);
	//_delay_us(10);
	PORTB |= _BV(CSN);	//CSN high
	_delay_us(1);
}

void SPI_Write_Bytes(uint8_t reg, uint8_t* data, uint8_t len)
{
	//_delay_us(10);
	PORTB &= ~_BV(CSN);	//CSN low
	//_delay_us(10);
	SPI_Write(reg);
	//_delay_us(10);
	writePayload(data, len);
	//_delay_us(10);
	PORTB |= _BV(CSN);	//CSN high
	_delay_us(1);
}


void writePayload(uint8_t* data, uint8_t len)
{
	for (uint8_t i = 0; i < len; ++i)
	{
		SPI_Write(data[i]);
	}
}