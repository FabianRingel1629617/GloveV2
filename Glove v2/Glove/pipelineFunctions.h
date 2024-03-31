#ifndef PIPELINE_FUNCTIONS_H_
#define PIPELINE_FUNCTIONS_H_

#include "config.h"

void initPackets(uint8_t mode, uint8_t sensorId);	//Paket Initialisation

void updatePacketsSampleID();						//Paket Sample ID

uint8_t modeIsValid(uint8_t mode);					//Set new valid mode

#endif /* PIPELINE_FUNCTIONS_H_ */