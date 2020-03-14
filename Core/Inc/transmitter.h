/*
 * receiver.h
 *
 *  Created on: Feb 29, 2020
 *      Author: sbaya
 */

#ifndef INC_TRANSMITTER_H_
#define INC_TRANSMITTER_H_
#include "meteo_home.h"

typedef enum{meteoIndoor,meteoOutdoor}meteoFrom;

struct meteo_data_struct meteoData[2];

typedef struct {
	double temp;
	uint32_t press;
	meteoFrom from;
} BMP180_DATA;

typedef struct {
	data_type type;
	q_commands query;
}transmit_query;
struct meteo meteoOutDoor,meteoInDoor;
struct server_ack ackData[2];


 int32_t unixtime;
uint8_t synchro_delta;

uint8_t current_channel;
uint8_t current_data_rate;
uint8_t current_power;

transmit_states transmit_state;

void transmitter_init();

#endif /* INC_TRANSMITTER_H_ */
