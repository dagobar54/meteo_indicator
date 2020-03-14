/*
 * meteo_home.h
 *
 *  Created on: Feb 16, 2020
 *      Author: sbaya
 */

#ifndef INC_METEO_HOME_H_
#define INC_METEO_HOME_H_


#include "stdio.h"

struct RTCDateTime
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t dayOfWeek;
    uint32_t unixtime;
};

typedef enum {none_command=0,set_time,start_command,test_command}mode_commands;
typedef enum {
  start_state=0,  // -только включился и сделал первую передачу
  connecting_state, // -получил ack и проверяет устойчивость связи (подбор мощности передатчика
  connected_state,  // -связь установлена. Перешел в поминутный режим обмена
  lost_connectState,// -связь временно потеряна. Попытка восстановления
  disconnected_state,// -режим работы без связи
  test_state       // -специальный режим тестирования
}transmit_states;
typedef enum {none_q=0,get_time,get_meteoOutDoor,get_meteoInDoor}q_commands;

typedef enum {start_pipe=0,disconnected_pipe,connected_pipe,test_pipe}pipe_statuses;
typedef enum {power_normal,power_lowing,power_highing}power_mode;
typedef enum {data_null,data_meteoOutDoor,data_meteoInDoor,exactly_time}data_type;
struct meteo{
	uint32_t unixtime;
	int16_t T;
	uint16_t P,H,CO2;
};
struct meteo_data_struct{
	uint32_t round_tripDelay;
	struct meteo meteo_data;
	uint8_t query;
	uint8_t type_of_data;
	uint8_t state;
	uint8_t power;
	uint8_t vcc;
	uint8_t vc1;
	uint8_t vc2;
	uint8_t vc3;
} ;
struct server_ack{
	int32_t time_interval; //задержка перед следующей передачей
	int32_t server_time;
	struct meteo meteo_data;
	mode_commands command;
	q_commands ack_query;
	uint8_t channel,data_rate,power;
	uint8_t ac1;
};
struct pipe_data{
	pipe_statuses pipe_status;
	uint32_t lastConnectedTime;
	long int exchange_counter;
	int longer_exchange_counter;
	power_mode power_tune;
	struct server_ack ackData;
};

struct sensors_data{
	struct meteo meteo_data;
	uint16_t lux;
} ;



void ShowMessage(char* str);
void unixtimeToString(uint32_t unixT,char* str);


#endif /* INC_METEO_HOME_H_ */
