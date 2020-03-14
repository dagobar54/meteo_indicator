//#include <DS3231.h>
#include "meteo_home.h"
//#include "RF24.h"
#include "math.h"
#include "string.h"






void unixtimeToString(uint32_t unixT,char* str)
{
    unixT -= 946681200;

    static uint8_t hour;
    static uint8_t minute;
    static uint8_t second;

    second = unixT % 60;
    unixT /= 60;

    minute = unixT % 60;
    unixT /= 60;

    hour = unixT % 24;

	snprintf(str,8,"%i:%i:%i",hour,minute,second);
	//Serial.println(str);
	//delay(50);
}

void ShowMessage(char* str)
{
	//static str_len;
	//str_len = strlen(str);
	//CDC_Transmit_FS((unsigned char*)str, strlen(str));
	//HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen("RF24_CRC_8\n"), 1000);
}

