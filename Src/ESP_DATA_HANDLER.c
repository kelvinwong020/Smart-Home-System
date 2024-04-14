/**
  ******************************************************************************

  File:		ESP DATA HANDLER
  Author:   ControllersTech
  Updated:  3rd Aug 2020

  ******************************************************************************
  Copyright (C) 2017 ControllersTech.com

  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.

  ******************************************************************************
*/



#include "ESP_DATA_HANDLER.h"
#include "UartRingbuffer.h"
#include "stdio.h"
#include "string.h"

extern UART_HandleTypeDef huart3;
extern uint8_t fan_mode;

extern void user_pwm_setvalue(uint16_t value);
extern void fan_speed_auto(unsigned int digit);

#define wifi_uart &huart3

#define maxnumberofusers  10  // Max number of users

char buffer[20];
userDetails user[maxnumberofusers];

int usernumber = 0;

int sizeofuser (userDetails *user)
{
	int size=0;
	while (user[size].firstname[0] != '\0') size++;
	return size+1;
}

char *home_top = "<!DOCTYPE html>\n\
		<html>\n\
		<body>\n\
		<h1>STM32 ESP8266 Smart Home Server</h1>\n\
		<p>DHT11 Readings: </p>\n";



/*****************************************************************************************************************************************/

void ESP_Init (char *SSID, char *PASSWD, char *STAIP)
{
	char data[80];

	/********* Configure PB8,PB9 **********/
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
	ESP_Rst();
	Ringbuf_init();

	Uart_sendstring("AT+RST\r\n");
	HAL_Delay(2000);

	/********* AT **********/
	Uart_flush();
	Uart_sendstring("AT\r\n");
	while(!(Wait_for("OK\r\n")));


	/********* AT+CWMODE=1 **********/
	Uart_flush();
	Uart_sendstring("AT+CWMODE=1\r\n");
	while (!(Wait_for("OK\r\n")));

	/* Set Static IP Address */
	/********* AT+CWSTAIP=IPADDRESS **********/
	Uart_flush();
	sprintf (data, "AT+CIPSTA=\"%s\"\r\n", STAIP);
	Uart_sendstring(data);
	while (!(Wait_for("OK\r\n")));

	/********* AT+CWJAP="SSID","PASSWD" **********/
	Uart_flush();
	sprintf (data, "AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, PASSWD);
	Uart_sendstring(data);
	while (!(Wait_for("OK\r\n")));

	/********* AT+CIPMUX **********/
	Uart_flush();
	Uart_sendstring("AT+CIPMUX=1\r\n");
	while (!(Wait_for("OK\r\n")));

	/********* AT+CIPSERVER **********/
	Uart_flush();
	Uart_sendstring("AT+CIPSERVER=1,80\r\n");
	while (!(Wait_for("OK\r\n")));
	
	
}




int Server_Send (char *str, int Link_ID)
{
	int len = strlen (str);
	char data[80];
	Uart_flush();
	sprintf (data, "AT+CIPSEND=%d,%d\r\n", Link_ID, len);
	Uart_sendstring(data);
	while (!(Wait_for(">")));
	Uart_sendstring (str);
	while (!(Wait_for("SEND OK")));
	Uart_flush();
	sprintf (data, "AT+CIPCLOSE=%d\r\n",Link_ID);
	Uart_sendstring(data);
	while (!(Wait_for("OK\r\n")));
	return 1;
}

void Server_Handle (char *str, int Link_ID, short temp_int, short temp_deci, short humi_int, short humi_deci)
{
	char datatosend[300] = {0};
	if (!(strcmp (str, "/fan30")))
	{
		sprintf (datatosend, home_top);
		char temp[50] = "";
		sprintf(temp, "<p>Temperature: %d.%d<br>Humidity: %d.%d%%</p>\n",temp_int,temp_deci,humi_int,humi_deci);
		strcat(datatosend,temp);
		strcat(datatosend,"<p>Fan Mode: MANUAL<br>\n");
		strcat(datatosend,"Fan Speed: 30%</p>\n");
		strcat(datatosend,"<form action=\"fan60\"><input type=\"submit\" value=\"60% Fan Speed\"></form>\n");
		strcat(datatosend,"</body></html>");
		Server_Send(datatosend, Link_ID);
	}

	else if (!(strcmp (str, "/fan60")))
	{
		sprintf (datatosend, home_top);
		char temp[50] = "";
		sprintf(temp, "<p>Temperature: %d.%d<br>Humidity: %d.%d%%</p>\n",temp_int,temp_deci,humi_int,humi_deci);
		strcat(datatosend,temp);
		strcat(datatosend,"<p>Fan Mode: MANUAL<br>\n");
		strcat(datatosend,"Fan Speed: 60%</p>\n");
		strcat(datatosend,"<form action=\"fan100\"><input type=\"submit\" value=\"100% Fan Speed\"></form>\n");
		strcat(datatosend,"</body></html>");
		Server_Send(datatosend, Link_ID);
	}
	
	else if(!(strcmp(str, "/fan100"))){
		sprintf (datatosend, home_top);
		char temp[50] = "";
		sprintf(temp, "<p>Temperature: %d.%d<br>Humidity: %d.%d%%</p>\n",temp_int,temp_deci,humi_int,humi_deci);
		strcat(datatosend,temp);
		strcat(datatosend,"<p>Fan Mode: MANUAL<br>\n");
		strcat(datatosend,"Fan Speed: 100%</p>\n");
		strcat(datatosend,"<form action=\"/home\"><input type=\"submit\" value=\"Fan Auto\"></form>\n");
		strcat(datatosend,"</body></html>");
		Server_Send(datatosend, Link_ID);
	}
	
	else
	{
		sprintf (datatosend, home_top);
		char temp[50] = "";
		sprintf(temp, "<p>Temperature: %d.%d<br>Humidity: %d.%d%%</p>\n",temp_int,temp_deci,humi_int,humi_deci);
		strcat(datatosend,temp);
		strcat(datatosend,"<p>Fan Mode: AUTO</p>\n");
		strcat(datatosend,"<form action=\"fan30\"><input type=\"submit\" value=\"30% Fan Speed\"></form>\n");
		//strcat(datatosend,"<form action=\"fan60\"><input type=\"submit\" value=\"60% Fan Speed\"></form>\n");
		//strcat(datatosend,"<form action=\"fan100\"><input type=\"submit\" value=\"100% Fan Speed\"></form>\n");
		strcat(datatosend,"</body></html>");
		Server_Send(datatosend, Link_ID);
	}
	HAL_Delay(1000);
}

void Server_Start (short temp_int, short temp_deci, short humi_int, short humi_deci)
{
	char buftostoreheader[128] = {0};
	char Link_ID;
	while (!(Get_after("+IPD,", 1, &Link_ID)));

	Link_ID -= 48;
	while (!(Copy_upto(" HTTP/1.1", buftostoreheader)));
	if (Look_for("/fan30", buftostoreheader) == 1)
	{
		Server_Handle("/fan30",Link_ID,temp_int, temp_deci, humi_int, humi_deci);
		fan_mode = 1;
		user_pwm_setvalue(864);
	}

	else if (Look_for("/fan60", buftostoreheader) == 1)
	{
		Server_Handle("/fan60",Link_ID,temp_int, temp_deci, humi_int, humi_deci);
		fan_mode = 1;
		user_pwm_setvalue(1728);
	}
	
	else if(Look_for("/fan100",buftostoreheader)==1){
		Server_Handle("/fan100",Link_ID,temp_int, temp_deci, humi_int, humi_deci);
		fan_mode = 1;
		user_pwm_setvalue(2860);
	}

	else if (Look_for("/home", buftostoreheader) == 1)
	{
		Server_Handle("/home",Link_ID, temp_int, temp_deci, humi_int, humi_deci);
		fan_mode = 0;
	}
	

	else if (Look_for("/favicon.ico", buftostoreheader) == 1);

	else
	{
		Server_Handle("/ ", Link_ID, temp_int, temp_deci, humi_int, humi_deci);
		fan_speed_auto(temp_int);
	}
}

void ESP_Rst ( void )
{
	#if 0
	 ESP8266_Cmd ( "AT+RST", "OK", "ready", 2500 );   	
	
	#else
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
		HAL_Delay( 500 ); 
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
	#endif

}

