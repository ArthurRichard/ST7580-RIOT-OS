/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       ST7580 Test
 *
 * @author      Arthur RICHARD <arthur.richard-2@etu.univ-tours.fr>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>
#include "shell.h"
#include "led.h"
#include "board.h"
#include "periph/gpio.h"
#include "periph/uart.h"
#include "periph/hwrng.h"
#include "thread.h"
#include "xtimer.h"
#include "msg.h"

#include "ST7580_Serial.h"

/* Private define ------------------------------------------------------------*/
#define TRIG_BUF_SIZE   21
#define ACK_BUF_SIZE   	17

char MsgOut[100];

kernel_pid_t main_pid;
kernel_pid_t tmp_pid;
gpio_t user_button;


void AppliMasterBoard(void)
{
	uint8_t ret;
	uint8_t cRxLen;
	ST7580Frame* RxFrame;
	uint8_t lastIDRcv = 0;
	int it = 0;

	uint8_t aTrsBuffer[TRIG_BUF_SIZE] = {'T','R','I','G','G','E','R',' ',\
																						'M','E','S','S','A','G','E',' ',\
																						'I','D',':',' ','@'};
	uint8_t aRcvBuffer[ACK_BUF_SIZE];
	
	printf("P2P Communication Test - Master Board Side\n\r\n\r");
	
	while(1)
	{	
		/* Initialize Trigger Msg */
		aTrsBuffer[TRIG_BUF_SIZE-1]++;
		if (aTrsBuffer[TRIG_BUF_SIZE-1] > 'Z')
		{
			aTrsBuffer[TRIG_BUF_SIZE-1] = 'A';
		}
	
		printf("Iteration %d\n\r", ++it);
		
		/* Send Trigger Msg send */
		ret = ST7580DlData(DATA_OPT, aTrsBuffer, TRIG_BUF_SIZE, NULL);	

		/* Check TRIGGER Msg send result */
		if(ret)
		{
			/* Transmission Error */
			printf("Trigger Transmission Err: %d\n\r", ret);
			continue;
		}
		
		printf("Trigger Msg Sent, ID: %c\n\r", aTrsBuffer[TRIG_BUF_SIZE-1]);
		
		/* Wait ACK Msg sent back from slave */
		RxFrame = NULL;
		for (int j=0;((j<10) && (RxFrame==NULL));j++)
		{
			RxFrame = ST7580NextIndicationFrame();
			if (RxFrame != NULL)
			{
				/* Check if a duplicated indication frame with STX = 03 is received */
				if ((RxFrame->stx == ST7580_STX_03)&&(lastIDRcv == RxFrame->data[3+ACK_BUF_SIZE]))
				{
					RxFrame = NULL;
				}
				else
				{
					lastIDRcv = RxFrame->data[3+ACK_BUF_SIZE];
					break;
				}
			}	
			xtimer_usleep(200000);
		}
		/* Check received ACK Msg */
		if (RxFrame == NULL)
		{
			/* No ACK Msg received until timeout */
			printf("ACK Timeout - No ACK Received\n\r");
			continue;
		}
		
		cRxLen = (RxFrame->length - 4);
		
		printf("ACK Msg Received\n\r");

		if (cRxLen != ACK_BUF_SIZE){
			/* ACK len mismatch */
			printf("Wrong ACK Length\n\r");
			continue;
		}
		
		/* Copy payload from RX frame */
		memcpy(aRcvBuffer,&(RxFrame->data[4]),cRxLen);

		/* Check ID to verify if the right ACK has been received */
		if (aRcvBuffer[ACK_BUF_SIZE-1] == aTrsBuffer[TRIG_BUF_SIZE-1])
		{
			printf("ACK Msg Received, ID: %c\n\r", aRcvBuffer[ACK_BUF_SIZE-1]);
		}
		else
		{
			printf("WRONG ACK Msg Received, ID: %c\n\r", aRcvBuffer[ACK_BUF_SIZE-1]);
		}
		printf("PAYLOAD: %s\n\r\n\r", aRcvBuffer);

		xtimer_usleep(1000000);
	}
	return;
}

void AppliSlaveBoard(void)
{
	ST7580Frame* RxFrame;
	uint8_t cRxLen;
	int ret;
	uint8_t lastIDRcv = 0;
	int it = 0;
	
	uint8_t aTrsBuffer[ACK_BUF_SIZE] = {'A','C','K',' ','M','E','S','S',\
																						'A','G','E',' ','I','D',':',' ',\
																						'@'};
	uint8_t aRcvBuffer[TRIG_BUF_SIZE];
	
	printf("P2P Communication Test - Slave Board Side\n\r\n\r");

	
	while(1)
	{
		printf("Iteration %d\n\r", ++it);
		
		/* Receive Trigger Msg from M board */
		RxFrame=NULL;
		
		do
		{
			RxFrame = ST7580NextIndicationFrame();
			
			if (RxFrame != NULL)
			{
				/* Check if a duplicated indication frame with STX = 03 is received */
				if ((RxFrame->stx == ST7580_STX_03)&&(lastIDRcv == RxFrame->data[3+TRIG_BUF_SIZE]))
				{
					RxFrame = NULL;
				}
				else
				{
					lastIDRcv = RxFrame->data[3+TRIG_BUF_SIZE];
					break;
				}
			}
			xtimer_usleep(200000);
		} while(RxFrame==NULL);
	
		cRxLen = (RxFrame->length - 4);
		memcpy(aRcvBuffer,&(RxFrame->data[4]),cRxLen);
		
		printf("Trigger Msg Received, ID: %c\n\r", aRcvBuffer[TRIG_BUF_SIZE-1]);

		printf("PAYLOAD: ");
		sprintf(MsgOut, "%s", aRcvBuffer);
		printf("%s\n\r", MsgOut);

		/* Send back ACK Msg to Master Board */
		aTrsBuffer[ACK_BUF_SIZE-1] = aRcvBuffer[TRIG_BUF_SIZE-1];
		do
		{
			ret = ST7580DlData(DATA_OPT, aTrsBuffer, ACK_BUF_SIZE, NULL);
		} while (ret!=0);
		
		printf("ACK Msg Sent, ID: %c\n\r", aTrsBuffer[ACK_BUF_SIZE-1]);
		printf("PAYLOAD: ");
		sprintf(MsgOut, "%s", aTrsBuffer);
		printf("%s\n\r\n\r", MsgOut);
	}
}
void *tmpthread_handler(void *arg)
{	
	(void) 	arg;
	
	ST7580InterfaceInit();

	printf("Writing MIB_MODEM_CONF\n");
	ST7580MibWrite(MIB_MODEM_CONF, modem_config, sizeof(modem_config));
	xtimer_usleep(500000);
	printf("Writing MIB_PHY_CONF\n");
	ST7580MibWrite(MIB_PHY_CONF, phy_config, sizeof(phy_config));

	if(gpio_read(user_button))
	{
		AppliMasterBoard();
	}
	else
	{
		AppliSlaveBoard();
	}
	
	return NULL;
}


char tmp_stack[THREAD_STACKSIZE_LARGE];

int main(void)
{
	char line_buf[SHELL_DEFAULT_BUFSIZE];

	printf("ST7580 \n");
	user_button = GPIO_PIN(PORT_C, 13);
	gpio_init(user_button, GPIO_IN);

	main_pid = thread_getpid();
	tmp_pid = thread_create(tmp_stack, sizeof(tmp_stack), THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, tmpthread_handler, NULL, "tmp_thread");
	 
	shell_run(NULL, line_buf, SHELL_DEFAULT_BUFSIZE);
	
	return 0;	
}


