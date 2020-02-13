/**
******************************************************************************
* @file    ST7580_Serial.h
* @author  CLABS
* @version 1.1.0
* @date    18-Sept-2017 
* @brief   ST7580 device specific interface functions and constants
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ST7580_H
#define ST7580_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>
#include "periph/gpio.h"
#include "periph/uart.h"
#include "periph/hwrng.h"
#include "thread.h"
#include "xtimer.h"

#ifdef __cplusplus
 extern "C" {
#endif

	 

/**
 * @addtogroup ST7580_Libraries
 * @{
 */


/**
 * @defgroup ST7580_Serial           Serial
 * @brief Management functions for ST7580 serial protocol and local messages.
 * @details See the file <i>@ref ST7580_Serial.h</i> for more details.
 * @{
 */
	 
	  
 /**
 * @defgroup Serial_Exported_Constants               Serial Exported Constants
 * @{
 */

/***********************Uncomment the PLC Modem Operating mode*****************/
//#define USE_PHY_DATA
#define USE_DL_DATA
/******************************************************************************/

#define BPSK												0
#define QPSK												1
#define PSK8												2
#define BFSK												3
#define BPSKCOD											4
#define QPSKCOD											5
#define BPSKCODPEAKAV								7

/***********************Set configuration parameters **************************/
#define CUSTOM_MIB_FREQUENCY				0
#define FREQUENCY_OVERWRITE					0
#define FREQUENCY_SET								1
#define GAIN_SELECTOR								0

#define FRAME_MODULATION						BPSKCOD
#define ZERO_CROSS_SYNC							0

#if CUSTOM_MIB_FREQUENCY
	#define TXFREQ1										0
	#define TXFREQ2										0
	#define TXFREQ3										0
#endif

#if GAIN_SELECTOR
	#define	TXGAIN										0													
#endif
/******************************************************************************/

//#define DATA_OPT 0x44
#define DATA_OPT										((CUSTOM_MIB_FREQUENCY) | \
																		 (FREQUENCY_OVERWRITE << 1) | \
																		 (FREQUENCY_SET << 2) | \
																		 (GAIN_SELECTOR << 3) | \
																		 (FRAME_MODULATION << 4) | \
																		 (ZERO_CROSS_SYNC << 7))

#define IND_FRAMES_QUEUE_LEN				6		 /* Indication Frames Queue length */
#define PHY_DATALEN_MAX							250
#define DL_DATALEN_MAX							242
#define SS_DATALEN_MAX							226

/**
 * @brief  Timeout parameters
 */
#define	IC_TMO											10000	 /* Intercharacter timeout msec	*/
#define ACK_TMO											40000	 /* ACK timeout msec */
#define STATUS_MSG_TMO									200000	 /* Status Message timeout msec */
#define CMD_TMO											4000000 /* Comand timeout */
#define RESET_IND_TMO									4000000 /* Reset Indication timeout */

#define N_TIMEOUTS									4 	 /* Three different kind of timeouts */
#define IC_TMO_IDX									0		 /* Intercharacter timeout ID	*/
#define ACK_TMO_IDX									1		 /* ACK timeout ID	*/
#define STATUS_MSG_TMO_IDX					2		 /* Status Message timeout ID	*/
#define CMD_TMO_IDX									3		 /* Comand timeout ID	*/

/**
 * @brief  Functions returns codes
 */
#define ST7580_OK										0		/* Req/Res exchange ok */
#define ST7580_TX_IN_PROGRESS				-1	/* Tx in course */
#define ST7580_ERR_CONFIRM  				-2	/* Wrong confirm code */
#define ST7580_ERR_BUF_LEN  				-3	/* Wrong received buffer length */
#define ST7580_ERR_TIMEOUT					-4	/* Timeout on response */
#define ST7580_ERR_PING							-5	/* Pong data doesn't match Ping data */
#define ST7580_ERR_ARGS							-6	/* Wrong function's argument */
#define ST7580_UNEXPECTED_FRAME			-7	/* Unexpected local frame received */
#define ST7580_RCV_BUF_TOO_SMALL		-8	/* Receive buffer too small */

#define ST7580_TXERR_NAK						-10	/* NAK received */
#define ST7580_TXERR_NO_STATUS			-11	/* No status received */
#define ST7580_TXERR_ACK_TMO				-12	/* No ack received */
#define ST7580_TXERR_BUSY						-13	/* ST7580 busy */

/**
 * @brief  Acknoledgement codes
 */
#define ST7580_ACK          				0x06 /* ST7580 ACK */
#define ST7580_NAK          				0x15 /* ST7580 NACK */
#define BUSY_MASK							0x06 

/**
 * @brief  Start of frames codes
 */
#define	ST7580_STX_02 							0x02
#define	ST7580_STX_03							0x03
#define	ST7580_STX_STATUS						0x3F

/**
 * @brief  Command codes
 */
#define CMD_RESET_REQ       				0x3C	/* Reset request command */
#define CMD_RESET_CNF       				0x3D	/* Reset confirmation command */
#define CMD_RESET_IND       				0x3E	/* Reset indication command */
#define CMD_RESET_ERR       				0x3F	/* Reset error command code */

#define CMD_MIB_WRITE_REQ   				0x08	/* MIB Write request command */
#define CMD_MIB_WRITE_CNF   				0x09	/* MIB Write confirmation command */
#define CMD_MIB_WRITE_ERR   				0x0B	/* MIB Write error command */

#define CMD_MIB_READ_REQ    				0x0C	/* MIB Read request command */
#define CMD_MIB_READ_CNF    				0x0D	/* MIB Read confirmation command */
#define CMD_MIB_READ_ERR    				0x0F	/* MIB Read error command */

#define CMD_MIB_ERASE_REQ   				0x10	/* MIB Erase request command */
#define CMD_MIB_ERASE_CNF   				0x11	/* MIB Erase confirmation command */
#define CMD_MIB_ERASE_ERR   				0x13	/* MIB Erase error command */

#define CMD_PING_REQ        				0x2C	/* PING request command */
#define CMD_PING_CNF        				0x2D	/* PING confirmation command */

#define CMD_PHY_DATA_REQ    				0x24	/* PHY Data request command */
#define CMD_PHY_DATA_CNF    				0x25	/* PHY Data confirmation command */
#define CMD_PHY_DATA_IND    				0x26	/* PHY Data indication command */
#define CMD_PHY_DATA_ERR    				0x27	/* PHY Data error command */

#define CMD_DL_DATA_REQ     				0x50	/* DL Data request command */
#define CMD_DL_DATA_CNF     				0x51	/* DL Data confirmation command */
#define CMD_DL_DATA_IND     				0x52	/* DL Data indication command */
#define CMD_DL_DATA_ERR     				0x53	/* DL Data error command */
#define CMD_DL_SNIFFER_IND  				0x5A	/* DL Sniffer indication command */

#define CMD_SS_DATA_REQ     				0x54	/* SS Data request command */
#define CMD_SS_DATA_CNF     				0x55	/* SS Data confirmation command */
#define CMD_SS_DATA_IND     				0x56	/* SS Data indication command */
#define CMD_SS_DATA_ERR     				0x57	/* SS Data error command */
#define CMD_SS_SNIFFER_IND  				0x5E	/* SS Sniffer indication command */

#define CMD_SYNTAX_ERR      				0x36	/* Syntax error command code */

/* 
 * Returns true of the given command code is an indication, false otherwise
 */
#define IS_INDICATION(CMD)					((CMD == CMD_RESET_IND) || \
																		 (CMD == CMD_PHY_DATA_IND) || \
																		 (CMD == CMD_DL_DATA_IND) || \
																		 (CMD == CMD_DL_SNIFFER_IND) || \
																		 (CMD == CMD_SS_DATA_IND) || \
																		 (CMD == CMD_SS_SNIFFER_IND))

/**
 * @brief  Indexes
 */
#define IND_FRAME_PAYLOAD_IDX				0x04	/* Payload offset inside packet */


/**
 * @brief  MIBs Objects
 */
#define MIB_MODEM_CONF      				0x00	/* Modem configuration MIB */
#define MIB_PHY_CONF        				0x01	/* PHY configuration MIB */
#define MIB_SS_KEY          				0x02	/* SS key MIB */
#define MIB_LAST_DATA_IND   				0x04	/* Last data indication MIB */
#define MIB_LAST_TX_CNF     				0x05	/* Last TX confirm MIB */
#define MIB_PHY_DATA        				0x06	/* PHY Data MIB */
#define MIB_DL_DATA         				0x07	/* DL Data MIB */
#define MIB_SS_DATA         				0x08	/* SS Data MIB */
#define MIB_HOST_IF_TOUT    				0x09	/* Host interface timeout MIB */	
#define MIB_FW_VERSION      				0x0A	/* Firmware version MIB */
 
#define ST7580_MOD_BPSK         		(0 << 4)	/* B-PSK modulation */
#define ST7580_MOD_QPSK         		(1 << 4)	/* Q-PSK modulation */
#define ST7580_MOD_8PSK         		(2 << 4)	/* 8-PSK modulation */
#define ST7580_MOD_BFSK         		(3 << 4)	/* B-FSK modulation */
#define ST7580_MOD_BPSK_COD     		(4 << 4)	/* B-PSK coded modulation */
#define ST7580_MOD_QPSK_COD     		(5 << 4)	/* Q-PSK coded modulation */
#define ST7580_MOD_BPSK_COD_PNA 		(7 << 4)	/* B-PSK coded with Peak \
																							Noise Avoidance modulation */
#define ST7580_ZC               		(1 << 7)  /* Zero crossing synchronization */
/**
 *@}
 */
 
/**
 * @defgroup Serial_Exported_Macros            Serial Exported Macros
 * @{
 */


/**
 *@}
 */



/**
 * @defgroup Serial_Exported_Types   Serial Exported Types
 * @{
 */

/**
 * @brief  Frame tx High Level state machine states.
 */
typedef enum
{
	TXREQ_LOW,
	WAIT_STATUS_FRAME,
	WAIT_TX_FRAME_DONE,
	WAIT_ACK
} TxStatus;


/**
 * @brief  Frame Tx Interrupt Level state machine states.
 */
typedef enum
{
	SEND_STX,
	SEND_LENGTH,
	SEND_COMMAND,
  	SEND_DATA,
  	SEND_CHECKSUM_LSB,
	SEND_CHECKSUM_MSB,
	TX_DONE
} TxIrqStatus;


/**
 * @brief  Rx frame state machine states
 */
typedef enum
{
	RCV_FIRST_BYTE,
	RCV_STATUS_VALUE,
	RCV_LENGTH,
	RCV_COMMAND,
	RCV_DATA,
	RCV_CHECKSUM_LSB,
	RCV_CHECKSUM_MSB
} RxIrqStatus;


/**
 * @brief  Timeout structure
 */
typedef struct
{
	uint32_t tmo;				/* The timeout value to check */
	uint32_t tmo_start_time; 	/* The start time */
} Timeout;


/**
 * @brief Command Frame structure
 */
typedef struct
{
  uint8_t stx;						/* Start of text delimiter */
  uint8_t length;					/* Length of data field */
  uint8_t command;					/* Command code */
  uint8_t data[255];					/* Data field */
  uint16_t checksum;					/* Checksum */
} ST7580Frame;


/** 
 * @brief Frame Queue structure
 */
typedef struct
{
	ST7580Frame frames[IND_FRAMES_QUEUE_LEN];	/* The frames */
	volatile uint8_t wr_idx;			/* The queue write index */
	volatile uint8_t rd_idx;			/* The queue read index */
} ST7580FrameQueue;

/** 
 * @brief ST7580 Channel structure
 */
typedef struct
{
	volatile bool wait_ack;		/* True when an ACK/NACK is being waited */
	volatile bool wait_status;	/* True when a STATUS message is waited */
	volatile bool ack_rx;		/* True when an ACK/NACK received */
	volatile uint8_t ack_rx_value;	/* Received ack value */
	volatile bool status_rx;		/* True when status received */
	volatile bool confirm_rx;		/* True when a confirm received */
	volatile bool local_frame_tx;		/* True when local frame transmitted */
	volatile bool ack_tx;			/* True if ACK/NAK has to be transmitted */
	volatile uint8_t ack_tx_value;		/* Acnowledgement value (0x06 or 0x15) */
	volatile uint8_t status_value;		/* The Received status  */
	volatile Timeout timeout[N_TIMEOUTS];	/* timeouts (milliseconds) */
  	ST7580Frame tx_frame;			/* Transmit frame */
  	ST7580Frame confirm_frame;		/* Received confirm frame */
	ST7580FrameQueue ind_frames;		/* Incoming indication frames */	
	
} ST7580Channel;

extern const uint8_t phy_config[14];
extern const uint8_t modem_config[1];
/**
 *@}
 */

/**
 * @defgroup Serial_Exported_Functions               Serial Exported Functions
 * @{
 */

void ST7580InitChannel(void);

void ST7580InterfaceInit(uart_t uart_handle, gpio_t reset, gpio_t t_req);
int ST7580Reset(void);

int ST7580MibWrite(uint8_t indexMib, const uint8_t* bufMib, uint8_t lenBuf);
int ST7580MibRead(uint8_t indexMib, uint8_t* bufMib, uint8_t lenBuf);
int ST7580MibErase(uint8_t indexMib);

int ST7580Ping(const uint8_t* pingBuf, uint8_t pingLen);

int ST7580PhyData(uint8_t plmOpts, const uint8_t* dataBuf, uint8_t dataLen, uint8_t* confData);
int ST7580DlData(uint8_t plmOpts, const uint8_t* dataBuf, uint8_t dataLen, uint8_t* confData);
int ST7580SsData(uint8_t plmOpts, const uint8_t* dataBuf, uint8_t clrLen, uint8_t encLen, uint8_t* retData);

ST7580Frame *ST7580NextIndicationFrame(void);

void NucleoST7580RxInt(void *arg, uint8_t data);
void NucleoST7580TxInt(void);

/**
 *@}
 */

/**
 *@}
 */


/**
 *@}
 */

#ifdef __cplusplus
}
#endif
#endif /* ST7580_SERIAL_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
