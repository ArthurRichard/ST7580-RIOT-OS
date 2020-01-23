/**
******************************************************************************
* @file    ST7580_Serial.c
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

#include "ST7580_Serial.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/**
 * @addtogroup ST7580_Libraries
 * @{
 */

/**
 * @addtogroup ST7580_Serial
 * @{
 */

/**
 * @defgroup Serial_Private_TypesDefinitions  Serial Private TypesDefinitions
 * @{
 */

/**
 *@}
 */

/**
 * @defgroup Serial_Private_Defines           Serial Private Defines
 * @{
 */

/**
 *@}
 */

/**
 * @defgroup Serial_Private_Macros            Serial Private Macros
 * @{
 */

/**
 *@}
 */

/**
 * @defgroup Serial_Private_Functions            Serial Private Functions
 * @{
 */

static bool ST7580TmoExpired(uint8_t tmoIdx);
static void ST7580TmoSet(uint8_t tmoIdx, uint32_t tmo);
static void ST7580FrameChecksum(ST7580Frame *frame);
static bool ST7580FrameReady(void);
static ST7580Frame *ST7580RecvConfirm(void);
static int ST7580SendFrame(ST7580Frame *frame);

/**
 *@}
 */

/**
 * @defgroup Serial_Private_Variables         Serial Private Variables
 * @{
 */
static volatile gpio_t PLM_GPIO_RESETN_PIN;
static volatile gpio_t PLM_GPIO_T_REQ_PIN;

/**
* @brief  Instance of ST7580Channel struct used through the application
*/
static ST7580Channel ch;

/**
* @brief ST7580 PHY configuration parameters fitting
*/
const uint8_t phy_config[14] = {0x01, 0xC9, 0x08, 0x01, 0x8E, 0x70, 0x0E,
								0x15, 0x00, 0x00, 0x02, 0x35, 0x9B, 0x58};

/**
* @brief ST7580 MODEM configuration parameters fitting
*/
#ifdef USE_PHY_DATA
const uint8_t modem_config[1] = {0x00};
#endif

/**
* @brief ST7580 MODEM configuration parameters fitting
*/
#ifdef USE_DL_DATA
const uint8_t modem_config[1] = {0x11};
#endif

/**
 *@}
 */

/** @defgroup Serial_Private_FunctionPrototypes       Serial Private FunctionPrototypes
 * @{
 */

/**
 * @}
 */

/** @defgroup Serial_Private_Functions                 Serial Private Functions
 * @{
 */

/**
* @brief  Initialize the ST7580 channel
* @param  None
* @retval None
*/
volatile uint8_t UART_TX_EN = 0;

void __HAL_UART_ENABLE_IT(void)
{
	UART_TX_EN = 1;
}

void __HAL_UART_DISABLE_IT(void)
{
	UART_TX_EN = 0;
}

void ST7580InitChannel(void)
{
	uint8_t i;
	Timeout tmo_ini = {0, 0};

	ch.ack_rx = false;
	ch.status_rx = false;
	ch.confirm_rx = false;
	ch.local_frame_tx = false;
	ch.ack_tx = false;
	ch.wait_ack = false;
	ch.wait_status = false;
	for (i = 0; i < N_TIMEOUTS; i++)
	{
		ch.timeout[i] = tmo_ini;
	}
	ch.ind_frames.rd_idx = 0;
	ch.ind_frames.wr_idx = 0;
}

/**
* @brief  Initializes ST7580 UART and GPIOs interfaces and wait for PLC Modem start.
* @param  None
* @retval None
*/
void ST7580InterfaceInit(void)
{
	ST7580Frame *rx_frame;
	
	ST7580InitChannel();

	PLM_GPIO_RESETN_PIN = GPIO_PIN(PORT_A, 8);
	PLM_GPIO_T_REQ_PIN = GPIO_PIN(PORT_A, 5);

	gpio_init(PLM_GPIO_RESETN_PIN, GPIO_OUT);
	gpio_init(PLM_GPIO_T_REQ_PIN, GPIO_OUT);

	gpio_set(PLM_GPIO_T_REQ_PIN);
	uart_poweron(UART_DEV(1));
	if(uart_init(UART_DEV(1), 57600, NucleoST7580RxInt, NULL) != UART_OK)	printf("ST7580: UART INIT ERROR");

	/*HAL_GPIO_WritePin(PLM_GPIO_RESETN_PORT,PLM_GPIO_RESETN_PIN,GPIO_PIN_RESET);
  	HAL_Delay(1500);
  	HAL_GPIO_WritePin(PLM_GPIO_RESETN_PORT,PLM_GPIO_RESETN_PIN,GPIO_PIN_SET);*/
	gpio_clear(PLM_GPIO_RESETN_PIN);
	xtimer_usleep(1500000);
	gpio_set(PLM_GPIO_RESETN_PIN);
	/* Wait first indication frame after reset */
	do
	{
		rx_frame = ST7580NextIndicationFrame();
		xtimer_usleep(100000);
	} while ((rx_frame == NULL) || (rx_frame->command != CMD_RESET_IND));
	return;
}

/**
* @brief  Reset ST7580 PLC Modem.
* @param  None
* @retval None
*/
int ST7580Reset(void)
{
	int ret;
	ST7580Frame *rxframe = NULL;

	ch.tx_frame.stx = ST7580_STX_02;
	ch.tx_frame.command = CMD_RESET_REQ;
	ch.tx_frame.length = 0;

	/* Update the checksum */
	ST7580FrameChecksum(&(ch.tx_frame));
	/* Send frame */
	while ((ret = ST7580SendFrame(&(ch.tx_frame))) == ST7580_TX_IN_PROGRESS)
		;
	if (ret != ST7580_OK)
	{
		return ret;
	}

	/* Wait for response */
	ST7580TmoSet(CMD_TMO_IDX, CMD_TMO);
	while (!ST7580FrameReady() && !ST7580TmoExpired(CMD_TMO_IDX))
	{
	};
	/* No response, timeout expired */
	if (ST7580TmoExpired(CMD_TMO_IDX))
	{
		return ST7580_ERR_TIMEOUT;
	}
	rxframe = ST7580RecvConfirm();
	/* Confirmation frame not correct */
	if (rxframe->command != CMD_RESET_CNF)
	{
		return ST7580_ERR_CONFIRM;
	}
	/* Command confirmed */
	return 0;
}

/**
* @brief  Write ST7580 PLC Modem MIB parameter.
* @param  indexMib	Selects MIB parameter to be written
* @param  bufMib   Pointer to buffer containing value to be written
* @param  lenBuf		Length of buffer pointed by buf
* @retval 0 command confirmed, error code otherwise
*/
int ST7580MibWrite(uint8_t indexMib, const uint8_t *bufMib, uint8_t lenBuf)
{
	int ret;
	ST7580Frame *rxframe = NULL;

	ch.tx_frame.stx = ST7580_STX_02;
	ch.tx_frame.command = CMD_MIB_WRITE_REQ;
	ch.tx_frame.length = lenBuf + 1;
	ch.tx_frame.data[0] = indexMib;
	memcpy(&(ch.tx_frame.data[1]), bufMib, lenBuf);

	/* update the checksum */
	ST7580FrameChecksum(&(ch.tx_frame));
	/* Send frame */
	while ((ret = ST7580SendFrame(&(ch.tx_frame))) == ST7580_TX_IN_PROGRESS);
	if (ret != ST7580_OK)
	{
		return ret;
	}

	/* wait for response */
	ST7580TmoSet(CMD_TMO_IDX, CMD_TMO);
	while (!ST7580FrameReady() && !ST7580TmoExpired(CMD_TMO_IDX))
	{
	};
	/* No response, timeout expired */
	if (ST7580TmoExpired(CMD_TMO_IDX))
	{
		return ST7580_ERR_TIMEOUT;
	}
	/* Get the received frame */
	rxframe = ST7580RecvConfirm();
	if (rxframe->command == CMD_MIB_WRITE_ERR)
	{
		/* Error frame received, return received error code */
		return rxframe->data[0];
	}
	else if (rxframe->command != CMD_MIB_WRITE_CNF)
	{
		/* Confirmation frame not correct */
		return ST7580_ERR_CONFIRM;
	}
	/* Command confirmed */
	return 0;
}

/**
* @brief  Read ST7580 PLC Modem MIB parameter.
* @param  indexMib	Selects MIB parameter to be read
* @param  bufMib   Pointer to buffer to store read value
* @param  lenBuf		Length of buffer pointed by buf
* @retval 0 command confirmed, error code otherwise
*/
int ST7580MibRead(uint8_t indexMib, uint8_t *bufMib, uint8_t lenBuf)
{
	int ret;
	ST7580Frame *rxframe = NULL;

	ch.tx_frame.stx = ST7580_STX_02;
	ch.tx_frame.command = CMD_MIB_READ_REQ;
	ch.tx_frame.length = 1;
	ch.tx_frame.data[0] = indexMib;

	/* Update the checksum */
	ST7580FrameChecksum(&(ch.tx_frame));
	/* Send frame */
	while ((ret = ST7580SendFrame(&(ch.tx_frame))) == ST7580_TX_IN_PROGRESS);
	if (ret != ST7580_OK)
	{
		return ret;
	}

	/* Wait for response */
	ST7580TmoSet(CMD_TMO_IDX, CMD_TMO);
	while (!ST7580FrameReady() && !ST7580TmoExpired(CMD_TMO_IDX))
	{
	};
	/* No response, timeout expired */
	if (ST7580TmoExpired(CMD_TMO_IDX))
	{
		return ST7580_ERR_TIMEOUT;
	}
	/* Get the received frame */
	rxframe = ST7580RecvConfirm();

	if (rxframe->command == CMD_MIB_READ_ERR)
	{
		/* Error frame received, return received error code */
		return rxframe->data[0];
	}
	else if (rxframe->command != CMD_MIB_READ_CNF)
	{
		/* Confirmation frame not correct */
		return ST7580_ERR_CONFIRM;
	}
	if (rxframe->length > lenBuf)
	{
		/* Provided buffer is not long enough, return error */
		return ST7580_ERR_BUF_LEN;
	}
	memcpy(bufMib, rxframe->data, rxframe->length);
	/* Command confirmed */
	return 0;
}

/**
* @brief  Erase ST7580 PLC Modem MIB parameter.
* @param  indexMib	Selects MIB parameter to be erased
* @retval 0 command confirmed, error code otherwise
*/
int ST7580MibErase(uint8_t indexMib)
{
	int ret;
	ST7580Frame *rxframe = NULL;

	ch.tx_frame.stx = ST7580_STX_02;
	ch.tx_frame.command = CMD_MIB_ERASE_REQ;
	ch.tx_frame.length = 1;
	ch.tx_frame.data[0] = indexMib;

	/* Update the checksum */
	ST7580FrameChecksum(&(ch.tx_frame));
	/* Send frame */
	while ((ret = ST7580SendFrame(&(ch.tx_frame))) == ST7580_TX_IN_PROGRESS);
	if (ret != ST7580_OK)
	{
		return ret;
	}

	/* Wait for response */
	ST7580TmoSet(CMD_TMO_IDX, CMD_TMO);
	while (!ST7580FrameReady() && !ST7580TmoExpired(CMD_TMO_IDX))
	{
	};
	/* No response, timeout expired */
	if (ST7580TmoExpired(CMD_TMO_IDX))
	{
		return ST7580_ERR_TIMEOUT;
	}
	/* Get the received frame */
	rxframe = ST7580RecvConfirm();

	if (rxframe->command == CMD_MIB_ERASE_ERR)
	{
		/* Error frame received, return received error code */
		return rxframe->data[0];
	}
	else if (rxframe->command != CMD_MIB_ERASE_CNF)
	{
		/* Confirmation frame not correct */
		return ST7580_ERR_CONFIRM;
	}

	/* Command confirmed */
	return 0;
}

/**
* @brief  Ping ST7580 PLC Modem.
* @param  pingBuf   Pointer to buffer containing ping test data to be sent.
								If ping is success ST7580 PLC Modem will reply with the same
								data
* @param  pingLen		Length of buffer pointed by buf
* @retval 0 command confirmed, error code otherwise
*/
int ST7580Ping(const uint8_t *pingBuf, uint8_t pingLen)
{
	int ret;
	ST7580Frame *rxframe = NULL;

	ch.tx_frame.stx = ST7580_STX_02;
	ch.tx_frame.command = CMD_PING_REQ;
	ch.tx_frame.length = pingLen;
	memcpy(&(ch.tx_frame.data[1]), pingBuf, pingLen);

	/* Update the checksum */
	ST7580FrameChecksum(&(ch.tx_frame));
	/* Send frame */
	while ((ret = ST7580SendFrame(&(ch.tx_frame))) == ST7580_TX_IN_PROGRESS)
		;
	if (ret != ST7580_OK)
	{
		return ret;
	}

	/* Wait for response */
	ST7580TmoSet(CMD_TMO_IDX, CMD_TMO);
	while (!ST7580FrameReady() && !ST7580TmoExpired(CMD_TMO_IDX))
	{
	};
	/* No response, timeout expired */
	if (ST7580TmoExpired(CMD_TMO_IDX))
	{
		return ST7580_ERR_TIMEOUT;
	}
	/* Get received frame */
	rxframe = ST7580RecvConfirm();
	if (rxframe->command != CMD_PING_CNF)
	{
		/* Confirmation frame not correct */
		return ST7580_ERR_CONFIRM;
	}
	if (!memcmp(rxframe->data, pingBuf, pingLen))
	{
		return ST7580_ERR_PING;
	}
	/* Command confirmed */
	return 0;
}

/**
* @brief  Send data via PLC in PHY mode.
* @param  plmOpts  Transmission options
* @param  dataBuf		Pointer to buffer containing data to be sent.
* @param  dataLen		Length of buffer pointed by buf
* @param	confData	Pointer to buffer to store transmission confirmation data
					from ST7580 PLC Modem, if requested
* @retval 0 command confirmed, error code otherwise
*/
int ST7580PhyData(uint8_t plmOpts, const uint8_t *dataBuf, uint8_t dataLen, uint8_t *confData)
{
	int ret;
	ST7580Frame *rxframe = NULL;
	uint8_t offset = 1;

	if (dataLen > PHY_DATALEN_MAX)
	{
		return ST7580_ERR_ARGS;
	}

	ch.tx_frame.stx = ST7580_STX_02;
	ch.tx_frame.command = CMD_PHY_DATA_REQ;
	ch.tx_frame.length = dataLen + 1;
	ch.tx_frame.data[0] = plmOpts;

#if CUSTOM_MIB_FREQUENCY
	ch.tx_frame.data[1] = TXFREQ1;
	ch.tx_frame.data[2] = TXFREQ2;
	ch.tx_frame.data[3] = TXFREQ3;
	offset += 3;
#endif

#if GAIN_SELECTOR
	ch.tx_frame.data[offset] = TXGAIN;
	offset += 1;
#endif

	memcpy(&(ch.tx_frame.data[offset]), dataBuf, dataLen);

	/* Update the checksum */
	ST7580FrameChecksum(&(ch.tx_frame));
	/* Send frame */
	while ((ret = ST7580SendFrame(&(ch.tx_frame))) == ST7580_TX_IN_PROGRESS)
		;
	if (ret != ST7580_OK)
	{
		return ret;
	}

	/* Wait for response */
	ST7580TmoSet(CMD_TMO_IDX, CMD_TMO);
	while (!ST7580FrameReady() && !ST7580TmoExpired(CMD_TMO_IDX))
	{
	};
	/* No response, timeout expired */
	if (ST7580TmoExpired(CMD_TMO_IDX))
	{
		return ST7580_ERR_TIMEOUT;
	}

	/* Get received frame */
	rxframe = ST7580RecvConfirm();
	if (rxframe->command == CMD_PHY_DATA_ERR)
	{
		/* Error frame received, return received error code */
		return rxframe->data[0];
	}
	else if (rxframe->command != CMD_PHY_DATA_CNF)
	{
		/* Confirmation frame not correct */
		return ST7580_ERR_CONFIRM;
	}

	if (confData)
	{
		/* Return data was requested, copy it */
		memcpy(confData, rxframe->data, 5);
	}
	/* Command confirmed */
	return 0;
}

/**
* @brief  Send data via PLC in DL mode.
* @param  plmOpts  Transmission options
* @param  dataBuf		Pointer to buffer containing data to be sent.
* @param  dataLen		Length of buffer pointed by buf
* @param	confData	Pointer to buffer to store transmission confirmation data
					from ST7580 PLC Modem, if requested
* @retval 0 command confirmed, error code otherwise
*/
int ST7580DlData(uint8_t plmOpts, const uint8_t *dataBuf, uint8_t dataLen, uint8_t *confData)
{
	int ret;
	ST7580Frame *rxframe = NULL;
	uint8_t offset = 1;

	if (dataLen > DL_DATALEN_MAX)
	{
		return ST7580_ERR_ARGS;
	}

	ch.tx_frame.stx = ST7580_STX_02;
	ch.tx_frame.command = CMD_DL_DATA_REQ;
	ch.tx_frame.length = dataLen + 1;
	ch.tx_frame.data[0] = plmOpts;

#if CUSTOM_MIB_FREQUENCY
	ch.tx_frame.data[1] = TXFREQ1;
	ch.tx_frame.data[2] = TXFREQ2;
	ch.tx_frame.data[3] = TXFREQ3;
	offset += 3;
#endif

#if GAIN_SELECTOR
	ch.tx_frame.data[offset] = TXGAIN;
	offset += 1;
#endif

	memcpy(&(ch.tx_frame.data[offset]), dataBuf, dataLen);

	/* Update the checksum */
	ST7580FrameChecksum(&(ch.tx_frame));

	/* Send frame */
	while ((ret = ST7580SendFrame(&(ch.tx_frame))) == ST7580_TX_IN_PROGRESS);
	if (ret != ST7580_OK)
	{
		return ret;
	}
	/* Wait for response */
	ST7580TmoSet(CMD_TMO_IDX, CMD_TMO);
	while (!ST7580FrameReady() && !ST7580TmoExpired(CMD_TMO_IDX))
	{
	};
	/* No response, timeout expired */
	if (ST7580TmoExpired(CMD_TMO_IDX))
	{
		return ST7580_ERR_TIMEOUT;
	}
	rxframe = ST7580RecvConfirm();
	if (rxframe->command == CMD_DL_DATA_ERR)
	{
		/* Error frame received, return received error code */
		return rxframe->data[0];
	}
	else if (rxframe->command != CMD_DL_DATA_CNF)
	{
		/* Confirmation frame not correct */
		return ST7580_ERR_CONFIRM;
	}
	if (confData)
	{
		/* Return data was requested, copy it */
		memcpy(confData, rxframe->data, 5);
	}
	/* Command confirmed */
	return 0;
}

/**
* @brief  Send data via PLC in SS mode.
* @param  plmOpts Transmission options
* @param  dataBuf	Pointer to buffer containing data to be sent.
* @param  clrLen Buf portion that has to be sent in clear
* @param  encLen Buf portion that has to be encypted
* @param	retData	Pointer to buffer to store transmission confirmation data
					from ST7580 PLC Modem, if requested
* @retval 0 command confirmed, error code otherwise
*/
int ST7580SsData(uint8_t plmOpts, const uint8_t *dataBuf, uint8_t clrLen, uint8_t encLen, uint8_t *retData)
{
	int ret;
	ST7580Frame *rxframe = NULL;
	uint8_t offset = 1;

	uint16_t len = clrLen + encLen;

	if ((len > SS_DATALEN_MAX) || ((encLen == 0) && (clrLen < 16)) || ((encLen > 0) && (len < 4)))
	{
		return ST7580_ERR_ARGS;
	}

	ch.tx_frame.stx = ST7580_STX_02;
	ch.tx_frame.command = CMD_SS_DATA_REQ;
	ch.tx_frame.length = len + 2;
	ch.tx_frame.data[0] = plmOpts;

#if CUSTOM_MIB_FREQUENCY
	ch.tx_frame.data[1] = TXFREQ1;
	ch.tx_frame.data[2] = TXFREQ2;
	ch.tx_frame.data[3] = TXFREQ3;
	offset += 3;
#endif
#if GAIN_SELECTOR
	ch.tx_frame.data[offset] = TXGAIN;
	offset += 1;
#endif
	ch.tx_frame.data[offset] = clrLen;
	memcpy(&(ch.tx_frame.data[offset + 1]), dataBuf, len);

	/* Update the checksum */
	ST7580FrameChecksum(&(ch.tx_frame));
	/* Send frame */
	while ((ret = ST7580SendFrame(&(ch.tx_frame))) == ST7580_TX_IN_PROGRESS)
		;
	if (ret != ST7580_OK)
	{
		return ret;
	}

	/* Wait for response */
	ST7580TmoSet(CMD_TMO_IDX, CMD_TMO);
	while (!ST7580FrameReady() && !ST7580TmoExpired(CMD_TMO_IDX))
	{
	};
	/* No response, timeout expired */
	if (ST7580TmoExpired(CMD_TMO_IDX))
	{
		return ST7580_ERR_TIMEOUT;
	}

	/* Get the received frame */
	rxframe = ST7580RecvConfirm();

	if (rxframe->command == CMD_SS_DATA_ERR)
	{
		/* Error frame received, return received error code */
		return rxframe->data[0];
	}
	else if (rxframe->command != CMD_SS_DATA_CNF)
	{
		/* Confirmation frame not correct */
		return ST7580_ERR_CONFIRM;
	}

	if (retData)
	{
		/* Return data was requested, copy it */
		memcpy(retData, rxframe->data, 5);
	}

	/* Command confirmed */
	return 0;
}

/**
* @brief  Get next ST7580 PLC Modem indication frame.
* @param  None
* @retval NULL if no indication frame is available, pointer no next indication
					frame otherwise
*/
ST7580Frame *ST7580NextIndicationFrame(void)
{
	ST7580Frame *ret_frame = NULL;

	if (ch.ind_frames.rd_idx != ch.ind_frames.wr_idx)
	{
		ret_frame = &(ch.ind_frames.frames[ch.ind_frames.rd_idx++]);
		if (ch.ind_frames.rd_idx == IND_FRAMES_QUEUE_LEN)
		{
			ch.ind_frames.rd_idx = 0;
		}
	}
	/* Return the frame or NULL */
	return ret_frame;
}

/**
* @brief  Check if a timeout is expired
* @param  tmoIdx Index of the timeout to be checked
* @retval true is a timeout is expired, false otherwise
*/
static bool ST7580TmoExpired(uint8_t tmoIdx)
{
	uint32_t time;

	//time = HAL_GetTick();
	time = xtimer_now_usec();

	if (time >= ch.timeout[tmoIdx].tmo_start_time)
	{
		return (time - ch.timeout[tmoIdx].tmo_start_time) >= ch.timeout[tmoIdx].tmo;
	}
	else
	{
		return (time + (0xFFFFFFFF - ch.timeout[tmoIdx].tmo_start_time) >= ch.timeout[tmoIdx].tmo);
	}
}

/**
* @brief  Check if a timeout is expired
* @param  tmoIdx Index of the timeout to be checked
* @retval None
*/
static void ST7580TmoSet(uint8_t tmo_idx, uint32_t tmo)
{
	ch.timeout[tmo_idx].tmo = tmo;
	//ch.timeout[tmo_idx].tmo_start_time = HAL_GetTick();
	ch.timeout[tmo_idx].tmo_start_time = xtimer_now_usec();
}

/**
* @brief  Computes the checksum of a frame
* @param  frame Input frame for timeout coumputing
* @retval None
*/
static void ST7580FrameChecksum(ST7580Frame *frame)
{
	uint8_t i;

	frame->checksum = frame->command + frame->length;
	for (i = 0; i < frame->length; i++)
	{
		frame->checksum += frame->data[i];
	}
}

/**
* @brief  Check if a confirm frame has been received
* @param  None
* @retval true if a frame has beeen received, false otherwise
*/
static bool ST7580FrameReady(void)
{
	if (ch.confirm_rx)
	{
		ch.confirm_rx = false;
		return true;
	}
	return false;
}

/**
* @brief  Get the received confirm frame
* @param  None
* @retval The received confirm frame
*/
static ST7580Frame *ST7580RecvConfirm(void)
{
	return &(ch.confirm_frame);
}

/**
* @brief  State machine for local frame receive
* @param  step Current iteration state machine step
* @retval Next iteration state machine step
*/
static RxIrqStatus ST7580RcvLocalFrame(RxIrqStatus step, uint8_t c)
{
	static uint8_t len, i;
	static uint16_t checksum;
	static volatile ST7580Frame *local_frame;
	static bool rcv_indication;

	switch (step)
	{
	/* Receive length */
	case RCV_LENGTH:
		len = c;
		ch.confirm_frame.length = len;
		ch.ind_frames.frames[ch.ind_frames.wr_idx].length = len;
		checksum = c;
		ST7580TmoSet(IC_TMO_IDX, IC_TMO);
		step = RCV_COMMAND;
		break;

	/* Receive command */
	case RCV_COMMAND:
		if (IS_INDICATION(c))
		{
			rcv_indication = true;
			local_frame = &(ch.ind_frames.frames[ch.ind_frames.wr_idx]);
		}
		else
		{
			rcv_indication = false;
			local_frame = &(ch.confirm_frame);
		}
		local_frame->command = c;
		checksum += c;
		i = 0;
		ST7580TmoSet(IC_TMO_IDX, IC_TMO);
		if (local_frame->length == 0)
		{
			step = RCV_CHECKSUM_LSB;
		}
		else
		{
			step = RCV_DATA;
		}
		break;

	/* Receive Data */
	case RCV_DATA:
		local_frame->data[i++] = c;
		checksum += c;
		ST7580TmoSet(IC_TMO_IDX, IC_TMO);
		if (i == len)
		{
			step = RCV_CHECKSUM_LSB;
		}
		break;

	/* Receive Checksum LSB */
	case RCV_CHECKSUM_LSB:
		local_frame->checksum = c;
		ST7580TmoSet(IC_TMO_IDX, IC_TMO);
		step = RCV_CHECKSUM_MSB;
		break;

	/* Receive Checksum MSB */
	case RCV_CHECKSUM_MSB:
		local_frame->checksum |= ((uint16_t)(c << 8));
		/* Send ack */
		ch.ack_tx = true;
		if (local_frame->checksum == checksum)
		{
			if (!rcv_indication)
			{
				/* Validate local frame */
				ch.confirm_rx = true;
			}
			else
			{
				/* Advance the indication frame queue */
				ch.ind_frames.wr_idx++;
				if (ch.ind_frames.wr_idx == IND_FRAMES_QUEUE_LEN)
				{
					ch.ind_frames.wr_idx = 0;
				}
			}
			/* Ack value */
			ch.ack_tx_value = ST7580_ACK;
		}
		else
		{
			/* Nak value */
			ch.ack_tx_value = ST7580_NAK;
		}
		/* Triggers transmission interrupt */
		__HAL_UART_ENABLE_IT();
		step = RCV_FIRST_BYTE;
		
		break;

	/* Default status to never be reached */
	default:
		break;
	}

	return step;
}

/**
* @brief  Rx interrupt state machine
* @param  Uarthandle Handle for ST7580 serial communication
* @retval None
*/
void NucleoST7580RxInt(void *arg, uint8_t data)
{
	static volatile RxIrqStatus step = RCV_FIRST_BYTE;
	/* Wait for first byte */
	uint8_t c;
	(void) arg;

	//printf("RX STEP: %d ", step);

	/* First check whether a timeout is expired or not */
	if (ST7580TmoExpired(IC_TMO_IDX))
	{
		/* Reset state machine */
		step = RCV_FIRST_BYTE;
	}

	/* Get received character; */
	//c = LL_USART_ReceiveData8(UartHandle->Instance);
	c = data;

	/* Switch on RX status */
	switch (step)
	{
		/* Wait for first byte */
		case RCV_FIRST_BYTE:
			/* Switch on received character */
			switch (c)
			{
				case ST7580_ACK:
				case ST7580_NAK:
					if (ch.wait_ack)
					{
						ch.ack_rx = true;
						ch.ack_rx_value = c;
						ch.wait_ack = false;
					}
					else
					{
						ch.wait_status = false;
					}
					break;

				case ST7580_STX_02:
				case ST7580_STX_03:
					ch.confirm_frame.stx = c;
					ch.ind_frames.frames[ch.ind_frames.wr_idx].stx = c;
					ST7580TmoSet(IC_TMO_IDX, IC_TMO);
					step = RCV_LENGTH;
					break;

				case ST7580_STX_STATUS:
					if (ch.wait_status)
					{
						ST7580TmoSet(IC_TMO_IDX, IC_TMO);
						step = RCV_STATUS_VALUE;
					}
					else
					{
						ch.wait_ack = false;
					}
					break;
				default:
					ch.wait_status = false;
					ch.wait_ack = false;
					break;
			}
			break;

		/* Receive Status Byte */
		case RCV_STATUS_VALUE:
			ch.status_value = c;
			ch.status_rx = true;
			step = RCV_FIRST_BYTE;
			ch.wait_status = false;
			break;

		/* Receive Local Frame */
		default:
			step = ST7580RcvLocalFrame(step, c);
			break;
	}
}

/**
* @brief  Tx interrupt state machine
* @param  Uarthandle Handle for ST7580 serial communication
* @retval None
*/
void NucleoST7580TxInt(void)
{
	static TxIrqStatus step = SEND_STX;
	static uint8_t i = 0;
	uint8_t uart_data[1];

	/* Send ack or nack, if requested */
	if (ch.ack_tx)
	{
		//LL_USART_TransmitData8(UartHandle->Instance, ch.ack_tx_value);
		uart_data[0] = ch.ack_tx_value;
		uart_write(UART_DEV(1), uart_data, 1);
		step = TX_DONE;
	}
	/* Switch on tx status */
	switch (step)
	{
		/* Send STX character */
		case SEND_STX:
			//LL_USART_TransmitData8(UartHandle->Instance, ch.tx_frame.stx);
			uart_data[0] = ch.tx_frame.stx;
			uart_write(UART_DEV(1), uart_data, 1);
			step = SEND_LENGTH;
			break;

		/* Send length */
		case SEND_LENGTH:
			//HAL_GPIO_WritePin(PLM_GPIO_T_REQ_PORT, PLM_GPIO_T_REQ_PIN, GPIO_PIN_SET);
			//LL_USART_TransmitData8(UartHandle->Instance, ch.tx_frame.length);
			gpio_set(PLM_GPIO_T_REQ_PIN);
			uart_data[0] = ch.tx_frame.length;
			uart_write(UART_DEV(1), uart_data, 1);
			step = SEND_COMMAND;
			break;

		/* Send data */
		case SEND_COMMAND:
			//LL_USART_TransmitData8(UartHandle->Instance, ch.tx_frame.command);
			uart_data[0] = ch.tx_frame.command;
			uart_write(UART_DEV(1), uart_data, 1);
			step = SEND_DATA;
			break;

		/* Send data */
		case SEND_DATA:
			if (i == ch.tx_frame.length)
			{
				step = SEND_CHECKSUM_LSB;
			}
			else
			{
				//LL_USART_TransmitData8(UartHandle->Instance, ch.tx_frame.data[i++]);
				uart_data[0] = ch.tx_frame.data[i++];
				uart_write(UART_DEV(1), uart_data, 1);
			}
			break;

		/* Send checksum (LSB) */
		case SEND_CHECKSUM_LSB:
			//LL_USART_TransmitData8(UartHandle->Instance, ch.tx_frame.checksum);
			uart_data[0] = ch.tx_frame.checksum;
			uart_write(UART_DEV(1), uart_data, 1);
			step = SEND_CHECKSUM_MSB;
			break;

		/* Send checksum (MSB) */
		case SEND_CHECKSUM_MSB:
			//LL_USART_TransmitData8(UartHandle->Instance, ch.tx_frame.checksum >> 8);
			uart_data[0] = ch.tx_frame.checksum >> 8;
			uart_write(UART_DEV(1), uart_data, 1);
			step = TX_DONE;
			break;

		/* TX DONE */
		case TX_DONE:
			/* Interrupt acknowledge */
			//__HAL_UART_DISABLE_IT(UartHandle, UART_IT_TXE);
			__HAL_UART_DISABLE_IT();
			
			if (!ch.ack_tx)
			{
				/* If we sent a local frame */
				ch.local_frame_tx = true;
			}
			else
			{
				ch.ack_tx = false;
			}

			step = SEND_STX;
			i = 0;
			break;
	}
	if(UART_TX_EN)	NucleoST7580TxInt();
}

/**
* @brief  Send frame state machine
* @param  frame Frame to be sent
* @retval Next iteration state machine's step
*/
static int ST7580SendFrame(ST7580Frame *frame)
{
	static TxStatus step = TXREQ_LOW;
	int ret = ST7580_TX_IN_PROGRESS;
	static bool firstIter = true;

	(void) frame;

	//printf("SF step: %d\n", step);

	/* Switch on tx state machine step */
	switch (step)
	{
		/* Pull down the TXREQ */
		case TXREQ_LOW:
			/* Frame not sent yet */
			ch.local_frame_tx = false;
			/* Status frame not received yet */
			ch.status_rx = false;
			
			/* Pull down the T_REQ */
			//HAL_GPIO_WritePin(PLM_GPIO_T_REQ_PORT, PLM_GPIO_T_REQ_PIN, GPIO_PIN_RESET);
			gpio_clear(PLM_GPIO_T_REQ_PIN);
			
			/* Initialize the status message rcv time */
			ST7580TmoSet(STATUS_MSG_TMO_IDX, STATUS_MSG_TMO);
			/* Wait for status frame */
			step = WAIT_STATUS_FRAME;
			break;

		/* Wait for receival of Status frame */
		case WAIT_STATUS_FRAME:
			if (firstIter)
			{
				ch.wait_status = true;
				firstIter = false;
			}
			/* Check for timeout expiration */
			if (!ST7580TmoExpired(STATUS_MSG_TMO_IDX))
			{
				if (ch.status_rx)
				{
					/* Check for busyness */
					if (ch.status_value & BUSY_MASK)
					{
						//HAL_GPIO_WritePin(PLM_GPIO_T_REQ_PORT, PLM_GPIO_T_REQ_PIN, GPIO_PIN_SET);
						gpio_set(PLM_GPIO_T_REQ_PIN);
						step = TXREQ_LOW;
						ret = ST7580_TXERR_BUSY;
					}
					else
					{
						/* Triggers TX interrupt */
						step = WAIT_TX_FRAME_DONE;
						//__HAL_UART_ENABLE_IT(&pUartPlmHandle, UART_IT_TXE);
						__HAL_UART_ENABLE_IT();
					}
					firstIter = true;
				}
			}
			else
			{
				//HAL_GPIO_WritePin(PLM_GPIO_T_REQ_PORT, PLM_GPIO_T_REQ_PIN, GPIO_PIN_SET);
				gpio_set(PLM_GPIO_T_REQ_PIN);
				step = TXREQ_LOW;
				ret = ST7580_TXERR_NO_STATUS;
				ch.wait_status = false;
				firstIter = true;
			}
			break;

		/* Wait for end of frame transmission */
		case WAIT_TX_FRAME_DONE:
			if (ch.local_frame_tx == true)
			{
				ST7580TmoSet(ACK_TMO_IDX, ACK_TMO);
				step = WAIT_ACK;
			}
			break;

		/* Wait for the acknoledgement */
		case WAIT_ACK:
			if (firstIter)
			{
				ch.wait_ack = true;
				firstIter = false;
			}
			if (!ST7580TmoExpired(ACK_TMO_IDX))
			{
				if (ch.ack_rx)
				{
					/* Consume the acknowledgement */
					ch.ack_rx = false;
					/* Set the return value */
					ret = (ch.ack_rx_value == ST7580_ACK) ? (ST7580_OK) : (ST7580_TXERR_NAK);
					/* Reset the state machine */
					step = TXREQ_LOW;
					firstIter = true;
				}
			}
			else
			{
				ret = ST7580_TXERR_ACK_TMO;
				/* Reset the state machine */
				step = TXREQ_LOW;
				ch.wait_ack = false;
				firstIter = true;
			}
			break;
	}
	if(UART_TX_EN) NucleoST7580TxInt();
	return ret;
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
