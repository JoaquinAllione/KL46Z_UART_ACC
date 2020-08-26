/* Copyright 2019, DSI FCEIA UNR - Sistemas Digitales 2
 *    DSI: http://www.dsi.fceia.unr.edu.ar/
 * Copyright 2019, 2018, Gustavo Muro
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*==================[inclusions]=============================================*/

// Standard C Included Files
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

// Project Included Files
#include "SD2_board.h"
#include "fsl_lpsci.h"
#include "fsl_uart.h"
#include "fsl_port.h"
#include "board.h"
#include "MKL46Z4.h"
#include "pin_mux.h"
#include "ringBuffer.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/
static void* pRingBufferRx;
static void* pRingBufferTx;

static void* pRingBufferRx_UART1;
static void* pRingBufferTx_UART1;
/*==================[internal functions declaration]=========================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

void uart0_ringBuffer_init(void)
{
	lpsci_config_t config;

    pRingBufferRx = ringBuffer_init(16);
    pRingBufferTx = ringBuffer_init(16);

	CLOCK_SetLpsci0Clock(0x1U);

	/* PORTA1 (pin 35) is configured as UART0_RX */
	PORT_SetPinMux(PORTA, 1U, kPORT_MuxAlt2);

	/* PORTA2 (pin 36) is configured as UART0_TX */
	PORT_SetPinMux(PORTA, 2U, kPORT_MuxAlt2);

	/*
	 * config.parityMode = kLPSCI_ParityDisabled;
	 * config.stopBitCount = kLPSCI_OneStopBit;
	 * config.enableTx = false;
	 * config.enableRx = false;
	 */
	LPSCI_GetDefaultConfig(&config);
	config.baudRate_Bps = 115200;
	config.parityMode = kLPSCI_ParityDisabled;
	config.stopBitCount = kLPSCI_OneStopBit;
	config.enableTx = true;
	config.enableRx = true;

	LPSCI_Init(UART0, &config, CLOCK_GetFreq(kCLOCK_CoreSysClk));

	/* Habilita interrupciones */
	LPSCI_EnableInterrupts(UART0, kLPSCI_RxDataRegFullInterruptEnable);
	LPSCI_EnableInterrupts(UART0, kLPSCI_TxDataRegEmptyInterruptEnable);
	EnableIRQ(UART0_IRQn);
}

void uart1_ringBuffer_init(void)
{
	uart_config_t config;

    pRingBufferRx_UART1 = ringBuffer_init(16);
    pRingBufferTx_UART1 = ringBuffer_init(16);

	/* PORTE1 (pin 35) is configured as UART1_RX */
	PORT_SetPinMux(PORTE, 1U, kPORT_MuxAlt3);

	/* PORTE0 (pin 36) is configured as UART1_TX */
	PORT_SetPinMux(PORTE, 0U, kPORT_MuxAlt3);

	UART_GetDefaultConfig(&config);
	config.baudRate_Bps = 9600;
	//config.parityMode = kUART_ParityDisabled;
	//config.stopBitCount = kUART_OneStopBit;
	config.enableTx = true;
	config.enableRx = true;

	UART_Init(UART1, &config, CLOCK_GetFreq(kCLOCK_BusClk));

	/* Habilita interrupciones */
	UART_EnableInterrupts(UART1, kUART_RxDataRegFullInterruptEnable);
	UART_EnableInterrupts(UART1, kUART_TxDataRegEmptyInterruptEnable);
	EnableIRQ(UART1_IRQn);
}

/** \brief recibe datos por puerto serie accediendo al RB
 **
 ** \param[inout] pBuf buffer a donde guardar los datos
 ** \param[in] size tamaño del buffer
 ** \return cantidad de bytes recibidos
 **/

int32_t uart0_ringBuffer_recDatos(uint8_t *pBuf, int32_t size)
{
    bool ret = false;

    /* entra sección de código crítico */
    NVIC_DisableIRQ(UART0_IRQn);

    while (!ringBuffer_isEmpty(pRingBufferRx) && ret < size)
    {
        ringBuffer_getData(pRingBufferRx, &pBuf[ret]);
        ret = true;
    }
    /* sale de sección de código crítico */
    NVIC_EnableIRQ(UART0_IRQn);

    return ret;
}

int32_t uart1_ringBuffer_recDatos(uint8_t *pBuf, int32_t size)
{
    bool ret = false;

    /* entra sección de código crítico */
    NVIC_DisableIRQ(UART1_IRQn);

    while (!ringBuffer_isEmpty(pRingBufferRx_UART1) && ret < size)
    {
        ringBuffer_getData(pRingBufferRx_UART1, &pBuf[ret]);
        ret = true;
    }
    /* sale de sección de código crítico */
    NVIC_EnableIRQ(UART1_IRQn);

    return ret;
}

/** \brief envía datos por puerto serie accediendo al RB
 **
 ** \param[inout] pBuf buffer a donde estan los datos a enviar
 ** \param[in] size tamaño del buffer
 ** \return cantidad de bytes enviados
 **/
int32_t uart0_ringBuffer_envDatos(uint8_t *pBuf, int32_t size)
{
	int32_t ret = 0;

    /* entra sección de código crítico */
    NVIC_DisableIRQ(UART0_IRQn);

    /* si el buffer estaba vacío hay que habilitar la int TX */
    if (ringBuffer_isEmpty(pRingBufferTx))
    	LPSCI_EnableInterrupts(UART0, kLPSCI_TxDataRegEmptyInterruptEnable);

    while (!ringBuffer_isFull(pRingBufferTx) && ret < size)
    {
        ringBuffer_putData(pRingBufferTx, pBuf[ret]);
        ret++;
    }

    /* sale de sección de código crítico */
    NVIC_EnableIRQ(UART0_IRQn);

    return ret;
}

int32_t uart1_ringBuffer_envDatos(uint8_t *pBuf, int32_t size)
{
	int32_t ret = 0;

    /* entra sección de código crítico */
    NVIC_DisableIRQ(UART1_IRQn);

    /* si el buffer estaba vacío hay que habilitar la int TX */
    if (ringBuffer_isEmpty(pRingBufferTx_UART1))
    	UART_EnableInterrupts(UART1, kUART_TxDataRegEmptyInterruptEnable);

    while (!ringBuffer_isFull(pRingBufferTx_UART1) && ret < size)
    {
        ringBuffer_putData(pRingBufferTx_UART1, pBuf[ret]);
        ret++;
    }

    /* sale de sección de código crítico */
    NVIC_EnableIRQ(UART1_IRQn);

    return ret;
}

void UART0_IRQHandler(void)
{
	uint8_t data;

    if ( (kLPSCI_RxDataRegFullFlag)            & LPSCI_GetStatusFlags(UART0) &&
         (kLPSCI_RxDataRegFullInterruptEnable) & LPSCI_GetEnabledInterrupts(UART0) )
	{
        /* obtiene dato recibido por puerto serie */
	    data = LPSCI_ReadByte(UART0);

		/* pone dato en ring buffer */
		ringBuffer_putData(pRingBufferRx, data);


	}

	if ( (kLPSCI_TxDataRegEmptyFlag)            & LPSCI_GetStatusFlags(UART0) &&
         (kLPSCI_TxDataRegEmptyInterruptEnable) & LPSCI_GetEnabledInterrupts(UART0) )
	{
		if (ringBuffer_getData(pRingBufferTx, &data))
		{
			/* envía dato extraído del RB al puerto serie */
		    LPSCI_WriteByte(UART0, data);
		}
		else
		{
			/* si el RB está vacío deshabilita interrupción TX */
		    LPSCI_DisableInterrupts(UART0, kLPSCI_TxDataRegEmptyInterruptEnable);
		}


	}
}

void UART1_IRQHandler(void)
{
	uint8_t data;

    if ( (kUART_RxDataRegFullFlag)            & UART_GetStatusFlags(UART1) &&
         (kUART_RxDataRegFullInterruptEnable) & UART_GetEnabledInterrupts(UART1) )
	{
        /* obtiene dato recibido por puerto serie */
	    data = UART_ReadByte(UART1);
		/* pone dato en ring buffer */
		ringBuffer_putData(pRingBufferRx_UART1, data);
	}

	if ( (kUART_TxDataRegEmptyFlag)            & UART_GetStatusFlags(UART1) &&
         (kUART_TxDataRegEmptyInterruptEnable) & UART_GetEnabledInterrupts(UART1) )
	{
		if (ringBuffer_getData(pRingBufferTx_UART1, &data))
		{
			/* envía dato extraído del RB al puerto serie */
		    UART_WriteByte(UART1, data);
		}
		else
		{
			/* si el RB está vacío deshabilita interrupción TX */
		    UART_DisableInterrupts(UART1, kUART_TxDataRegEmptyInterruptEnable);
		}
	}
}


/*==================[end of file]============================================*/

