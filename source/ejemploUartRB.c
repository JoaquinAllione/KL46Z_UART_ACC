/* Copyright 2017, DSI FCEIA UNR - Sistemas Digitales 2
 *    DSI: http://www.dsi.fceia.unr.edu.ar/
 * Copyright 2017, Diego Alegrechi
 * Copyright 2017, 2018, Gustavo Muro
 * All rights reserved.
 *
 *
 *
 */

/*==================[inclusions]=============================================*/

// Standard C Included Files
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

// Project Included Files
#include "SD2_board.h"
#include "fsl_lpsci.h"
#include "fsl_port.h"
#include "board.h"
#include "MKL46Z4.h"
#include "pin_mux.h"
#include "uart_ringBuffer.h"
#include "SD2_I2C.h"
#include "mma8451.h"

#define FIN_DE_TRAMA (0x0A) //caracter de fin de trama
#define INICIO_DE_TRAMA 58 //caracter de inicio de trama
#define NUM_GRUPO 53
#define LENGTH_BUFFER 20

/* ======== RESOLUCION CON UART 0 ======== */
uint8_t buffer[3];
uint8_t buffer_trama[LENGTH_BUFFER]; //buffer que almacena la instruccion recibida
int8_t index_bytes; //indice que marca en que posicion del buffer (arreglo de char) se debe escribir el proximo byte
bool dato_recibido = false;

// ===============//
//Para lectura del Acelerometro...
int16_t Acc;

void mefPrincipal(void);

void clear_buffer(uint8_t *pbuf){
	int8_t i;
	for(i=0;i<LENGTH_BUFFER;i++){
		pbuf[i] = '\0';
	}
}

bool fin_trama = false;

void main(void)
{

	BOARD_BootClockRUN();

	// Se inicializan funciones de la placa
	board_init();
	/* ============= I2C ================= */
	SD2_I2C_init();
	/* =========== MMA8451 =============== */
	mma8451_init(); //inicializacion del acelerometro
	mma8451_setDataRate(DR_50hz);
	NVIC_DisableIRQ(PORTC_PORTD_IRQn);//se desactivan las interrupciones del acelerometro hasta que sea necesario
	/* ============ UART0 ================ */
	uart0_ringBuffer_init(); //inicia la UART0 y crea los buffers para el envio y recepcion de datos

	clear_buffer(buffer);//limpia el buffer donde se almacenan los datos ya verificados (instrucciones)

	board_setLed(BOARD_LED_ID_VERDE, BOARD_LED_MSG_OFF);
	board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_OFF);

    while(1)
    {
    	dato_recibido = uart0_ringBuffer_recDatos(buffer, sizeof(buffer));
    	if((dato_recibido == true)||(fin_trama == true)){
    		mefPrincipal();
    	}
    }
}

typedef enum{
	EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA = 0,
	EST_MEF_PRINCIPAL_ESP_1ER_CARACTER,
	EST_MEF_PRINCIPAL_ESP_2DO_CARACTER,
	EST_MEF_PRINCIPAL_ESP_3ER_CARACTER,
	EST_MEF_PRINCIPAL_ESP_4TO_CARACTER,
	EST_MEF_PRINCIPAL_ESP_FINAL_TRAMA,
	EST_MEF_PRINCIPAL_EJECUTANDO,
}estMefPrincipal_enum;

typedef enum{
	X = 0,
	Y,
	Z,
}Axes_enum;

void sendDataAxes(Axes_enum ax){

	char str[12] = ":AccX:S:000\n";

	NVIC_EnableIRQ(PORTC_PORTD_IRQn); //activa las interrupciones del acelerometro (puerto C y D)

	if(ax == X){
		Acc = mma8451_getAcX(); //almacena los datos de la aceleracion en el eje X
		str[4] = 'X';
	}else if (ax == Y){
		Acc = mma8451_getAcY(); //almacena los datos de la aceleracion en el eje Y
		str[4] = 'Y';
	}else if (ax == Z){
		Acc = mma8451_getAcZ(); //almacena los datos de la aceleracion en el eje Z
		str[4] = 'Z';
	}

	NVIC_DisableIRQ(PORTC_PORTD_IRQn); //desactiva las interrupciones del acelerometro

	if(Acc < 0){
		str[6] = 'N';
		Acc = -Acc;
	}else{
		str[6] = 'P';
	}

	str[8] = Acc/100 + 48;
	str[9] = (float)(Acc%100) / 10 + 48;
	str[10] = Acc%10 + 48;

	uart0_ringBuffer_envDatos(str, sizeof(str)); //carga el buffer de respuesta en el ringbuffer y se envia por la UART0

}

void mefPrincipal(void){

	static estMefPrincipal_enum estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;

	switch(estMefPrincipal){

	case EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA:
		if(buffer[0]==INICIO_DE_TRAMA){
			index_bytes = 0;
			buffer_trama[index_bytes] = buffer[0]; //guardo lo que llegó.
			index_bytes++;//incrementa en indice que marca en que posicion del buffer_resp se debe ingresar el byte luego de ser verificado
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_1ER_CARACTER;
		}
		break;

	case EST_MEF_PRINCIPAL_ESP_1ER_CARACTER:
		if(buffer[0]=='A'){
			buffer_trama[index_bytes] = buffer[0]; //guardo lo que llegó.
			index_bytes++;
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_2DO_CARACTER;
		}else{
			clear_buffer(buffer_trama);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
		}
		break;

	case EST_MEF_PRINCIPAL_ESP_2DO_CARACTER:
		if((buffer[0]=='c')||(buffer[0]=='x')){
			buffer_trama[index_bytes] = buffer[0]; //guardo lo que llegó.
			index_bytes++;
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_3ER_CARACTER;
		}else{
			clear_buffer(buffer_trama);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
		}
		break;

	case EST_MEF_PRINCIPAL_ESP_3ER_CARACTER:

		if((buffer[0]=='c')||(buffer[0]=='e')){
			buffer_trama[index_bytes] = buffer[0]; //guardo lo que llegó.
			index_bytes++;
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_4TO_CARACTER;
		}else{
			clear_buffer(buffer_trama);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
		}
		break;

	case EST_MEF_PRINCIPAL_ESP_4TO_CARACTER:

		if((buffer[0]=='X')||(buffer[0]=='Y')||(buffer[0]=='Z')||(buffer[0]=='s')){
			buffer_trama[index_bytes] = buffer[0]; //guardo lo que llegó.
			index_bytes++;
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_FINAL_TRAMA;
		}else{
			clear_buffer(buffer_trama);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
		}
		break;

	case EST_MEF_PRINCIPAL_ESP_FINAL_TRAMA:
		if(buffer[0]==FIN_DE_TRAMA){
			buffer_trama[index_bytes] = buffer[0]; //guardo lo que llegó.
			fin_trama = true;
			estMefPrincipal = EST_MEF_PRINCIPAL_EJECUTANDO;
		}else{
			clear_buffer(buffer_trama);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
		}
		break;

	case EST_MEF_PRINCIPAL_EJECUTANDO:

		fin_trama = false;
		/*
		 * Si bien durante la recepcion de los bytes se procedio a la verificacion de los datos ingresados,
		 * no se discriminó de tal manera que llegado a este punto de la mef se sabe que la trama es correcta pero
		 * no se sabe a cual corresponde, con lo cual ahora se procede a comparar el buffer_resp con cada instruccion
		 * posible mediante la funcion strcmp().
		 *
		 * Puede discutirse acerca de este modo de proceder en cuanto al tiempo de procesamiento requerido para casos
		 * donde las tramas son mas extensas. No obstante se eligió trabajar de esta manera dado que las tramas (segun el enunciado)
		 * don de tamaño muy reducido. Sin dudas que si fuesen tramas mucho mas extensas se hubiese optado por otra resolucion (pero no es el caso).
		 */

		if(strcmp(buffer_trama, ":AccX\n")==0){//si se recibió esa instruccion entonces se debe enviar al maestro el valor de aceleracion (con el formato indicado en el enunciado)

			sendDataAxes(X);

			clear_buffer(buffer_trama);//limpia el buffer donde se almacena la instruccion o trama
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
			break;
		}

		if(strcmp(buffer_trama, ":AccY\n")==0){//si se recibió esa instruccion entonces se debe enviar al maestro el valor de aceleracion (con el formato indicado en el enunciado)

			sendDataAxes(Y);

			clear_buffer(buffer_trama);//limpia el buffer donde se almacena la instruccion o trama
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
			break;
		}

		if(strcmp(buffer_trama, ":AccZ\n")==0){//si se recibió esa instruccion entonces se debe enviar al maestro el valor de aceleracion (con el formato indicado en el enunciado)

			sendDataAxes(Z);

			clear_buffer(buffer_trama);//limpia el buffer donde se almacena la instruccion o trama
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
			break;
		}

		if(strcmp(buffer_trama, ":Axes\n")==0){//si se recibió esa instruccion entonces se debe enviar al maestro el valor de aceleracion (con el formato indicado en el enunciado)


			clear_buffer(buffer_trama);//limpia el buffer donde se almacena la instruccion o trama
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
			break;
		}

		clear_buffer(buffer_trama);//limpia el buffer donde se almacena la instruccion o trama
		estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
		break;
	}
}

/*==================[end of file]============================================*/


