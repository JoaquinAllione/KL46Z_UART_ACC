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

#define COVID19

#define FIN_DE_TRAMA (0x0A) //caracter de fin de trama
#define INICIO_DE_TRAMA 58 //caracter de inicio de trama
#define NUM_GRUPO 53
#define LENGTH_BUFFER 20

#ifdef COVID19
/* ======== RESOLUCION CON UART 0 ======== */
uint8_t buffer[3];
uint8_t buffer_resp[LENGTH_BUFFER]; //buffer que almacena la instruccion recibida
int8_t index_bytes; //indice que marca en que posicion del buffer (arreglo de char) se debe escribir el proximo byte
bool dato_recibido = false;

// ===============//
//Para lectura del Acelerometro...
int16_t Acc_X;
int16_t Acc_Y;
int16_t Acc_Z;
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

void insertar(uint8_t *pbuf, uint8_t caracter_a_insertar, int8_t pos_insertar){

	uint8_t aux;
	aux = pbuf[pos_insertar];
	pbuf[pos_insertar] = caracter_a_insertar;
	pbuf[pos_insertar+1] = aux;

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

void mefPrincipal(void){

	static estMefPrincipal_enum estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;

	switch(estMefPrincipal){

	case EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA:
		if(buffer[0]==INICIO_DE_TRAMA){
			index_bytes = 0;
			buffer_resp[index_bytes] = buffer[0]; //guardo lo que llegó.
			index_bytes++;//incrementa en indice que marca en que posicion del buffer_resp se debe ingresar el byte luego de ser verificado
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_1ER_CARACTER;
		}
		break;

	case EST_MEF_PRINCIPAL_ESP_1ER_CARACTER:
		if(buffer[0]=='A'){
			buffer_resp[index_bytes] = buffer[0]; //guardo lo que llegó.
			index_bytes++;
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_2DO_CARACTER;
		}else{
			clear_buffer(buffer_resp);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
		}
		break;

	case EST_MEF_PRINCIPAL_ESP_2DO_CARACTER:
		if(buffer[0]=='c'){
			buffer_resp[index_bytes] = buffer[0]; //guardo lo que llegó.
			index_bytes++;
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_3ER_CARACTER;
		}else{
			clear_buffer(buffer_resp);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
		}
		break;

	case EST_MEF_PRINCIPAL_ESP_3ER_CARACTER:

		if(buffer[0]=='c'){
			buffer_resp[index_bytes] = buffer[0]; //guardo lo que llegó.
			index_bytes++;
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_4TO_CARACTER;
		}else{
			clear_buffer(buffer_resp);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
		}
		break;

	case EST_MEF_PRINCIPAL_ESP_4TO_CARACTER:

		if((buffer[0]=='X')||(buffer[0]=='Y')||(buffer[0]=='Z')||(buffer[0]=='2')){
			buffer_resp[index_bytes] = buffer[0]; //guardo lo que llegó.
			index_bytes++;
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_FINAL_TRAMA;
		}else{
			clear_buffer(buffer_resp);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
		}
		break;

	case EST_MEF_PRINCIPAL_ESP_FINAL_TRAMA:
		if(buffer[0]==FIN_DE_TRAMA){
			buffer_resp[index_bytes] = buffer[0]; //guardo lo que llegó.
			fin_trama = true;
			estMefPrincipal = EST_MEF_PRINCIPAL_EJECUTANDO;
		}else{
			clear_buffer(buffer_resp);
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

		if(strcmp(buffer_resp, ":AccX\n")==0){//si se recibió esa instruccion entonces se debe enviar al maestro el valor de aceleracion (con el formato indicado en el enunciado)
			NVIC_EnableIRQ(PORTC_PORTD_IRQn); //activa las interrupciones del acelerometro (puerto C y D)
			Acc_X = abs(mma8451_getAcX()); //almacena los datos de la aceleracion en el eje X
			NVIC_DisableIRQ(PORTC_PORTD_IRQn); //desactiva las interrupciones del acelerometro
			insertar(buffer_resp, Acc_X/100 + 48 , 5); //se pone +48 para pasar de numero entero a caracter segun tabla ASCII
			insertar(buffer_resp, (float)(Acc_X%100) / 10 + 48 , 6);
			insertar(buffer_resp, Acc_X%10 + 48 , 7);
			uart0_ringBuffer_envDatos(buffer_resp, sizeof(buffer_resp)); //carga el buffer de respuesta en el ringbuffer y se envia por la UART0
			clear_buffer(buffer_resp);//limpia el buffer donde se almacena la instruccion o trama
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
			break;
		}

		if(strcmp(buffer_resp, ":AccY\n")==0){//si se recibió esa instruccion entonces se debe enviar al maestro el valor de aceleracion (con el formato indicado en el enunciado)
			NVIC_EnableIRQ(PORTC_PORTD_IRQn); //activa las interrupciones del acelerometro (puerto C y D)
			Acc_Y = abs(mma8451_getAcY()); //almacena los datos de la aceleracion en el eje X
			NVIC_DisableIRQ(PORTC_PORTD_IRQn); //desactiva las interrupciones del acelerometro
			insertar(buffer_resp, Acc_Y/100 + 48 , 5); //se pone +48 para pasar de numero entero a caracter segun tabla ASCII
			insertar(buffer_resp, (float)(Acc_Y%100) / 10 + 48 , 6);
			insertar(buffer_resp, Acc_Y%10 + 48 , 7);
			uart0_ringBuffer_envDatos(buffer_resp, sizeof(buffer_resp)); //carga el buffer de respuesta en el ringbuffer y se envia por la UART0
			clear_buffer(buffer_resp);//limpia el buffer donde se almacena la instruccion o trama
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
			break;
		}

		if(strcmp(buffer_resp, ":AccZ\n")==0){//si se recibió esa instruccion entonces se debe enviar al maestro el valor de aceleracion (con el formato indicado en el enunciado)
			NVIC_EnableIRQ(PORTC_PORTD_IRQn); //activa las interrupciones del acelerometro (puerto C y D)
			Acc_Z = abs(mma8451_getAcZ()); //almacena los datos de la aceleracion en el eje X
			NVIC_DisableIRQ(PORTC_PORTD_IRQn); //desactiva las interrupciones del acelerometro
			insertar(buffer_resp, Acc_Z/100 + 48 , 5); //se pone +48 para pasar de numero entero a caracter segun tabla ASCII
			insertar(buffer_resp, (float)(Acc_Z%100) / 10 + 48 , 6);
			insertar(buffer_resp, Acc_Z%10 + 48 , 7);
			uart0_ringBuffer_envDatos(buffer_resp, sizeof(buffer_resp)); //carga el buffer de respuesta en el ringbuffer y se envia por la UART0
			clear_buffer(buffer_resp);//limpia el buffer donde se almacena la instruccion o trama
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
			break;
		}

		if(strcmp(buffer_resp, ":Acc2\n")==0){//si se recibió esa instruccion entonces se debe enviar al maestro el valor de aceleracion (con el formato indicado en el enunciado)
			NVIC_EnableIRQ(PORTC_PORTD_IRQn); //activa las interrupciones del acelerometro (puerto C y D)
			Acc_X = mma8451_getAcX(); //almacena los datos de la aceleracion en el eje X
			Acc_Y = mma8451_getAcY(); //almacena los datos de la aceleracion en el eje Y
			Acc_Z = mma8451_getAcZ();	//almacena los datos de la aceleracion en el eje Z
			NVIC_DisableIRQ(PORTC_PORTD_IRQn); //desactiva las interrupciones del acelerometro
			Acc = sqrt(pow(Acc_X,2)+pow(Acc_Y,2)+pow(Acc_Z,2)); //realiza el calculo del modulo de la aceleracion
			//para descomponer el valor numerico resultante (almacenado en Acc) en unidad, decena y centena
			//y luego cargarlos individualmente en el buffer de respuesta se realizan las operaciones que se muentran
			//en el segundo de los argumentos de la funcion insertar()
			insertar(buffer_resp, Acc/100 + 48 , 5); //se pone +48 para pasar de numero entero a caracter segun tabla ASCII
			insertar(buffer_resp, (float)(Acc%100) / 10 + 48 , 6);
			insertar(buffer_resp, Acc%10 + 48 , 7);
			uart0_ringBuffer_envDatos(buffer_resp, sizeof(buffer_resp)); //carga el buffer de respuesta en el ringbuffer y se envia por la UART0
			clear_buffer(buffer_resp);//limpia el buffer donde se almacena la instruccion o trama
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
			break;
		}

		clear_buffer(buffer_resp);//limpia el buffer donde se almacena la instruccion o trama
		estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
		break;
	}
}

#else

/*
 * Aqui se plantea la implementacion para el uso de la UART1.
 * En este caso se tiene un codigo muy similar al anterior cambiando (claro), las configuraciones e iniciaciones
 * que la UART1 requiere para su funcionamiento. Así como tambien el uso adicional de un PIN (PORTA 16) para activar
 * el envio o recepcion de datos (ya que se trata de una comunicacion half-duplex).
 *
 * Las concideraciones y aclaraciones respecto al funcionamiento de las funciones descriptas en la implementacion
 * con UART0 continuan en vigencia y no se detallan aqui (ya que fueron antes detalladas).
 */
#define RECEIVER_ENABLE board_setLed(BOARD_UART1_CONTROL, BOARD_LED_MSG_ON)
#define DRIVER_ENABLE board_setLed(BOARD_UART1_CONTROL, BOARD_LED_MSG_OFF)
/* ======== RESOLUCION CON UART 1 ======== */
uint8_t buffer[3];
uint8_t buffer_resp[LENGTH_BUFFER]; //buffer que almacena la instruccion recibida
int8_t index_bytes; //indice que marca en que posicion del buffer (arreglo de char) se debe escribir el proximo byte
bool dato_recibido = false;
// ===============//

//Para lectura del Acelerometro...
int16_t Acc_X;
int16_t Acc_Y;
int16_t Acc_Z;
int16_t Acc;

void clear_buffer(uint8_t *pBuf){ //funcion para limpiar el buffer donde se almacenan las instrucciones
	int8_t i;                     //así se permite que se deje el buffer listo para ser usado para un proximo arribo de instruccion
	for(i=0;i<LENGTH_BUFFER;i++){
		pBuf[i] = 0;
	}
}

void mefPrincipal(void);

bool fin_trama = false;

void main(void)
{

	BOARD_BootClockRUN();

	// Se inicializan funciones de la placa
	board_init();
	/* ============= I2C ================= */
	SD2_I2C_init();
	/* =========== MMA8451 =============== */
	mma8451_init(); //el acelerometro se configura pero no se habilita (arranca desactivado)
	mma8451_setDataRate(DR_50hz);
	NVIC_DisableIRQ(PORTC_PORTD_IRQn);
	/* ============ UART1 ================ */
	uart1_ringBuffer_init();

	clear_buffer(buffer);

	board_setLed(BOARD_LED_ID_VERDE, BOARD_LED_MSG_OFF);
	board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_OFF);
	RECEIVER_ENABLE;

	while(1)
	{
	    dato_recibido = uart1_ringBuffer_recDatos(buffer, sizeof(buffer));
	    if((dato_recibido == true)||(fin_trama == true)){
	    	mefPrincipal();
	    }
	}

}

void insertar(uint8_t *pbuf, uint8_t caracter_a_insertar, int8_t pos_insertar){
	uint8_t aux;
	aux = pbuf[pos_insertar];
	pbuf[pos_insertar] = caracter_a_insertar;
	pbuf[pos_insertar+1] = aux;
}

typedef enum{
	EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA = 0,
	EST_MEF_PRINCIPAL_ESP_NUM_GRUPO_1ER_DIGITO,
	EST_MEF_PRINCIPAL_ESP_NUM_GRUPO_2DO_DIGITO,
	EST_MEF_PRINCIPAL_ESP_INSTRUC_1ER_DIGITO,
	EST_MEF_PRINCIPAL_ESP_INSTRUC_2DO_DIGITO,
	EST_MEF_PRINCIPAL_ESP_ACCION_LED,
	EST_MEF_PRINCIPAL_ESP_FINAL_TRAMA,
	EST_MEF_PRINCIPAL_EJECUTANDO,
}estMefPrincipal_enum;

void mefPrincipal(void){

	static estMefPrincipal_enum estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;

	switch(estMefPrincipal){

	case EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA:
		if(buffer[0]==INICIO_DE_TRAMA){
			board_setLed(BOARD_LED_ID_VERDE, BOARD_LED_MSG_TOGGLE);
			index_bytes = 0;
			buffer_resp[index_bytes] = buffer[0]; //guardo lo que llegó.
			index_bytes++;
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_NUM_GRUPO_1ER_DIGITO;
		}else{
			clear_buffer(buffer_resp);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
		}
		break;

	case EST_MEF_PRINCIPAL_ESP_NUM_GRUPO_1ER_DIGITO:
		if(buffer[0]=='0'){
			board_setLed(BOARD_LED_ID_VERDE, BOARD_LED_MSG_TOGGLE);
			buffer_resp[index_bytes] = buffer[0]; //guardo lo que llegó.
			index_bytes++;
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_NUM_GRUPO_2DO_DIGITO;
		}else{
			clear_buffer(buffer_resp);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
		}
		break;

	case EST_MEF_PRINCIPAL_ESP_NUM_GRUPO_2DO_DIGITO:
		if(buffer[0]==NUM_GRUPO){
			buffer_resp[index_bytes] = buffer[0]; //guardo lo que llegó.
			index_bytes++;
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_INSTRUC_1ER_DIGITO;
		}else{
			clear_buffer(buffer_resp);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
		}
		break;

	case EST_MEF_PRINCIPAL_ESP_INSTRUC_1ER_DIGITO:

		if((buffer[0]=='0')||(buffer[0]=='1')||(buffer[0]=='2')){
			buffer_resp[index_bytes] = buffer[0]; //guardo lo que llegó.
			index_bytes++;
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_INSTRUC_2DO_DIGITO;
		}else{
			clear_buffer(buffer_resp);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
		}
		break;

	case EST_MEF_PRINCIPAL_ESP_INSTRUC_2DO_DIGITO:

		if((buffer[0]=='0')||(buffer[0]=='1')||(buffer[0]=='2')||(buffer[0]=='3')){
			buffer_resp[index_bytes] = buffer[0]; //guardo lo que llegó.
			if(buffer_resp[index_bytes-1]!='0'){
				estMefPrincipal = EST_MEF_PRINCIPAL_ESP_FINAL_TRAMA;
			}else{
				estMefPrincipal = EST_MEF_PRINCIPAL_ESP_ACCION_LED;
			}
			index_bytes++;
		}else{
			clear_buffer(buffer_resp);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
		}
		break;

	case EST_MEF_PRINCIPAL_ESP_ACCION_LED:

		if((buffer[0]=='A')||(buffer[0]=='E')||(buffer[0]=='T')){
			buffer_resp[index_bytes] = buffer[0]; //guardo lo que llegó.
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_FINAL_TRAMA;
			index_bytes++;
		}else{
			clear_buffer(buffer_resp);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
		}
		break;

	case EST_MEF_PRINCIPAL_ESP_FINAL_TRAMA:
		if(buffer[0]==FIN_DE_TRAMA){
			buffer_resp[index_bytes] = buffer[0]; //guardo lo que llegó.
			fin_trama = true;
			estMefPrincipal = EST_MEF_PRINCIPAL_EJECUTANDO;
		}else{
			clear_buffer(buffer_resp);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
		}
		break;

	case EST_MEF_PRINCIPAL_EJECUTANDO:

		fin_trama = false;

		if(strcmp(buffer_resp, ":0501E\n")==0){
			board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_ON);
			DRIVER_ENABLE;
			uart1_ringBuffer_envDatos(buffer_resp, sizeof(buffer_resp));
			RECEIVER_ENABLE;
			clear_buffer(buffer_resp);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
			break;
		}
		if(strcmp(buffer_resp, ":0501A\n")==0){
			board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_OFF);
			DRIVER_ENABLE;
			uart1_ringBuffer_envDatos(buffer_resp, sizeof(buffer_resp));
			RECEIVER_ENABLE;
			clear_buffer(buffer_resp);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
			break;
		}
		if(strcmp(buffer_resp, ":0501T\n")==0){
			board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_TOGGLE);
			DRIVER_ENABLE;
			uart1_ringBuffer_envDatos(buffer_resp, sizeof(buffer_resp));
			RECEIVER_ENABLE;
			clear_buffer(buffer_resp);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
			break;
		}
		if(strcmp(buffer_resp, ":0502E\n")==0){
			board_setLed(BOARD_LED_ID_VERDE, BOARD_LED_MSG_ON);
			DRIVER_ENABLE;
			uart1_ringBuffer_envDatos(buffer_resp, sizeof(buffer_resp));
			RECEIVER_ENABLE;
			clear_buffer(buffer_resp);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
			break;
		}
		if(strcmp(buffer_resp, ":0502A\n")==0){
			board_setLed(BOARD_LED_ID_VERDE, BOARD_LED_MSG_OFF);
			DRIVER_ENABLE;
			uart1_ringBuffer_envDatos(buffer_resp, sizeof(buffer_resp));
			RECEIVER_ENABLE;
			clear_buffer(buffer_resp);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
			break;
		}
		if(strcmp(buffer_resp, ":0502T\n")==0){
			board_setLed(BOARD_LED_ID_VERDE, BOARD_LED_MSG_TOGGLE);
			DRIVER_ENABLE;
			uart1_ringBuffer_envDatos(buffer_resp, sizeof(buffer_resp));
			RECEIVER_ENABLE;
			clear_buffer(buffer_resp);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
			break;
		}
		if(strcmp(buffer_resp, ":0511\n")==0){
			if(board_getSw(BOARD_SW_ID_1)){
				insertar(buffer_resp, 'P', 5);
			}else{
				insertar(buffer_resp, 'N', 5);
			}
			DRIVER_ENABLE;
			uart1_ringBuffer_envDatos(buffer_resp, sizeof(buffer_resp));
			RECEIVER_ENABLE;
			clear_buffer(buffer_resp);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
			break;
		}
		if(strcmp(buffer_resp, ":0513\n")==0){
			if(board_getSw(BOARD_SW_ID_3)){
				insertar(buffer_resp, 'P', 5);
			}else{
				insertar(buffer_resp, 'N', 5);
			}
			DRIVER_ENABLE;
			uart1_ringBuffer_envDatos(buffer_resp, sizeof(buffer_resp));
			RECEIVER_ENABLE;
			clear_buffer(buffer_resp);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
			break;
		}
		if(strcmp(buffer_resp, ":0520\n")==0){
			NVIC_EnableIRQ(PORTC_PORTD_IRQn);
			Acc_X = mma8451_getAcX();
			Acc_Y = mma8451_getAcY();
			Acc_Z = mma8451_getAcZ();
			NVIC_DisableIRQ(PORTC_PORTD_IRQn);
			Acc = sqrt(pow(Acc_X,2)+pow(Acc_Y,2)+pow(Acc_Z,2));
			insertar(buffer_resp, Acc/100 + 48 , 5);
			insertar(buffer_resp, (float)(Acc%100) / 10 + 48 , 6);
			insertar(buffer_resp, Acc%10 + 48 , 7);
			DRIVER_ENABLE;
			uart1_ringBuffer_envDatos(buffer_resp, sizeof(buffer_resp));
			RECEIVER_ENABLE;
			clear_buffer(buffer_resp);
			estMefPrincipal = EST_MEF_PRINCIPAL_ESP_COMIENZO_TRAMA;
		}
		break;
	}
}

#endif
/*==================[end of file]============================================*/


