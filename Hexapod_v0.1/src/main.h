/*
 * funktionheader.h
 *
 * Created: 05.08.2015 19:15:52
 *  Author: flyingsee
 */ 

#include <asf.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <nunchuck.h>


#define seriell				UART1
#define serielltxd			PIO_PB3A_UTXD1
#define seriellrxd			PIO_PB2A_URXD1
#define seriell_IRQ			UART1_IRQn

#define twiwaitus			50					// us wartezeit auf TWI

typedef struct __attribute__((__packed__)) datensat {
	char gang[7];
	char datens[8];
	float p_max;
	uint8_t p_fest;
	uint8_t druecke[33];
} datensatz;


#define LAENGE		(179)
#define LAENGE2		(32041.000000)
#define R			(19.5)
#define R2			(380.25)
#define RAD	    	(0.017453293)
#define GRAD		(57.29577951)
#define COSPM120	(-0.5)
#define SIN120		(0.866025404)
#define SINM120		(-0.866025404)

void twi_init(void);

void uartsendbyte (uint8_t c);
void uartsendstring(char s[]);

char get_menuebefehl(char rx[]);
char get_datensatzbefehl(uint16_t daten[], char rx[]);
char get_uebersetzbefehl(float daten[], char rx[]);
char get_samplebefehl(uint16_t daten[], char rx[]);

void print_menu(void);
void print_nunchuch_daten(void);




void drehung_x(float x[], float y[],float winkel);
void drehung_y(float x[],float y[],float winkel);
void drehung_z(float x[],float y[],float winkel);
void drehung_anwenden(float o[][3], float t[][3], float winkel_x, float winkel_y, float winkel_z);
void verschiebung_anwenden(float o[][3], float t[][3], float x, float y, float z);
void systemtransformation(float o[][3], float ta[][3], float t[][3]);
void stellwinkel(float ta[][3], float a[]);
void drehung_z_minus_120(float x[],float y[]);
void drehung_z_plus_120(float x[],float y[]);

void touchkoord(void);
void print_xy(void);
void print_xy_raw(void);

void regler_x(void);
void beobachter_x(void);
void regler_y(void);
void beobachter_y(void);