/*
 * nunchuck.c
 *
 * Created: 11.03.2016 22:09:04
 *  Author: Jona Radtke
 */ 

// Nunchuck Daten auslesen

#include <asf.h>
#include <stdio.h>
#include <stdint.h>
#include <nunchuck.h>



void receive_nundhuck(void){
	uint8_t daten[6];
	uint8_t twiBuffer[2];
	twiBuffer[0] = 0x00; twiBuffer[1] = 0x00;
	twi_package_t packet_write2 = {
		.addr         = 0x00,      // TWI slave memory address data
		.addr_length  = sizeof (uint8_t),    // TWI slave memory address data size
		.chip         = 0x52,      // TWI slave bus address
		.buffer       = (void *)twiBuffer, // transfer data source buffer
		.length       = 1  // transfer data size (bytes)
	};
	twi_master_write(TWI2, &packet_write2);
	delay_us(300);
	twi_package_t packet_read = {
		.addr         = 0x00,      // TWI slave memory address data
		.addr_length  = 0, //keine internal adresse, nur lesen
		.chip         = 0x52,      // TWI slave bus address
		.buffer       = daten, // transfer data source buffer
		.length       = 6  // transfer data size (bytes)
	};
	twi_master_read(TWI2, &packet_read);
	//Daten entschluesseln
	uint8_t k;
	for (k=0;k<7;k++)
	{
		daten[k] = (daten[k] ^ 0x17) + 0x17; //Byte Entschluesseln
	}
	// Daten aufbereiten
	nunchuck.x_stick = daten[0];
	nunchuck.y_stick = daten[1];
	nunchuck.x_beschl = ((uint16_t)daten[2] << 2) | (daten[5]&0x0c >> 2); //10bit
	nunchuck.y_beschl = ((uint16_t)daten[3] << 2) | (daten[5]&0x30 >> 4);
	nunchuck.z_beschl = ((uint16_t)daten[4] << 2) | (daten[5]&0xc0 >> 6);
	nunchuck.c_button = daten[5]&0x02;
	nunchuck.z_button = daten[5]&0x01;
}

//Nunchuck initalisieren
void nunchuck_init(void){

	uint8_t twiBuffer[2];
	twiBuffer[0] = 0x00; twiBuffer[1] = 0x00;
	twi_package_t packet_write = {
		.addr         = 0x40,      // TWI slave memory address data
		.addr_length  = sizeof (uint8_t),    // TWI slave memory address data size
		.chip         = 0x52,      // TWI slave bus address
		.buffer       = (void *)twiBuffer, // transfer data source buffer
		.length       = 1  // transfer data size (bytes)
	};
	twi_master_write(TWI2, &packet_write);
}

