/*
 * nunchuck.h
 *
 * Created: 11.03.2016 22:12:42
 *  Author: Jona Radtke
 */ 

struct nunchuckdaten
{
	uint8_t x_stick;
	uint8_t y_stick;
	uint16_t x_beschl;
	uint16_t y_beschl;
	uint16_t z_beschl;
	bool c_button;
	bool z_button;
}nunchuck;


void receive_nundhuck(void);
void nunchuck_init(void);
