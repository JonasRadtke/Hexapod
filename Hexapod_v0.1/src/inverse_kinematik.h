/*
 * inverse_kinematik.h
 *
 * Created: 12.03.2016 16:49:08
 *  Author: Jona Radtke
 */ 

//#include <main.h>

struct servowinkel
{
	float A1_Winkel;
	float A2_Winkel;
	float B1_Winkel;
	float B2_Winkel;
	float C1_Winkel;
	float C2_Winkel;
} volatile servowinkel_1;

extern float basispunkte[6][3] = {
	{  27.5000000000000,	-84.1784025900000,	-170.232820844743 },
	{ -27.5000000000000,	-84.1784025900000,	-170.232820844743 },
	{ -86.6506350929338,	 18.2735026909279,	-170.232820844743 },
	{ -59.1506350929338,	 65.9048998990720,	-170.232820844743 },
	{  59.1506350929338,	 65.9048998990720,	-170.232820844743 },
	{  86.6506350929338,	 18.2735026909279,	-170.232820844743 },
};
extern float platformpunkte[6][3] ={
	{  54.7762794444011,	-38.5533029127066,	 0 },
	{ -54.7762794444011,	-38.5533029127066,	 0 },
	{ -60.7762794444011,	-28.1609980672934,	 0 },
	{ -6,					 66.7143009800000, 	 0 },
	{  6,					 66.7143009800000,	 0 },
	{  60.7762794444011,	-28.1609980672934,	 0 },
};

extern float transformiertepunkte[6][3];
extern float servo_transformiertepunkte[6][3];

#define LAENGE		179
#define LAENGE2		LAENGE*LAENGE
#define R			19
#define R2			R*R




void drehung_x(float x[],float y[],float winkel);
void drehung_y(float x[],float y[],float winkel);
void drehung_z(float x[],float y[],float winkel);
void drehung_anwenden(float o[][3], float t[][3], float winkel_x, float winkel_y, float winkel_z);