/**
 * \file
 *
 * \brief Empty user application template
 *
 */

#include <main.h>

#define twiwaitus			50	
#define SERVO_A1			PIO_PA11
#define SERVO_A2			PIO_PA12
#define SERVO_B1			PIO_PA0
#define SERVO_B2			PIO_PA3
#define SERVO_C1			PIO_PA4
#define SERVO_C2			PIO_PA13

#define UR					PIO_PA18
#define OR					PIO_PA17	
#define OL					PIO_PA20
#define TOUCH				PIO_PA19

#define A			(-1.10943806957025)
#define B			(0.707166047382547)
#define C			( 452.400030586869)
#define D			(-0.823791315178334)
#define E			(0.525139409327861)
#define F			( 337.674523223194)

#define TA			(0.03)
#define TA2			(TA*TA)
#define TV			(1.5)
#define TN			(5)
#define TR			(0.05*TV)
#define KP			(2.3)

#define TA_MESS		(0.01)
#define T_MESS		(0.1)

#define  SERVO_A1_OBEN	 83   //-90°
#define  SERVO_A1_MITTE	 155  // 0
#define  SERVO_A1_UNTEN  210  // +68,75

#define  SERVO_A2_OBEN	 210	//75
#define  SERVO_A2_MITTE	 150	//
#define  SERVO_A2_UNTEN  85		//-81

#define  SERVO_B1_OBEN	 83		//-90
#define  SERVO_B1_MITTE  158	//0
#define  SERVO_B1_UNTEN  210	//63,75

#define  SERVO_B2_OBEN	 210	//75
#define  SERVO_B2_MITTE  150	//0
#define  SERVO_B2_UNTEN  85		//-65

#define  SERVO_C1_OBEN	 83		//-81,25
#define  SERVO_C1_MITTE  148	//0
#define  SERVO_C1_UNTEN  210	//77,5

#define  SERVO_C2_OBEN	 210	//73,75
#define  SERVO_C2_MITTE  151	//0	
#define  SERVO_C2_UNTEN  85		//-82,5

#define	 NUN_X_MITTE	 127
#define	 NUN_X_RECHTS	 0xe3
#define	 NUN_X_LINKS	 0x1e

#define	 NUN_Y_MITTE	 129
#define	 NUN_Y_OBEN		 0xe1
#define	 NUN_Y_UNTEN	 0x26

#define	 NUN_XBS_MITTE	 512
#define	 NUN_XBS_RECHTS	 0x2d3
#define	 NUN_XBS_LINKS	 0x13b

#define	 NUN_YBS_MITTE	 455
#define	 NUN_YBS_RECHTS	 0x173
#define	 NUN_YBS_LINKS	 0x28f

// Aufhaengepunkte auf der Platform
// Punkt ist jeweils über die Stange mit dem Arm verbunden

float basispunkte[6][3] = {
	{  27.50000000000,	-87.67840259000,	-165,649341269548 },
	{ -27.50000000000,	-87.67840259000,	-165,649341269548 },
	{ -89.68172400617,	20.02350269092,	-165,649341269548 },
	{ -62.18172400617,	67.65489989907,	-165,649341269548 },
	{  62.18172400617,	67.65489989907,	-165,649341269548 },
	{  89.68172400617,	20.02350269092,	-165,649341269548 },
};

// 0,0,0 Mitte Platform
float platformpunkte[6][3] ={
	{  54.7762794444011,	-38.5533029127066,	 0 },
	{ -54.7762794444011,	-38.5533029127066,	 0 },
	{ -60.7762794444011,	-28.1609980672934,	 0 },
	{ -6,					 66.7143009800000, 	 0 },
	{  6,					 66.7143009800000,	 0 },
	{  60.7762794444011,	-28.1609980672934,	 0 },
};

float transformiertepunkte[6][3];
float servo_transformiertepunkte[6][3];
float winkel_vor[6];
uint8_t winkel_aktuell[6] = {0};
uint8_t winkel_aktuell_neu[6] = {0};
float temp3[6][3];
volatile float x_rotation = 0;
volatile float y_rotation = 0;
volatile float z_rotation = 0;
volatile float x_achse = 0;
volatile float y_achse = 0;
volatile float hoehe = 0;
	
// UART Variablen
uint8_t uartrxbuffer[250] = {0};
char eingabebuffer[250] = {0};
volatile uint8_t uartindex = 0;

// Servo 
volatile uint32_t aktuellerwert = 0;
volatile uint32_t ticks = 0;
volatile uint32_t servoticks = 0;

//TOUCH
volatile uint32_t x_adc = 0;
volatile uint32_t y_adc = 0;
volatile float x_koord = 0.14;
volatile float y_koord = 0.12;
uint32_t touchtime = 0;
uint32_t reglertime = 0;
float lastfive_x[10];
float lastfive_y[10];

//Regler x;
float uk_x = 0;
float uk_1_x = 0;
float uk_2_x = 0;

float yk_x = 0;
float yk_1_x = 0;
float yk_2_x = 0;

float soll_x = 0.14;

float ykb_x = 0;

//Regler y;
float uk_y = 0;
float uk_1_y = 0;
float uk_2_y = 0;

float yk_y = 0;
float yk_1_y = 0;
float yk_2_y = 0;

float soll_y = 0.12;

float ykb_y = 0;


// Beobachter x
float y1_x = 0;
float y1_1_x = 0;

float y2_x = 0;
float y2_1_x = 0;

float yb_x = 0;

// Beobachter y
float y1_y = 0;
float y1_1_y = 0;

float y2_y = 0;
float y2_1_y = 0;

float yb_y = 0;

uint32_t ballfromplatetime = 0;

// Messen
float x_mess_alt = 0;
float y_mess_alt = 0;


uint32_t wt = 0;
uint32_t test;

uint32_t rc;

// Modus Koordinaten
uint8_t modus_wahl = 1;
uint16_t mod_k = 0; 
float mod_mitte[1][2] = {0.14, 0.12};

uint32_t quadrat_time = 0;
float mod_quadrat[4][2] ={
	{	0.18,		0.07	},
	{	0.06,	0.07	},
	{	0.1053651357,	0.14535682		},
	{	0.17488341,		0.14535682		},
};

int main (void)
{
	
	sysclk_init();
	wdt_disable(WDT);	// Watchdog ausschalten
	SystemCoreClockUpdate();	// Systemclock akualisieren
	fpu_enable ();	
	delay_init(SystemCoreClock);
	//SysTick_Config(SystemCoreClock / 1000);
	
	SysTick_Config(SystemCoreClock / 1000);      /* Configure SysTick to generate an interrupt every millisecond */



	pmc_enable_periph_clk(ID_PIOA);	
	pmc_enable_periph_clk(ID_PIOB);
	pio_set_output 	( 	PIOA, SERVO_A1 | SERVO_A2 | SERVO_B1 | SERVO_B2 | SERVO_C1 | SERVO_C2 | UR | OR | UR ,LOW,DISABLE,DISABLE);
	
	// Servo Timer initalisieren!
	sysclk_enable_peripheral_clock(ID_TC0); // Clock auf Timer 0
	tc_init(TC, 0,
	TC_CMR_TCCLKS_TIMER_CLOCK1		// Waveform Clock Selection
	| TC_CMR_WAVE					// Waveform mode
	| TC_CMR_WAVSEL_UP_RC			// Bei RC zurücksetzen
	);
	tc_enable_interrupt(TC, 0, TC_IER_CPCS); // Compare RC Interrupt einschalten
	rc = (SystemCoreClock /2 /100000);	// 100khz
	tc_write_rc(TC, 0, rc);				// RC einstellen
    NVIC_EnableIRQ(TC0_IRQn);			// Interrupt einschalten
	tc_start(TC, 0);					// Timer starten

	// I2C aktivieren
	pmc_enable_periph_clk(ID_PIOB);	// Clock auf PIOB
	pio_set_peripheral 	( PIOB, PIO_PERIPH_B, PIO_PB0 | PIO_PB1 );	// TWI pins aktivieren
	twi_init(); // Twi inistalisieren
	delay_us(500);

	//UART initalisieren
	pmc_enable_periph_clk(ID_UART1); // Clock auf UART
	pio_set_peripheral	( PIOB, PIO_PERIPH_A, serielltxd | seriellrxd); // UART Pins aktivieren
	sam_uart_opt_t uartoptionen  = {
		.ul_baudrate = 115200,
		.ul_mode =  UART_MR_PAR_NO,
		.ul_mck = SystemCoreClock
	}; // Daten für den UART
	uart_init(seriell, &uartoptionen);
	uart_enable_interrupt(UART1, UART_IER_RXRDY);
	NVIC_EnableIRQ(seriell_IRQ);


	//ADC Init
	pmc_enable_periph_clk(ADC);
	adc_enable();
	adc_get_config_defaults(&adc_cfg);
	adc_init(ADC, &adc_cfg);
	adc_set_trigger(ADC, ADC_TRIG_SW);
	adc_channel_enable(ADC, ADC_CHANNEL_2);


	delay_ms(1000);
	nunchuck_init();
	uint32_t zeit_berechnung;
	volatile	uint32_t zeit;
	
	








	touchtime = ticks;	
	quadrat_time = ticks;


	while(1)
	{
		
		//receive_nundhuck();
		//print_nunchuch_daten();

	/*
		if (nunchuck.z_button == 0 && nunchuck.c_button != 0)
		{
			hoehe = (nunchuck.y_stick - NUN_Y_MITTE) * 0.15;
		}
		else if (nunchuck.z_button == 1 && nunchuck.c_button == 1)
		{
			x_achse = (nunchuck.x_stick - NUN_X_MITTE) * 0.35;
			y_achse = (nunchuck.y_stick - NUN_Y_MITTE) * 0.35;
		}
	
		if (nunchuck.z_button == 0 && nunchuck.c_button == 0)
		{
			z_rotation = (nunchuck.x_stick - NUN_X_MITTE) * 0.35;
		}
		else
		{
			z_rotation = 0;
		}
	
	*/
		if (nunchuck.c_button != 0)
		{
	//		soll_x = 0.1;
		}
		else
		{
	//		soll_x = 0.14;
		}
		
	

		// Position bestimmen!
		if (ticks-touchtime > (TA_MESS*1000))
		{
			touchtime = ticks;	
			
			touchkoord();
			// Wenn Ball nicht auf Platte ist Y negativ.
			// Ball verliert manchmal Haftung
			if (y_koord > 0.01)
			{
				x_koord = (x_koord + (T_MESS/TA_MESS)*x_mess_alt ) * (TA_MESS/(T_MESS+TA_MESS));
				y_koord = (y_koord + (T_MESS/TA_MESS)*y_mess_alt ) * (TA_MESS/(T_MESS+TA_MESS));
				x_mess_alt = x_koord;
				y_mess_alt = y_koord;	
			}

			if (y_koord > 0.01)
			{
				ballfromplatetime = ticks;
			}
			
			if (ticks - ballfromplatetime > 1000)
			{
				x_koord = soll_x;
				y_koord = soll_y;
			}
			print_xy_raw();
		}

		// Regler und Stellwinkel berechnen!
		if (ticks-reglertime > (TA*1000))
		{
			reglertime = ticks;
			// Regler berechnen
			//beobachter_x();
			//regler_x();
			//beobachter_y();
			//regler_y();
			
			// Punkte bauen, Stellwinkel berechnen
			drehung_anwenden(platformpunkte, transformiertepunkte, x_rotation, y_rotation, z_rotation);
			verschiebung_anwenden(transformiertepunkte, temp3, x_achse, y_achse, hoehe);
			systemtransformation(basispunkte, temp3, transformiertepunkte);
			stellwinkel(transformiertepunkte, winkel_vor);
		
			winkel_aktuell[0] = SERVO_A1_MITTE-winkel_vor[0]/1.25;
			winkel_aktuell[1] = SERVO_A2_MITTE+winkel_vor[1]/1.25;
			winkel_aktuell[2] = SERVO_B1_MITTE-winkel_vor[2]/1.25;
			winkel_aktuell[3] = SERVO_B2_MITTE+winkel_vor[3]/1.25;
			winkel_aktuell[4] = SERVO_C1_MITTE-winkel_vor[4]/1.25;
			winkel_aktuell[5] = SERVO_C2_MITTE+winkel_vor[5]/1.25;
		}

		zeit = ticks-zeit_berechnung;
	
		// Modus auswählen
		switch(modus_wahl) {
			case 0: 
					soll_x = mod_mitte[0][0];
					soll_y = mod_mitte[0][1];
			break;
			case 1:
			if (ticks-quadrat_time > 15000)
			{
					quadrat_time = ticks;
					if (mod_k > 3)
					{
						mod_k = 0;
					}
					soll_x = mod_quadrat[mod_k][0];
					soll_y = mod_quadrat[mod_k][1];
					mod_k++;
			}
			 
			break;
			
			case 2: 
			
			break;
			default: 
					soll_x = mod_mitte[0][0];
					soll_y = mod_mitte[0][1];
			break;
		}

	}
}

// Winkel stellen!
void TC0_Handler()
{
	servoticks++;
	if (servoticks == winkel_aktuell_neu[0])
	{
		PIOA->PIO_CODR = SERVO_A1;
	} 
	if (servoticks == winkel_aktuell_neu[1])
	{
		PIOA->PIO_CODR = SERVO_A2;
	}
	if (servoticks == winkel_aktuell_neu[2])
	{
		PIOA->PIO_CODR = SERVO_B1;
	}
	if (servoticks == winkel_aktuell_neu[3])
	{
		PIOA->PIO_CODR = SERVO_B2;
	}	
	if (servoticks == winkel_aktuell_neu[4])
	{
		PIOA->PIO_CODR = SERVO_C1;
	}
	if (servoticks == winkel_aktuell_neu[5])
	{
		PIOA->PIO_CODR = SERVO_C2;
	}	
	if (servoticks == 210)
	{
		rc = (SystemCoreClock /2 /10000);	// 10khz
		tc_write_rc(TC, 0, rc);	
	}
	
	if(servoticks == 320)
	{
		servoticks = 0;
		pio_set(PIOA, SERVO_A1 | SERVO_A2 | SERVO_B1| SERVO_B2| SERVO_C1 | SERVO_C2);
		rc = (SystemCoreClock /2 /100000);	// 10khz
		tc_write_rc(TC, 0, rc);
		winkel_aktuell_neu[0] = winkel_aktuell[0];
		winkel_aktuell_neu[1] = winkel_aktuell[1];
		winkel_aktuell_neu[2] = winkel_aktuell[2];
		winkel_aktuell_neu[3] = winkel_aktuell[3];
		winkel_aktuell_neu[4] = winkel_aktuell[4];
		winkel_aktuell_neu[5] = winkel_aktuell[5];
	}
	
	test = TC->TC_CHANNEL[0].TC_SR;
	//aktuellerwert = TC->TC_CHANNEL[0].TC_CV;
}

// Systemtakt 1ms
void SysTick_Handler(){
	ticks++;	
}

// I2C initalisieren
void twi_init(void)  // I2C einstellen auf 100khz
{
	twi_master_options_t opt = {
		.speed = 400000,
		.chip  = 0x51
	};
	twi_master_setup( TWI2, &opt);
}

// Uart Zeichen holen!
void UART1_Handler()   // Uart Interrupt bei Zeichen angekommen
{
	if(uart_is_rx_ready(seriell)) // Wenn Zeichen im Buffer
	{
		uart_read(seriell, &uartrxbuffer[uartindex]); // Zeichen aus Buffer holen
		if (uartrxbuffer[uartindex] != 0)	// Solange nicht Ende des Strings erreicht
		{
			if(uartrxbuffer[uartindex] == '\n' && uartrxbuffer[uartindex-1] == '\r' && uartindex >=1) // Wenn letztes Zeichen angekommen
			{
				uartrxbuffer[uartindex-1] = 0;			// Wenn ende erreicht letztes Zeichen mit 0 überschreiben
				strcpy(eingabebuffer, &uartrxbuffer);	// ganzen eingangsbuffer kopieren
				//uartrxbuffer[0] = 0;	
				uartindex = 0;							// Index wieder auf Anfang
			}
			else
			{
				uartindex++;  //Index um eins erhöhen fall noch kein Ende erreicht wurde
			}
		}
	}
}

// UART Zeichen senden
void uartsendbyte (uint8_t c) // Uart senden eines Bytes
{
	while (!uart_is_tx_ready(seriell) && !uart_is_tx_empty(seriell)); // Solang UART beschäftigt und Sendebuffer nicht leer ist WARTEN
	uart_write 	(seriell, c);	// Zeichen senden
	return;
}

// UART String senden
void uartsendstring(char s[])	// Uart senden eines Strings
{
	uint8_t x = 0;
	while (s[x] != 0) {			// Solang kein Ende des Strings gefunden wurde weiter senden
		uartsendbyte(s[x]);		// Zeichen senden
		x++;					// nächstes zeichen
	}
	return;
}

void print_menu()
{
	char text[200];
	sprintf(text,	"T T T T\n");
	uartsendstring(text);
	
	return;
}

void print_nunchuch_daten()
{
	char text[200];
	sprintf(text,   "  x-Achse: %x y-Achse: %x\n  x besch: %x ", nunchuck.x_stick,nunchuck.y_stick, nunchuck.x_beschl);
	uartsendstring(text);
	sprintf(text,	" y besch: %x  z besch: %x\n", nunchuck.y_beschl, nunchuck.z_beschl);
	uartsendstring(text);
	sprintf(text,   "  c button: %x\n  z button: %x\n\n\n\n", nunchuck.c_button, nunchuck.z_button );
	uartsendstring(text);
}






//---------------------------------------------------------------------------------------
// Inverse Kinematik

// Drehung um Achsen
void drehung_x(float x[],float y[],float winkel){ // X = Originale Daten, Y = gedrehte Daten, winkel = winkel 
	winkel = RAD*winkel;
	y[0] = x[0];
	y[1] = x[1]*cos(winkel)    +  x[2]*sin(winkel);
	y[2] = x[1]*(-sin(winkel)) +  x[2]*cos(winkel);
}

void drehung_y(float x[],float y[],float winkel){ // X = Originale Daten, Y = gedrehte Daten, winkel = winkel 
	winkel = RAD*winkel;
	y[0] = x[0]*cos(winkel) + x[2]*(-sin(winkel));
	y[1] = x[1];
	y[2] = x[0]*sin(winkel) + x[2]*cos(winkel);
}

void drehung_z(float x[],float y[],float winkel){ // X = Originale Daten, Y = gedrehte Daten, winkel = winkel 
	winkel = RAD*winkel;
	y[0] = x[0]*cos(winkel)		+ x[1]*sin(winkel);
	y[1] = x[0]*(-sin(winkel))	+ x[1]*cos(winkel);
	y[2] = x[2];
}

void drehung_z_minus_120(float x[],float y[]){ // X = Originale Daten, Y = gedrehte Daten
	y[0] = x[0]*COSPM120		+ x[1]*SINM120;
	y[1] = x[0]*(-SINM120)	+ x[1]*COSPM120;
	y[2] = x[2];
}

void drehung_z_plus_120(float x[],float y[]){ // X = Originale Daten, Y = gedrehte Daten
	y[0] = x[0]*COSPM120		+ x[1]*SIN120;
	y[1] = x[0]*(-SIN120)	+ x[1]*COSPM120;
	y[2] = x[2];
}

// Drehungen anwenden auf Punkte, 6 Befestigungspunkte
void drehung_anwenden(float o[][3], float t[][3], float winkel_x, float winkel_y, float winkel_z){ // O = Orginale Daten, t = transfomierte Daten
	float temp[3];
	float temp2[3];
	drehung_x(o[0],temp,winkel_x);
	drehung_y(temp,temp2, winkel_y);
	drehung_z(temp2,t[0], winkel_z);
	
	drehung_x( o[1],temp,winkel_x);
	drehung_y(temp,temp2, winkel_y);
	drehung_z(temp2,t[1], winkel_z);
	
	drehung_x( o[2],temp,winkel_x);
	drehung_y(temp,temp2, winkel_y);
	drehung_z(temp2, t[2], winkel_z);
	
	drehung_x( o[3],temp,winkel_x);
	drehung_y(temp,temp2, winkel_y);
	drehung_z(temp2, t[3], winkel_z);
	
	drehung_x( o[4],temp,winkel_x);
	drehung_y(temp,temp2, winkel_y);
	drehung_z(temp2, t[4], winkel_z);
	
	drehung_x( o[5],temp,winkel_x);
	drehung_y(temp,temp2, winkel_y);
	drehung_z(temp2, t[5], winkel_z);
}

// Punkte in der Ebene verschieben
void verschiebung_anwenden(float o[][3], float t[][3], float x, float y, float z){ // O = Orginale Daten, t = transfomierte Daten, x,y,z in mm
	t[0][0] = o[0][0]+ x; t[0][1] = o[0][1]+ y; t[0][2] = o[0][2]+ z;
	t[1][0] = o[1][0]+ x; t[1][1] = o[1][1]+ y; t[1][2] = o[1][2]+ z;
	t[2][0] = o[2][0]+ x; t[2][1] = o[2][1]+ y; t[2][2] = o[2][2]+ z;
	t[3][0] = o[3][0]+ x; t[3][1] = o[3][1]+ y; t[3][2] = o[3][2]+ z;
	t[4][0] = o[4][0]+ x; t[4][1] = o[4][1]+ y; t[4][2] = o[4][2]+ z;
	t[5][0] = o[5][0]+ x; t[5][1] = o[5][1]+ y; t[5][2] = o[5][2]+ z;
}

// Punkt auf jeweiliges Servokoordinatensystem transformieren, dadurch werden die Basispunkte 0,0,0 und eine Achse bleibt gleich
void systemtransformation(float o[][3], float ta[][3], float t[][3]){ // O = Orginale Daten, t = transfomierte Daten
	float temp[3];
	t[0][0] = ta[0][0] - o[0][0]; t[0][1] = ta[0][1] - o[0][1]; t[0][2] = ta[0][2] - o[0][2]; // tA11 = (tA1-A1);
	t[1][0] = ta[1][0] - o[1][0]; t[1][1] = ta[1][1] - o[1][1]; t[1][2] = ta[1][2] - o[1][2]; // tA21 = (tA2-A2);
	
	temp[0] = ta[2][0] - o[2][0]; temp[1] = ta[2][1] - o[2][1]; temp[2] = ta[2][2] - o[2][2]; // tB11 = (tB1-B1)*Rz;
	drehung_z_minus_120(temp, t[2]);
	temp[0] = ta[3][0] - o[3][0]; temp[1] = ta[3][1] - o[3][1]; temp[2] = ta[3][2] - o[3][2]; // tB21 = (tB2-B2)*Rz;
	drehung_z_minus_120(temp, t[3]);
	
	temp[0] = ta[4][0] - o[4][0]; temp[1] = ta[4][1] - o[4][1]; temp[2] = ta[4][2] - o[4][2]; // tC11 = (tC1-C1)*Rz;
	drehung_z_plus_120(temp, t[4]);
	temp[0] = ta[5][0] - o[5][0]; temp[1] = ta[5][1] - o[5][1]; temp[2] = ta[5][2] - o[5][2]; // tC21 = (tC2-C2)*Rz;
	drehung_z_plus_120(temp, t[5]);	
}

// Servowinkel berechnen
void stellwinkel(float ta[][3], float a[]){ //ta Servokoordinatensystem, a = Stellwinkel +/- von 0°
	a[0] = asin((ta[0][0]*ta[0][0]+ta[0][1]*ta[0][1]+ta[0][2]*ta[0][2]+R2-LAENGE2)/(2*R*(sqrt(ta[0][0]*ta[0][0]+ta[0][2]*ta[0][2]))))+atan2(ta[0][0],ta[0][2]);
	a[0] = a[0]*GRAD;
	a[1] = asin((ta[1][0]*ta[1][0]+ta[1][1]*ta[1][1]+ta[1][2]*ta[1][2]+R2-LAENGE2)/(2*R*(sqrt(ta[1][0]*ta[1][0]+ta[1][2]*ta[1][2]))))-atan2(ta[1][0],ta[1][2]);
	a[1] = a[1]*GRAD;
	a[2] = asin((ta[2][0]*ta[2][0]+ta[2][1]*ta[2][1]+ta[2][2]*ta[2][2]+R2-LAENGE2)/(2*R*(sqrt(ta[2][0]*ta[2][0]+ta[2][2]*ta[2][2]))))+atan2(ta[2][0],ta[2][2]);
	a[2] = a[2]*GRAD;
	a[3] = asin((ta[3][0]*ta[3][0]+ta[3][1]*ta[3][1]+ta[3][2]*ta[3][2]+R2-LAENGE2)/(2*R*(sqrt(ta[3][0]*ta[3][0]+ta[3][2]*ta[3][2]))))-atan2(ta[3][0],ta[3][2]);
	a[3] = a[3]*GRAD;
	a[4] = asin((ta[4][0]*ta[4][0]+ta[4][1]*ta[4][1]+ta[4][2]*ta[4][2]+R2-LAENGE2)/(2*R*(sqrt(ta[4][0]*ta[4][0]+ta[4][2]*ta[4][2]))))+atan2(ta[4][0],ta[4][2]);
	a[4] = a[4]*GRAD;
	a[5] = asin((ta[5][0]*ta[5][0]+ta[5][1]*ta[5][1]+ta[5][2]*ta[5][2]+R2-LAENGE2)/(2*R*(sqrt(ta[5][0]*ta[5][0]+ta[5][2]*ta[5][2]))))-atan2(ta[5][0],ta[5][2]);
	a[5] = a[5]*GRAD;
}

void touchkoord(){ 
	
	//X
	PIOA->PIO_SODR = OR | UR;
	PIOA->PIO_CODR = OL;
	delay_us(50);
	adc_start_software_conversion(ADC);
	while (!(adc_get_interrupt_status(ADC) & (1 << ADC_CHANNEL_2)));
	x_adc = adc_channel_get_value(ADC, ADC_CHANNEL_2);
	delay_us(50);
	
//	PIOA->PIO_CODR = OL | OR | UR;
//	delay_us(100);
	
	//Y
//	PIOA->PIO_SODR = OR | OL;
//	PIOA->PIO_CODR = UR;
//	delay_us(50);
//	adc_start_software_conversion(ADC);
//	while (!(adc_get_interrupt_status(ADC) & (1 << ADC_CHANNEL_2)));
//	y_adc = adc_channel_get_value(ADC, ADC_CHANNEL_2);
	
//	PIOA->PIO_CODR = OL | OR | UR;
//	delay_us(100);
	
	if ((x_adc <= 620 && x_adc >= 100) && (y_adc <= 420 && y_adc >= 50))
	{
		x_koord = (A*x_adc+B*y_adc+C)/1000;
		y_koord = (D*x_adc+E*y_adc+F)/1000;		
	}


}

void print_xy()
{
	char text[200];
	sprintf(text,   " x-koord: %.5f  \n", x_koord);
	uartsendstring(text);
	sprintf(text,	" y-koord: %.5f  \n\n", y_koord);
	uartsendstring(text);
}

void print_xy_raw()
{
	char text[200];
	sprintf(text,   " x-koord: %d  \n", x_adc);
	uartsendstring(text);
	sprintf(text,	" y-koord: %d  \n\n", y_adc);
	uartsendstring(text);
}

void regler_x()
{
	uk_2_x = uk_1_x;
	uk_1_x = uk_x;
	uk_x = soll_x - x_koord;
	
	yk_2_x = yk_1_x;	
	yk_1_x = yk_x;
	yk_x =  ((KP*((TN*TV)*((uk_x-2*uk_1_x+uk_2_x)/(TA*TA))+(((TN+TV)*(uk_x-uk_1_x))/TA)+uk_x))+((TN*TR*2*yk_1_x)/(TA*TA))-(TN*TR*yk_2_x/(TA*TA))+((TN*yk_1_x)/TA))/(((TN*TR)/(TA*TA))+(TN/TA));
	ykb_x = yk_x;// - (2*y1_x + x_koord);
	
	if (ykb_x > 1.8)
	{
		ykb_x = 1.8;
	}
	else if (ykb_x < -1.8)
	{
		ykb_x = -1.8;
	}
	
	y_rotation = (ykb_x/9.81)*GRAD;
}

void beobachter_x()
{
	yb_x = x_koord-y2_x;
	y1_x = (ykb_x+3*yb_x)*TA+y1_1_x;
	y1_1_x = y1_x;
	y2_x = (y1_x+3*yb_x)*TA+y2_1_x;
	
	
	if (y2_x > 0.307)
	{
		y2_x = 0.307;
	}
	else if (y2_x < 0)
	{
		y2_x = 0;
	}
	
	y2_1_x = y2_x;
}

void regler_y()
{
	uk_2_y = uk_1_y;
	uk_1_y = uk_y;
	uk_y = soll_y - y_koord;
	
	yk_2_y = yk_1_y;
	yk_1_y = yk_y;
	yk_y = ((KP*((TN*TV)*((uk_y-2*uk_1_y+uk_2_y)/(TA*TA))+(((TN+TV)*(uk_y-uk_1_y))/TA)+uk_y))+((TN*TR*2*yk_1_y)/(TA*TA))-(TN*TR*yk_2_y/(TA*TA))+((TN*yk_1_y)/TA))/(((TN*TR)/(TA*TA))+(TN/TA));
	ykb_y = yk_y;// - (2*y1_y + y_koord);
	
	if (ykb_y > 1.8)
	{
		ykb_y = 1.8;
	}
	else if (ykb_y < -1.8)
	{
		ykb_y = -1.8;
	}
	
	x_rotation = -(ykb_y/9.81)*GRAD;
}

void beobachter_y()
{
	yb_y = y_koord-y2_y;
	y1_y = (ykb_y+3*yb_y)*TA+y1_1_y;
	y1_1_y = y1_y;
	y2_y = (y1_y+3*yb_y)*TA+y2_1_y;
	if (y2_y > 0.230)
	{
		y2_y = 0.230;
	}
	else if (y2_y < 0)
	{
		y2_y = 0;
	}
	y2_1_y = y2_y;
}