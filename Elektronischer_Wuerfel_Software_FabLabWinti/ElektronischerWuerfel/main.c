/*
 * Elektronischer Wuerfel V1.1
 * (kleine Version)
 *
 * Created: 17.02.2016 15:20:19
 * Author : rags
 */

#define PATTERN1

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

/* Funktionsprototypen */
void enterSleep();
void One();
void Two();
void Three();
void Four();
void Five();
void Six();
void Pat1();
void Pat2();
void Pat3();
void LEDoff();

/* Bits setzen (sbi) und löschen (cbi) */
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

/* Inputs & Outputs */
#define led_A      1             // IC Pin6, PB1, output
#define led_B      3             // IC Pin2, PB3, output
#define led_C      0             // IC Pin5, PB0, output
#define led_D      2             // IC Pin7, PB2, output
#define buttonPin  4             // IC Pin3, PB4, input

/* Defines */
#define TIMEOUT_DISPLAY   500       // Timeout-Variable während dem Anzeigen der gewürfelten Zahl
#define TIMEOUT_SHUFFLE   5000      // Timeout-Variable während dem Würfeln

#define TIME_ON		5				// Zeitanteil von 10ms, in dem die LED eingeschaltet ist (Dimming)
#define TIME_OFF	5				// Zeitanteil von 10ms, in dem die LED ausgeschaltet ist (Dimming)


/* Variablen */
volatile char buttonState;					// Speichervariable für den Zustand des Kippschalters
volatile unsigned char dicecnt = 0;			// Zählvariable für die Zufallszahl (Wertebereich 0...5)
volatile unsigned int timeoutCnt_disp;		// Zählvariable für Timeout während dem Anzeigen der gewürfelten Zahl
volatile unsigned int timeoutCnt_shuf;		// Zählvariable für Timeout während dem Würfeln
volatile unsigned char dispcount = 0;		// Zählvariable für Timeout während dem Würfeln
volatile unsigned char count = 0;			// Zählvariable für Timeout während dem Würfeln

/* Einen Zeiger erstellen, der auf ein Array mit Funktionen (Anzeige Zahlen 1..6) zeigt */
typedef void (* NumFuncPTR) ();
NumFuncPTR ShowNumber[6] = {One,Two,Three,Four,Five,Six};
/* Würfelmuster definieren, das beim Würfeln angezeigt wird */
#ifdef PATTERN1
	NumFuncPTR ShowPattern[8] = {Pat1,Pat2,One,Pat2,Pat3,Pat2,One,Pat2};
#endif
#ifdef PATTERN2
	NumFuncPTR ShowPattern[8] = {Pat1,Pat2,Pat3,Pat2,Pat1,Pat2,Pat3,Pat2};
#endif
#ifdef PATTERN3
	NumFuncPTR ShowPattern[8] = {One,Six,LEDoff,LEDoff,One,Six,LEDoff,LEDoff};
#endif

void setup() {
	/* Ungenutzte Funktionen ausschalten -> Batterieverbrauch optimieren! */
	power_adc_disable();          // Analog/Digital-Wandler ausschalten
	power_usi_disable();          // Serielle Schnittstelle ausschalten
	power_timer1_disable();       // Timer 1 ausschalten
	
	/* Port B konfigurieren: 0 = input, 1 = output */
	DDRB |= (1 << led_A);
	DDRB |= (1 << led_B);
	DDRB |= (1 << led_C);
	DDRB |= (1 << led_D);
	DDRB &= ~(1 << buttonPin);
	
	/* Timeout-Variablen zurücksetzen */
	timeoutCnt_disp = 0;
	timeoutCnt_shuf = 0;
}


int main(void) {
	
	setup();
	
    while (1) {
	
		/* Einlesen des Kippschalters */
		buttonState = (PINB >> buttonPin) & 1;
	
		timeoutCnt_shuf = 0;
		
		/* Solange der Würfel gekippt wird, wird die folgende Schleife nicht verlassen: */
		while(buttonState == 0) {
			
			/* Aktuellen Zeitstempel in Variable speichern */
			timeoutCnt_disp = 0;
			
			/* Zeitverzögerung in ms, welche die Zählgeschwindigkeit regelt */
		
			timeoutCnt_shuf++;
			
			/* Zählervariable um 1 erhöhen. Wenn sie 5 erreicht hat, wieder nullsetzen */
			if (dicecnt < 5) dicecnt++;
			else dicecnt = 0;
			
			/* Teiler / 73 */
			if (count < 73) count++;
			else {
				count = 0;
				if (dispcount < 7) dispcount++;
				else dispcount = 0;
			}
			
			/* Zählmuster anzeigen */
			ShowPattern[dispcount]();
			/* LED's nur xx% der Zeit einschalten -> Batterieverbrauch optimieren */
			_delay_ms(0.1*TIME_ON);
			LEDoff();
			_delay_ms(0.1*TIME_OFF);

			/* Einlesen des Kippschalters */
			buttonState = (PINB >> buttonPin) & 1;
			
			/* Nach einer bestimmten Zeit wird der Würfel abgeschaltet -> Batterieverbrauch! */
			if (timeoutCnt_shuf > TIMEOUT_SHUFFLE) {
				LEDoff();
				enterSleep();
			}

		}
		
		/* Gewürfelte Zahl mit den LED anzeigen */
		ShowNumber[dicecnt]();
		/* LED's nur xx% der Zeit einschalten -> Batterieverbrauch optimieren */
		_delay_ms(TIME_ON);
		LEDoff();
		_delay_ms(TIME_OFF);
		/* Timeout-Variable inkrementieren */
		timeoutCnt_disp++;
			
		/* Nach einer bestimmten Zeit wird der Würfel abgeschaltet -> Batterieverbrauch! */
		if (timeoutCnt_disp > TIMEOUT_DISPLAY){
			LEDoff();
			enterSleep();
		}
		
    }
}


void enterSleep() {

	sbi(GIMSK,PCIE);                          // Pin-Change-Interrupt einschalten
	sbi(PCMSK,PCINT4);                        // Pin-Change-Eingang definieren (ATtiny: IC Pin3,PB4)
	
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	cli();
	sleep_enable();
	sleep_bod_disable();
	sei();
	sleep_cpu();
	
	/* ------------------------------------------------------------------------------------------- */
	/* uC befindet sich im Sleep Mode. Nach dem Kippen des Würfels wird der Code hier fortgesetzt! */
	/* ------------------------------------------------------------------------------------------- */

	sleep_disable();
	cli();
	
	/* Ungenutzte Funktionen ausschalten -> Batterieverbrauch optimieren! */
	power_adc_disable();
	power_usi_disable();
	power_timer1_disable();
}


/* Ist der Würfel im Sleep Mode und wird gekippt, so wird ein Interrupt mit dieser Interruptroutine ausgelöst */
ISR(PCINT0_vect) {
	unsigned char tempButtonState1 = (PINB >> buttonPin) & 1;
	/* SW-Entprellung: 50ms (verhindert das Würfeln z.B. bei Vibrationen)*/
	_delay_ms(50);
	unsigned char tempButtonState2 = (PINB >> buttonPin) & 1;
	if (tempButtonState1 != tempButtonState2) enterSleep();
}

/* ---------------------------------------------- */
/*  Funktionen zur Anzeige der gewürfelten Zahlen */
/* ---------------------------------------------- */

void One(){
	PORTB = 0b00001011;
};
void Two(){
	PORTB = 0b00001101;
};
void Three(){
	PORTB = 0b00001001;
};
void Four(){
	PORTB = 0b00000101;
};
void Five(){
	PORTB = 0b00000001;
};
void Six(){
	PORTB = 0b00000100;
};
void Pat1(){
	PORTB = 0b00001101;
};
void Pat2(){
	PORTB = 0b00001110;
};
void Pat3(){
	PORTB = 0b00000111;
};
void LEDoff(){
	PORTB = 0b00001111;
}
