/*
 * pin.h
 * Atmel mega32  digital read/write
 * Created: 12/25/2012 00:59:20
 * Author: Muchiri John
 * (c) www.dogolabs.com
 *
 *
 * Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in
 the documentation and/or other materials provided with the
 distribution.

 * Neither the name of the copyright holders nor the names of
 contributors may be used to endorse or promote products derived
 from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE. See the GNU General Public License for more details.
 */ 


#ifndef PIN_H_
#define PIN_H_

#include <avr/io.h>
#include <inttypes.h>

/************************************************************************
* structure to hold registers and bit number.                                                                      
************************************************************************/
struct reg_bit{
	volatile uint8_t* DDRx;
	volatile uint8_t* PORTx;
	volatile uint8_t* PINx;
	uint8_t bit; 
};

/************************************************************************
* enumeration to hold bit set/reset values or write mode. 
************************************************************************/
enum{	
	LOW, HIGH, INPUT=0, OUTPUT
};
/************************************************************************
* port registers and bit number.
* DDRx - data direction register.
* PORTx - write register.
* PINx - read register.       
************************************************************************/
static const struct reg_bit pins_def[] = {
{},
{ &DDRB, &PORTB, &PINB, PINB0 }, //1
{ &DDRB, &PORTB, &PINB, PINB1 }, //2
{ &DDRB, &PORTB, &PINB, PINB2 }, //3
{ &DDRB, &PORTB, &PINB, PINB3 }, //4
{ &DDRB, &PORTB, &PINB, PINB4 }, //5
{ &DDRB, &PORTB, &PINB, PINB5 }, //6
{ &DDRB, &PORTB, &PINB, PINB6 }, //7
{ &DDRB, &PORTB, &PINB, PINB7 }, //8
{},{},{},{},{},
{ &DDRD, &PORTD, &PIND, PIND0 }, //14
{ &DDRD, &PORTD, &PIND, PIND1 }, //15
{ &DDRD, &PORTD, &PIND, PIND2 }, //16
{ &DDRD, &PORTD, &PIND, PIND3 }, //17
{ &DDRD, &PORTD, &PIND, PIND4 }, //18
{ &DDRD, &PORTD, &PIND, PIND5 }, //19
{ &DDRD, &PORTD, &PIND, PIND6 }, //20
{ &DDRD, &PORTD, &PIND, PIND7 }, //21
	
{ &DDRC, &PORTC, &PINC, PINC0 }, //22
{ &DDRC, &PORTC, &PINC, PINC1 }, //23
{ &DDRC, &PORTC, &PINC, PINC2 }, //24
{ &DDRC, &PORTC, &PINC, PINC3 }, //25
{ &DDRC, &PORTC, &PINC, PINC4 }, //26
{ &DDRC, &PORTC, &PINC, PINC5 }, //27
{ &DDRC, &PORTC, &PINC, PINC6 }, //28
{ &DDRC, &PORTC, &PINC, PINC7 }, //29
{},{},{},
{ &DDRA, &PORTA, &PINA, PINA7 }, //33
{ &DDRA, &PORTA, &PINA, PINA6 }, //34
{ &DDRA, &PORTA, &PINA, PINA5 }, //35
{ &DDRA, &PORTA, &PINA, PINA4 }, //36
{ &DDRA, &PORTA, &PINA, PINA3 }, //37
{ &DDRA, &PORTA, &PINA, PINA2 }, //38
{ &DDRA, &PORTA, &PINA, PINA1 }, //39
{ &DDRA, &PORTA, &PINA, PINA0 }  //40
};

/************************************************************************
* declare number of physical pins
************************************************************************/
static const uint8_t pins = 41;

/************************************************************************
* check port pin direction.
* pin - physical pin number on the AVR.                                             
* e.g pinStatus(6); checks the status of PORTB5 - B5 whether an input or
* output. 
************************************************************************/
static inline uint8_t pinStatus(uint8_t pin) {
	if (pin < pins) {
		return (*pins_def[pin].DDRx >> pins_def[pin].bit) & 1;
	} else { return 0;  }
}		

/************************************************************************
* sets i/o direction of a port pin.
* pin - physical pin on the AVR.
* mode - set i/o mode of port bit, 1/0, OUTPUT/INPUT.
* e.g pinSet(6, 1); sets PORTB5 - B5  as output.                                                                      
************************************************************************/
static inline void pinSet(uint8_t pin, uint8_t mode) {
	if (pin < pins) {
		if (mode) {
			*pins_def[pin].DDRx |=  _BV(pins_def[pin].bit);
		} else {
			*pins_def[pin].DDRx &= ~ _BV(pins_def[pin].bit);
		}
	} else {}
}

/************************************************************************
* read bit.
* pin - physical pin number on the AVR.
* e.g pinRead(6); reads the status of PORTB5 bit.                                                                    
************************************************************************/
static inline uint8_t pinRead(uint8_t pin) {
	if (pin < pins) {
		return (*pins_def[pin].PINx >> pins_def[pin].bit) & 1;
	} else { return 0; }
}

/************************************************************************
* set/reset bit.
* pin - physical pin number on the AVR.
* Value - set/ reset value , 0/1, HIGH/LOW.
* e.g pinWrite(6, 1);  sets the bit of PORTB5 - B5 .                                                                 
************************************************************************/
static inline void pinWrite(uint8_t pin, uint8_t value) {
	if (pin < pins) {
		if (value) {
			*pins_def[pin].PORTx |= _BV(pins_def[pin].bit);
		} else {
			*pins_def[pin].PORTx &= ~ _BV(pins_def[pin].bit);
		}
	}
}
/************************************************************************
* toggle bit.
* pin - physical pin number on the AVR.                              
* e.g pinToggle(6, 1) toggles bit 5 of PORTB - B5. 
************************************************************************/
static inline uint8_t pinToggle(uint8_t pin) {
	if (pin < pins) {
		return *pins_def[pin].PORTx ^= _BV(pins_def[pin].bit);
	} else { return 0; }
}
/************************************************************************
* set i/o mode of many pins at once
* *p - pointer to an array of pins.
* mode - either OUTPUT/INPUT, 1/0.
* uint8_t ar[3] = {4, 5, 6};
* e.g pinMultiSet(ar, 1); sets pins 4, 5 and 6 as outputs.                                                                      
************************************************************************/
static inline void pinsMultiSet(const uint8_t *p, uint8_t mode){
	while(*p){
		mode ? pinSet(*p++, 1) : pinSet(*p++, 0);
	}
}
/************************************************************************
* write many pins at once
* *p - pointer to an array of pins.
* mode - either HIGH/LOW, 1/0.
* uint8_t ar[3] = {4, 5, 6};
* e.g pinMultiWrite(ar, 1); sets pins 4, 5 and 6 High.                                                                      
************************************************************************/
static inline void pinsMultiWrite(const uint8_t *p, uint8_t mode){
	while(*p){
			mode ? pinWrite(*p++, 1) : pinWrite(*p++, 0);
	}
}
/**************************************************************************
variable to set the pin array
e.g
	multiPins outPins[4] = { 2,3,4,5 };
	pinsMultiWrite(outPins, HIGH);
	set pins 2,3,4,5 high.
**************************************************************************/
typedef uint8_t multiPins;

#endif /* PIN_H_ */