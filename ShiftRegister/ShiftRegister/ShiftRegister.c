/*
 * ShiftRegister.c
 *
 * Created: 4/8/2013 17:05:58
 * Author: Muchiri John
 * Author: Mercy Ngoiri
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

/*
 *code for e-clothing. developed for as a part of open nasa challenge (spaceapps)
 *to celebrate 55 CANCRI E planet.                                                                      
*/


#include <avr/io.h>
/************************************************************************/
/* Frequency 16MHz                                                      */
/************************************************************************/
#define F_CPU 16000000L
#include <util/delay.h>
#include <avr/interrupt.h>

#include "pin.h"
#include "dat.h"

/************************************************************************/
/* Latch pin                                                            */
/************************************************************************/
#define latch_pin 17
/************************************************************************/
/*clock pin                                                             */
/************************************************************************/
#define clock_pin 18
/************************************************************************/
/* internal data pins                                                   */
/************************************************************************/
uint8_t int_data_pins[8]=
{
	5 ,6 ,11 ,12 ,13 ,14 ,15, 16
}; 
/************************************************************************/
/* global index for selecting multiplexing pins.                        */
/************************************************************************/
uint8_t plex_index = 0;
/************************************************************************/
/* pins for multiplexing.                                               */
/************************************************************************/
multiPins plex_pins[5] = {23, 24, 25, 26, 27};
/************************************************************************/
/* serial communication                                                 */
/************************************************************************/
void USART_Init( unsigned int baud )
{
	/* Set baud rate */
	UBRR0H = (unsigned char)(baud>>8);
	UBRR0L = (unsigned char)baud;
	/* Enable transmitter */
	UCSR0B = (1<<TXEN0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
	DDRD  |= (1 << PORTD1);
}

inline void USART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

/************************************************************************/
/* info interrupt														*/
/************************************************************************/
inline void InterruptInit()
{
	EICRA = (1 << ISC01) | (1 << ISC00); /*enable INT0 on rising edge*/
	EIMSK = (1 << INT0); /*enable INT0 interrupt*/
	sei(); /*enable interrupts*/
}

/************************************************************************/
/*  delay                                                               */
/************************************************************************/
int delayms = 2;
static inline void delay(int dl)
{
	while(dl)
	{
		_delay_ms(1);
		dl--;
	}
}
/************************************************************************/
/* Multiplexing method                                                  */
/************************************************************************/
static inline void MultiPlex(uint8_t *pin)
{
	pinsMultiWrite(plex_pins, LOW);
	pinWrite(plex_pins[*pin], HIGH);
	//delay(delayms);
}

/************************************************************************/
/* Clear Display                                                        */
/************************************************************************/
static inline void Clear()
{
	pinsMultiWrite(plex_pins, LOW);
	pinsMultiWrite(int_data_pins, LOW);
}
/************************************************************************/
/* Display data                                                         */
/************************************************************************/
static inline void Display(uint8_t *p)
{
	uint8_t byte, bit, index = 0;
	for(index=0; index<5; index++){
		pinsMultiWrite(plex_pins, LOW);
		//////////////////////////////////////////////////////
		pinWrite(latch_pin, HIGH);
		for(byte=0; byte < 4; byte++)
			USART_Transmit((unsigned char)*p++);
		pinWrite(latch_pin, LOW);	
		while(pinRead(clock_pin));
		///////////////////////////////////////////////////////
		for(bit = 0; bit < 8 ; bit++)
			pinWrite(int_data_pins[bit], (*p & (1  << bit)));
		p++;
		///////////////////////////////////////////////////////											
		pinWrite(plex_pins[index], HIGH);
		_delay_ms(5);
		Clear();
	}
	delayms = 1;
}
/************************************************************************/
/* initialize                                                           */
/************************************************************************/
void _init()
{
	pinSet(latch_pin, OUTPUT); /*latch pin*/
	pinSet(clock_pin, INPUT); /*clock pin*/
	pinsMultiSet(int_data_pins, OUTPUT); /*internal data pins*/
	pinsMultiSet(plex_pins, OUTPUT); /* multiplexing pins*/
	//InterruptInit();
	USART_Init(57600);
}

/************************************************************************/
/* initialize buffer                                                    */
/************************************************************************/
inline void InitBuffer()
{
	uint8_t *bf = buffer;
	uint8_t index = 0;
	for(index=0; index<25; index++) *bf++ = 0x00;
}
/************************************************************************/
/* Hold display for specified time                                      */
/************************************************************************/
static inline void HoldDisplay(uint8_t *data, uint8_t period)
{
	uint8_t x = 0;
	for(x=0; x< period; x++){
		Display(data);
	}
}
/************************************************************************/
/*split while displaying                                                */
/************************************************************************/
void Splitter(uint8_t *p, uint8_t delay)
{
	uint8_t rt = 0;
	delayms = delay;
	for(rt=0; rt<100; rt++){
		Display(p);
	}
}
/************************************************************************/
/* scroll right                                                         */
/************************************************************************/
void ScrollRight(uint8_t *p, uint8_t time)
{
	int x ,y,c = 0;
	uint8_t *bf = buffer; 
	HoldDisplay(p, 10);
	for(c=0; c<4; c++)
	{
		for(y=0; y<5; y++)
		{
			*bf = *p << 1;
			for(x=1; x<5; x++)
			{
				bf++; p++;
				*bf = (*p << 1) | (*(p+1) >> 7);		 
			}
			bf++; p++;
		}	
		HoldDisplay(buffer, time);
		p = bf = buffer;
	}	
	HoldDisplay(buffer, 50);
}

/************************************************************************/
/* scroll left                                                          */
/************************************************************************/
void ScrollLeft(uint8_t *p, uint8_t time)
{
	int x ,y,c = 0;
	uint8_t *bf = buffer;
	uint8_t *bk = p;
	HoldDisplay(p, 10);
	for(c=0; c<4; c++)
	{
		for(y=0; y<5; y++)
		{
			*bf = *p >> 1;
			for(x=1; x<5; x++)
			{	
				bf++; p++;
				*bf = (*p >> 1);// | (*(p-1) << 7);
			}
			bf++; p++;
		}
		HoldDisplay(buffer, time);
		p = bf = buffer;
	}
	p = bk;
	for(x=2; x<23; x+=5)
	{
		*(bf+x) |= (*(p+(x-1)) << 4);
		*(bf+x+1) |= (*(p+(x)) << 4);
	}		
	HoldDisplay(buffer, 50);
}

/************************************************************************/
/* drop down effect                                                     */
/************************************************************************/
#define In  1;
#define Out 0;
void FallIn(uint8_t *p, uint8_t time)
{
	uint8_t *bf = buffer;
	uint8_t index, step = 0;
	for(step = 0; step < 4; step++)
	{
		for(index = 0; index < 25; index++)
		{
			switch (step)
			{
				case  0:
					if(index >= 5) *(bf+index) = 0x00;
					else *(bf + index) = *(p+(index+20));
				break;
				case 1:
					if(index >= 10) *(bf+index) = 0x00;
					else *(bf + index) = *(p+(index+15));
				break;
				case 2:
					if(index >= 15) *(bf+index) = 0x00;
					else *(bf + index) = *(p+(index+10));
				break;
				case 3:
					if(index >= 20) *(bf+index) = 0x00;
					else *(bf + index) = *(p+(index+5));
				break;
			}
			
		}
		HoldDisplay(bf, time);
	}
	HoldDisplay(p, 50);
}

/************************************************************************/
/* drop down effect                                                     */
/************************************************************************/
void FallOut(uint8_t *p, uint8_t time)
{
	uint8_t *bf = buffer;
	uint8_t index, step = 0;
	HoldDisplay(p, 20);
	for(step = 0; step <5 ; step++)
	{
		for(index = 0; index < 25; index++)
		{
			switch (step)
			{
				case 4:
				*(bf+index) = 0x00;
				break;
				case 3:
				if(index >= 5) *(bf+index) = 0x00;
				else *(bf + index) = *(p+(index+20));
				break;
				case 2:
				if(index >= 10) *(bf+index) = 0x00;
				else *(bf + index) = *(p+(index+15));
				break;
				case 1:
				if(index >= 15) *(bf+index) = 0x00;
				else *(bf + index) = *(p+(index+10));
				break;
				case 0:
				if(index >= 20) *(bf+index) = 0x00;
				else *(bf + index) = *(p+(index+5));
				break;
			}
			
		}
		HoldDisplay(bf, time);
	}
	HoldDisplay(bf, 50);
}

/************************************************************************/
/* type effect                                                          */
/************************************************************************/
void TypeIn(uint8_t *p, uint8_t time)
{
	InitBuffer();
	uint8_t *bf = buffer;
	uint8_t index, step, rep = 0;	
	for(step = 4; step >= 1 ; step--)
	{
		for(rep = 0; rep < 5; rep+=4)
		{
			for(index = step ; index < 25; index+=5)
			{
				*(bf+index) = *(p+index) &  (*(p+index) >> (4-rep));
			}
			HoldDisplay(bf, time);
		}
	}
	HoldDisplay(p, 50);
}
/************************************************************************/
/* Delete effect                                                         */
/************************************************************************/
void TypeOut(uint8_t *p, uint8_t time)
{
	InitBuffer();
	uint8_t *bf = buffer;
	uint8_t index, step, rep = 0;
	HoldDisplay(p, 10);
	for(index=0; index<25; index++) *(bf+index) = *(p+index);
	for(step = 0; step < 5 ; step++)
	{
		for(rep = 0; rep < 9; rep+=8)
		{
			for(index = step ; index < 25; index+=5)
			{
				*(bf+index) = *(p+index) & (0x0F >> rep);
			}
			HoldDisplay(bf, time);
		}
	}
	HoldDisplay(bf, 50);
}

/************************************************************************/
/*  info interrupt routine                                              */
/************************************************************************/
uint8_t flag_execute  = 0 ; /*routine is not executed first time*/
static inline void InfoRoutine()
{
	/************************************************************************/
	/* CARBON RICH PLANET                                                   */
	/************************************************************************/
	FallIn(carbon, 2);
	FallOut(rich, 2);
	TypeIn(planet, 2);
	TypeOut(planet, 2);
}

ISR(INT0_vect)
{
	if(flag_execute)
	{
		InfoRoutine();
	}		
	flag_execute = 1;
}

/************************************************************************/
/* default information                                                  */
/************************************************************************/
static inline void DefaultInfo()
{
	//////////////////////////////////////////////////////////////////////////
	/************************************************************************/
	/* IHUB - Scroll right effect                                           */
	/************************************************************************/
	ScrollRight(ihub, 2); /*ScrollRight*/
	/************************************************************************/
	/*OPEN NASA - ScrollRight and Left effect		                        */
	/************************************************************************/
	Splitter(open, 50);
	Splitter(nasa, 50);
	/************************************************************************/
	/* 55 CANCRI E - TypeIn, FallOut, FallIn, TypeOut effect                */
	/************************************************************************/
	TypeIn(cancri, 3);
	FallOut(cancri, 2);
	FallIn(cancri, 2);
	TypeOut(cancri, 2);
	//////////////////////////////////////////////////////////////////////////
}
/************************************************************************/
/* main function                                                        */
/************************************************************************/
int main(void)
{
	_init();
    while(1)
    {   
		//HoldDisplay(cancri,100);
		Display(cancri);
		//DefaultInfo();
    }
}