/*
 * ShiftExtense.c
 *
 * Created: 4/16/2013 18:41:52
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
 * code for e-clothing. developed as a part of open nasa challenge (spaceapps)
 * to celebrate 55 CANCRI E planet.                                                                      
*/

#include <avr/io.h>
#define F_CPU 16000000
#include <util/delay.h>
#include <avr/interrupt.h>

#include "pin.h"

/************************************************************************/
/* clock pin                                                            */
/************************************************************************/
#define  clockpin 39
/************************************************************************/
/*latch pin																*/
/************************************************************************/
#define latch_pin 38
/************************************************************************/
/*bit pins array                                                        */
/************************************************************************/
multiPins bit_pins[28] =
{
	1, 2, 3, 4, 5, 6, 7, 8,
	15, 16, 17, 18, 19, 20, 21, 22,
	23, 24, 25, 26, 27, 28, 29, 33,
	34, 35, 36, 37
};


/************************************************************************/
/* buffer to hold data pins                                             */
/************************************************************************/
uint8_t bf[28];
uint8_t *buffer = bf;

/************************************************************************/
/* initialize serial communication                                      */
/************************************************************************/
void USART_Init( unsigned int baud )
{
	/* Set baud rate */
	UBRRH = (unsigned char)(baud>>8);
	UBRRL = (unsigned char)baud;
	/* Enable receiver */
	UCSRB = (1<<RXEN);
	/* Set frame format: 8data, 2stop bit */
	UCSRC = (1<<URSEL)|(1<<USBS)|(3<<UCSZ0);
	PORTD |= (1 << PORTD0);
}
/************************************************************************/
/* receive character                                                    */
/************************************************************************/
unsigned char USART_Receive( void )
{
	/* Wait for data to be received */
	while ( !(UCSRA & (1<<RXC)) );
	/* Get and return received data from buffer */
	return UDR;
}
/************************************************************************/
/* Check if a character has been received                               */
/************************************************************************/
static inline uint8_t USART_IsCharReceived(void)
{
	return ((UCSRA & (1 << RXC)) ? 1 : 0);
}
/************************************************************************/
/* disable serial communication                                         S*/
/************************************************************************/
static inline void USART_Disable()
{
	UCSRB = 0;
	UCSRA = 0;
	UCSRC = 0;
	UBRRH = 0;
	UBRRL = 0;
	PORTD &= ~(1 << PORTD0);
}
/************************************************************************/
/*initialize                                                            */
/************************************************************************/
static inline void _init()
{
	pinSet(latch_pin, INPUT);
	pinSet(clockpin, OUTPUT);
	pinsMultiSet(bit_pins, OUTPUT);
	pinsMultiWrite(bit_pins, LOW);
	USART_Init(57600);
}
/************************************************************************/
/* Display the bits                                                     */
/************************************************************************/
uint8_t value[4] = {0xFF, 0xFF, 0xFF, 0xFF};
uint8_t data = 0;
static inline void Display()
{
	uint8_t bit, x =0;
	uint8_t iplex = 27;
	uint8_t *p = value;
	pinWrite(clockpin, HIGH);
	for(bit = 4; bit<8; bit++)
	{
		pinWrite(bit_pins[iplex], (*p & (1  << (7-bit))));
		iplex--;
	}
	for(x=1; x<4; x++)
	{
		p++;
		for(bit = 0; bit < 8 ; bit++)
		{
			pinWrite(bit_pins[iplex], (*p & (1  << (7-bit))));
			iplex--;	
		}
	}
	pinWrite(clockpin, LOW);
	data = 0;
}
/************************************************************************/
/* Receive sent data serially                                           */
/************************************************************************/
void SerialReceive()
{
	uint8_t i = 0;
	uint8_t *p = value;
	while(pinRead(latch_pin))
	{
		for(i=0; i<4; i++)
		{
			*p = (uint8_t)USART_Receive();
			data++; p++;			
		}
	}
	if(data > 3) Display();
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/	
int main(void)
{
	_init();
    while(1)
    {
		SerialReceive();
		//Display();
	}		
}
