// TableTrafficLight.c
// Runs on LM4F120 or TM4C123
// Index implementation of a Moore finite state machine to operate
// a traffic light.
// Daniel Valvano, Jonathan Valvano
// July 20, 2013

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2013
   Volume 1 Program 6.8, Example 6.4
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
   Volume 2 Program 3.1, Example 3.1

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// east facing red light connected to PB5
// east facing yellow light connected to PB4
// east facing green light connected to PB3
// north facing red light connected to PB2
// north facing yellow light connected to PB1
// north facing green light connected to PB0
// north facing car detector connected to PE1 (1=car present)
// east facing car detector connected to PE0 (1=car present)
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "tm4c123gh6pm.h"
#include "TExaS.h"

#define LIGHT                   (*((volatile unsigned long *)0x400050FC))
#define GPIO_PORTB_OUT          (*((volatile unsigned long *)0x400050FC)) // bits 5-0
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))
#define GPIO_PORTE_IN           (*((volatile unsigned long *)0x4002400C)) // bits 1-0
#define SENSOR                  (*((volatile unsigned long *)0x4002400C))

#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC2_GPIOE      0x00000010  // port E Clock Gating Control
#define SYSCTL_RCGC2_GPIOB      0x00000002  // port B Clock Gating Control


// Linked data structure
struct State {
  unsigned long Out; 
  unsigned long Time;  
  unsigned long Next[4];}; 
typedef const struct State STyp;
#define goWest			0
#define	goWestE			1
#define	goSouth			2
#define	goSouthE		3
#define	goWalk			4
#define	goWalkE1		5
#define goWalkE2		6
#define goWalkE3		7
#define	goWalkE4		8
#define	goWalkE5		9
#define	goWalkE6		10
#define goWalkE7		11
#define goWalkE8		12
#define	goWalkEnd		13
#define standardWait	50
	
int West(void);
int WestE(void);
int South(void);
int SouthE(void);
int Walk(void);
int WalkE(void);
int	WalkEnd(void);
void InitPortsBEF(void);
int next;

int (*FSM[15])() = {West, WestE, South, SouthE, Walk, WalkE, WalkE, WalkE, WalkE, WalkE, WalkE, WalkE, WalkE, WalkE, WalkEnd};

int main(void){
	InitPortsBEF();
	SysTick_Init();
	next = 0;
  while(1){
		next = (*FSM[next])();
  }
}

int West() {
	GPIO_PORTB_DATA_R	|= 0x0C;		// set westbound to green, southbound to red
	GPIO_PORTB_DATA_R	&= 0xCC;		// clears other color street lights
	GPIO_PORTF_DATA_R	|= 0x02;		// set pedestrian light to red
	GPIO_PORTF_DATA_R	&= 0xFE;		// clear pedestrian lights
	SysTick_Wait10ms(standardWait);				// wait 10 s
	int input = GPIO_PORTE_DATA_R;
	return (((input & 0x02) >> 1) | ((input & 0x04) >> 2)) + next;
}

int	WestE() {
	GPIO_PORTB_DATA_R |= 0x14;		// set westbound to yellow, southbound to red
	GPIO_PORTB_DATA_R &= 0xD4;		// clears other street color lights
	SysTick_Wait10ms(standardWait/2);				// wait 5 s
	int	input = GPIO_PORTE_DATA_R;
	input = (input & 0x06) >> 1;
	return (((input & 0x02) >> 1) | input) + next;
	//if (input & 0x04) {return next+3;}
	//else {return next+1;}
}

int South(){
	GPIO_PORTB_DATA_R |= 0x21;		// set westbound to red, southbound to green
	GPIO_PORTB_DATA_R	&= 0xE1;		// clear other street color lights
	GPIO_PORTF_DATA_R	|= 0x02;		// set pedestrian light to red
	GPIO_PORTF_DATA_R	&= 0xFE;		// clear pedestrian lights
	SysTick_Wait10ms(standardWait);				// wait 10 s
	int input = GPIO_PORTE_DATA_R;
	return (((input & 0x01)) | ((input & 0x04) >> 2)) + next;
}

int SouthE() {
	GPIO_PORTB_DATA_R	|= 0x22;		// set westbound to red, southbound to yellow
	GPIO_PORTB_DATA_R	&= 0xE2;		// clear other street color lights
	SysTick_Wait10ms(standardWait/2);				// wait 5 s
	int input = GPIO_PORTE_DATA_R;
	return (input & 0x04) & ((~input & 0x01) << 2);
	//if(input & 0x01) {return 0;}
	//else	{return next+1;}
}

int Walk(){
	GPIO_PORTB_DATA_R |= 0x24;		// sets red streetligths
	GPIO_PORTB_DATA_R  &= 0xE4;		// clears other color street lights
	GPIO_PORTF_DATA_R	 |= 0x08;		// sets green walk light
	GPIO_PORTF_DATA_R	 &= 0xF9;		// clears other color pedestrian lights
	SysTick_Wait10ms(standardWait/2); 			// wait 5 s	
	int input = GPIO_PORTE_DATA_R;
	return ((input & 0x01) | ((input & 0x02) >> 1)) + next; // returns next state # based on inputs
}

int	WalkE()
{
	GPIO_PORTF_DATA_R	&=	0xF3;		// clears all pedestrian lights except for red
	GPIO_PORTF_DATA_R	= ((~GPIO_PORTF_DATA_R) & 0x02) + (GPIO_PORTF_DATA_R & ~0x02);		// toggle red pedestrian light
	SysTick_Wait10ms(standardWait/15);				// wait 1 s
	return next+1;
}

int WalkEnd()
{
	GPIO_PORTF_DATA_R	&=	0xF3;		// clears all pedestrian lights except for red
	GPIO_PORTF_DATA_R	= ((~GPIO_PORTF_DATA_R) & 0x02) + (GPIO_PORTF_DATA_R & ~0x02);		// toggle red pedestrian light
	SysTick_Wait10ms(standardWait/15);				// wait 1 s
	int input = GPIO_PORTE_DATA_R;
	//if(input & 0x02) {return 2;}
	//else	{return 0;}
	return (input & 0x02);
}

void InitPortsBEF(){
	volatile unsigned long delay;
  PLL_Init();       // 80 MHz, Program 10.1
  SysTick_Init();   // Program 10.2
  SYSCTL_RCGC2_R |= 0x32;      // 1) B E F
  delay = SYSCTL_RCGC2_R;      // 2) no need to unlock
  GPIO_PORTE_AMSEL_R &= ~0x03; // 3) disable analog function on PE1-0
  GPIO_PORTE_PCTL_R &= ~0x000000FF; // 4) enable regular GPIO
  GPIO_PORTE_DIR_R &= ~0x03;   // 5) inputs on PE1-0
  GPIO_PORTE_AFSEL_R &= ~0x03; // 6) regular function on PE1-0
  GPIO_PORTE_DEN_R |= 0x03;    // 7) enable digital on PE1-0
  GPIO_PORTB_AMSEL_R &= ~0x3F; // 3) disable analog function on PB5-0
  GPIO_PORTB_PCTL_R &= ~0x00FFFFFF; // 4) enable regular GPIO
  GPIO_PORTB_DIR_R |= 0x3F;    // 5) outputs on PB5-0
  GPIO_PORTB_AFSEL_R &= ~0x3F; // 6) regular function on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;    // 7) enable digital on PB5-0
	GPIO_PORTF_LOCK_R = 0x4C4F434B;
	GPIO_PORTF_CR_R = 0xFF;
	GPIO_PORTF_AMSEL_R = 0;
	GPIO_PORTF_PCTL_R = 0;
	GPIO_PORTF_DIR_R = 0x0E;
	GPIO_PORTF_AFSEL_R = 0;
	GPIO_PORTF_PUR_R = 0x11;
	GPIO_PORTF_DEN_R = 0xFF;
}

