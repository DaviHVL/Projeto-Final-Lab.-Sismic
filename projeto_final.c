#include <msp430f5529.h>
#include <msp430.h>
#include <stdint.h>

#define LEDRGB_RED BIT4
#define LEDRGB_GREEN BIT4
#define LEDRGB_BLUE BIT2

uint8_t operating_mode = 1; // 1: Estudo e 0: Descanso

uint8_t active_time = 1; // Tempo de Estudo Total em Minutos
uint8_t rest_time = 1;   // Tempo de Descanso Total em Minutos

volatile uint16_t counting_active_time = 0; // Tempo de Estudo Passado em Segundos
volatile uint16_t counting_rest_time = 0;   // Tempo de Descanso Passado em Segundos

void main(void){
  WDTCTL = WDTPW | WDTHOLD;  // Travar watchdog 

  P2DIR |= LEDRGB_RED;
  P2SEL &= ~(LEDRGB_RED);

  P1DIR |= LEDRGB_GREEN;
  P1SEL &= ~(LEDRGB_GREEN);

  P1DIR |= LEDRGB_BLUE;
  P1SEL &= ~(LEDRGB_BLUE);

  P2OUT &= ~(LEDRGB_RED);
  P1OUT &= ~(LEDRGB_GREEN);
  P1OUT &= ~(LEDRGB_BLUE);

  TA0CTL = 	TASSEL__ACLK | MC__UP;
	TA0CCR0 = 32768 - 1;				// Conta 1 segundo
  TA0CCTL0 = CCIE;

  __enable_interrupt(); 
  for(;;){
    P1OUT |= LEDRGB_GREEN;
    while((active_time*60) != counting_active_time);
    counting_active_time = 0;
    operating_mode = 0;
    P1OUT &= ~(LEDRGB_GREEN);
    P2OUT |= LEDRGB_RED;
    while((rest_time*60) != counting_rest_time);
    counting_rest_time = 0;
    operating_mode = 1;
    P2OUT &= ~(LEDRGB_RED);

  }
}

# pragma vector = TIMER0_A0_VECTOR
__interrupt void TA0CCR0_ISR(){
  if (operating_mode){
    counting_active_time += 1;
  } else{
    counting_rest_time += 1;
  }
}
