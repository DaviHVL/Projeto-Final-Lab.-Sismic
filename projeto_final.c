#include <msp430f5529.h>

#define LEDRGB_RED BIT4
#define LEDRGB_GREEN BIT4
#define LEDRGB_BLUE BIT2

void main(void){
  WDTCTL = WDTPW | WDTHOLD;  // Travar watchdog 

  P2DIR |= LEDRGB_RED;
  P2SEL &= ~(LEDRGB_RED);

  P1DIR |= LEDRGB_GREEN;
  P1SEL &= ~(LEDRGB_GREEN);

  P1DIR |= LEDRGB_BLUE;
  P1SEL &= ~(LEDRGB_BLUE);

  for(;;){
  }
}

