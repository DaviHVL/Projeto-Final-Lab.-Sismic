/*
    Das funções que já existiam, mudei a lcdWrite() para conseguir escrever na segunda linha
    (apesar de não precisar) e mudei a interrupção do Timer
*/


#include <stdio.h>

volatile uint16_t counted_active_time_min = 0; // Tempo de Estudo contado em Minutos
volatile uint16_t counted_active_time_sec = 0; // Tempo de Estudo contado em Segundos
volatile uint16_t counted_rest_time_min = 0; // Tempo de Descanso contado em Minutos
volatile uint16_t counted_rest_time_sec = 0; // Tempo de Estudo contado em Segundos

volatile uint8_t start_timer_flag = 0; // Determina se o timer deve iniciar a contagem
volatile uint8_t pause_timer_flag = 0; // Determina se o timer deve pausar a contagem
volatile uint8_t reset_timer_flag = 0; // Determina se o timer deve resetar a contagem
volatile uint8_t operation_mode_flag = 0; // Diz se o usuário apertou o botão para selecionar modo
volatile uint8_t houve_reset = 0; // Variável que mantém o "Reiniciar" no LCD caso reset desejado
volatile uint8_t minute_changed = 0; // Variável que indica quando houve mudança no minuto

void main(void){
// Configuração do buzzer
  P1DIR |= BUZZER;
  P1SEL &= ~(BUZZER);
  P1OUT &= ~(BUZZER);

  // Configuração dos botões de início/"despause" (P1.3) e de "pause" (P1.5)
  P1SEL &= ~(BIT3 | BIT5);             // Pino como GPIO
  P1DIR &= ~(BIT3 | BIT5);             // Pino como entrada
  P1REN |= (BIT3 | BIT5);              // Habilitar resistor de pull-up
  P1OUT |= (BIT3 | BIT5);
  P1IFG &= ~(BIT3 | BIT5);             // Limpa a flag
  P1IE |= (BIT3 | BIT5);               // Habilitar a interrupção dos pinos P1.3 e P1.5
  P1IES |= (BIT3 | BIT5);              // Interrupção na borda de descida

  // Configuração do botão de reset (P2.0)
  P2SEL &= ~BIT0;                      // Pino como GPIO
  P2DIR &= ~BIT0;                      // Pino como entrada
  P2REN |= BIT0;                       // Habilitar resistor de pull-up
  P2OUT |= BIT0;
  P2IFG &= ~BIT0;                      // Limpa a flag
  P2IE |= BIT0;                        // Habilitar interrupção do pino P2.0
  P2IES |= BIT0;                       // Interrupção na borda de descida


  // Configuração do botão para mudar o modo de operação (estudo/descanso) (P2.6)
  P2SEL &= ~BIT6;                      // Pino como GPIO
  P2DIR &= ~BIT6;                      // Pino como entrada
  P2REN |= BIT6;                       // Habilitar resistor de pull-up
  P2OUT |= BIT6;
  P2IFG &= ~BIT6;                      // Limpa a flag
  P2IE |= BIT6;                        // Habilitar interrupção do pino P2.6
  P2IES |= BIT6;                       // Interrupção na borda de descida


  // Criação de uma string para guardar o tempo contado
  char buffer[17];

  lcdWrite("Escolha o modo"); // Indica ao usuário para escolher o modo de operação

  // Loop Principal
  for(;;){
    while (!start_timer_flag){
      if (operation_mode_flag){
        __delay_cycles(10000);
        operating_mode = !operating_mode;
        if (!houve_reset){
          lcdClear();
        }
        else {
          lcdClear();
          lcdWrite_sec("Reiniciar");
        }
        if (operating_mode){
          lcdWrite("Estudo");
        }
        else{
          lcdWrite("Descanso");
        }
        operation_mode_flag = 0;
      }
      __delay_cycles(100000);
    }
      
    houve_reset = 0;
    lcdClear_sec();

    // Se o usuário escolheu o modo estudo
    if (operating_mode == 1){
      lcdClear();
      lcdWrite("Estudo");
      P1OUT |= LEDRGB_GREEN;
      while((active_time*60) != counting_active_time){
        if (minute_changed){
          lcdClear_sec();
          minute_changed = 0;
        }
        sprintf(buffer, "%dm%ds", counted_active_time_min, counted_active_time_sec);
        lcdWrite_sec(buffer);
        if (pause_timer_flag) {
          lcdClear_sec();
          sprintf(buffer, "PAUSADO %dm%ds", counted_active_time_min, counted_active_time_sec);
          lcdWrite_sec(buffer); // Escreve "PAUSADO - tempo passado(XmYs)" na segunda linha do LCD
          // Se está pausado e o usuário não apertou para reiniciar
          while (pause_timer_flag && !reset_timer_flag);
          // Se o usuário apertou para reiniciar
          if (reset_timer_flag) {
            lcdClear_sec();
            lcdWrite_sec("Reiniciar");
            counting_active_time = 0;
            counted_active_time_min = 0;
            counted_active_time_sec = 0;
            houve_reset = 1;
            break;
          }
          // Depois da pausa (sem reiniciar), voltar a contar
          lcdClear_sec();
        }
      }
      if ((active_time*60) == counting_active_time){
        lcdClear_sec();
        sprintf(buffer, "%dm%ds", counted_active_time_min, counted_active_time_sec);
        lcdWrite_sec(buffer);
        // Ligar o buzzer por 1 segundo
        P1OUT |= BUZZER;
        __delay_cycles(1000000);
        P1OUT &= ~(BUZZER);
        counting_active_time = 0;
        counted_active_time_min = 0;
        counted_active_time_sec = 0;
        P1OUT &= ~(LEDRGB_GREEN);
      }
    }

    // Se o usuário escolheu o modo descanso
    if (operating_mode == 0){
      lcdClear();
      lcdWrite("Descanso");
      P2OUT |= LEDRGB_RED;
      while((rest_time*60) != counting_rest_time){
        // Escrever quantos minutos se passaram
        if (minute_changed){
          lcdClear_sec();
          minute_changed = 0;
        }
        sprintf(buffer, "%dm%ds", counted_rest_time_min, counted_rest_time_sec);
        lcdWrite_sec(buffer);
        if (pause_timer_flag) {
          lcdClear_sec();
          sprintf(buffer, "PAUSADO %dm%ds", counted_rest_time_min, counted_rest_time_sec);
          lcdWrite_sec(buffer); // Escreve "PAUSADO - tempo passado (XmYs)" na segunda linha do LCD
          // Se está pausado e o usuário não apertou para reiniciar
          while (pause_timer_flag && !reset_timer_flag);
          // Se o usuário apertou para reiniciar
          if (reset_timer_flag) {
            lcdClear_sec();
            lcdWrite_sec("Reiniciar");
            counting_rest_time = 0;
            counted_rest_time_min = 0;
            counted_rest_time_sec = 0;
            houve_reset = 1;
            break;
          }
          // Depois da pausa (sem reiniciar), voltar a contar
          lcdClear_sec();
        }
      }
      if ((rest_time*60) == counting_rest_time){
        lcdClear_sec();
        sprintf(buffer, "%dm%ds", counted_rest_time_min, counted_rest_time_sec);
        lcdWrite_sec(buffer);
        // Ligar o buzzer por 1 segundo
        P1OUT |= BUZZER;
        __delay_cycles(1000000);
        P1OUT &= ~(BUZZER);
        counting_rest_time = 0;
        counted_rest_time_min = 0;
        counted_rest_time_sec = 0;
        P2OUT &= ~(LEDRGB_RED);
      }
    }
    // start_timer_flag = 0;
    pause_timer_flag = 0;
    operation_mode_flag = 0;
    // Apenas muda o modo automaticamente se o usuário não pediu reset
    if (!reset_timer_flag){
      if (operating_mode){
        operating_mode = 0;
      }
      else {
        operating_mode = 1;
      }
    }
    else{
      reset_timer_flag = 0;
    }

    /*if (reset_timer_flag){
      reset_timer_flag = 0;
    }
    else{
      lcdClear();
      lcdWrite("Escolha o modo");
    }
    */
  }
}

void lcdWrite(char* str){
    uint8_t col = 0;
    lcdWriteByte(0x80, 0);
     while (*str != '\0') {
        // Quando chegar na 16ª posição, muda para a segunda linha
        if (col == 16) {
            lcdWriteByte(0xC0, 0);
        }

        // Se já escreveu 32 caracteres (16 + 16), para
        if (col >= 32) {
            break;
        }
        lcdWriteByte(*str, 1);
        str++;
        col++;
    }
}

void lcdWrite_sec(char* str){
  uint8_t col = 0;
  lcdWriteByte(0xC0, 0); // Começa a escrever na segunda linha
  while (*str != '\0') {
    if (col >= 16){
      break;
    }
    lcdWriteByte(*str, 1);
    str++;
    col++;
  }
}

// Interrupção realizada a cada 1 segundo
# pragma vector = TIMER0_A0_VECTOR
__interrupt void TA0CCR0_ISR(){
  // O Timer só deve contar se a condição de start estiver ativada
  if ((!start_timer_flag)){
    return;
  }

  if (operating_mode){
    counting_active_time += 1;
    counted_active_time_sec += 1;
    if (counted_active_time_sec >= 60){
      minute_changed = 1;
      counted_active_time_min += 1;
      counted_active_time_sec = 0;
    }
  } 
  else{
    counting_rest_time += 1;
    counted_rest_time_sec += 1; 
    if (counted_rest_time_sec >= 60){
      minute_changed = 1;
      counted_rest_time_min += 1;
      counted_rest_time_sec = 0;
    }
  }

}

#pragma vector = PORT1_VECTOR
__interrupt void PORT1 (){
  if (P1IFG & BIT3){
    __delay_cycles(10000);
    if (!(P1IN & BIT3)){
      start_timer_flag = 1;
      pause_timer_flag = 0;
      reset_timer_flag = 0;
    }
    P1IFG &= ~BIT3;
  }
  if (P1IFG & BIT5){
    __delay_cycles(10000);
    if (!(P1IN & BIT5)){
      pause_timer_flag = 1;
      start_timer_flag = 0;
    }
    P1IFG &= ~BIT5;
  }
}

#pragma vector = PORT2_VECTOR
__interrupt void PORT2 (){
  if (P2IFG & BIT0){
    __delay_cycles(10000);
    if (!(P2IN & BIT0)){
      reset_timer_flag = 1;
      start_timer_flag = 0;
    }
    P2IFG &= ~BIT0;
  }
  if (P2IFG & BIT6){
    __delay_cycles(10000);
    if (!(P2IN & BIT6)){
      operation_mode_flag = 1;
    }
    P2IFG &= ~BIT6;
  }
}
