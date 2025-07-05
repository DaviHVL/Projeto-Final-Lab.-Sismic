// Bibliotecas importadas
#include <msp430f5529.h>
#include <msp430.h>
#include <stdio.h>
#include <stdint.h>
#include "intrinsics.h"

// Definição dos pinos do LED RGB
#define LEDRGB_RED BIT4
#define LEDRGB_GREEN BIT4
#define LEDRGB_BLUE BIT2

// Definição do Buzzer
#define BUZZER BIT6


// Endereço I2C do módulo LCD
#define LCD_I2C_ADDR 0x27 // talvez o seu seja 0x3F

// Definição dos pinos no PCF8574
// P7-P4: D7-D4 (dados)
// P3: BL (backlight)
// P2: EN (enable)
// P1: RW (read/write)
// P0: RS (register select)
#define LCD_RS      0x01    // 0: Comando, 1: Dados
#define LCD_RW      0x02    // 0: Escrita, 1: Leitura
#define LCD_EN      0x04    // Pulso de habilitação
#define LCD_BL      0x08    // Controle da luz de fundo
#define LCD_D4      0x10    // Bit de dados 4
#define LCD_D5      0x20    // Bit de dados 5
#define LCD_D6      0x40    // Bit de dados 6
#define LCD_D7      0x80    // Bit de dados 7

// Protótipos das funções
void initI2C_Master(void);
uint8_t i2cSend(uint8_t slaveAddr, uint8_t data);
uint8_t lcdWriteNibble (uint8_t nibble, uint8_t isChar);
uint8_t lcdWriteByte (uint8_t byte, uint8_t isChar);
void lcdInit(void);
void lcdWrite(char* str);

uint8_t backlight_flag = LCD_BL; // Variável global para a luz de fundo

uint8_t operating_mode = 1; // 1: Estudo e 0: Descanso

uint8_t active_time = 2; // Tempo de Estudo Total em Minutos
uint8_t rest_time = 2;   // Tempo de Descanso Total em Minutos


volatile uint16_t counting_active_time = 0; // Tempo de Estudo Passado em Segundos
volatile uint16_t counting_rest_time = 0;   // Tempo de Descanso Passado em Segundos
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
  WDTCTL = WDTPW | WDTHOLD;  // Travar watchdog 

  // Configuração dos pinos do LED RGB
  P2DIR |= LEDRGB_RED;
  P2SEL &= ~(LEDRGB_RED);

  P1DIR |= LEDRGB_GREEN;
  P1SEL &= ~(LEDRGB_GREEN);

  P1DIR |= LEDRGB_BLUE;
  P1SEL &= ~(LEDRGB_BLUE);

  P2OUT &= ~(LEDRGB_RED);
  P1OUT &= ~(LEDRGB_GREEN);
  P1OUT &= ~(LEDRGB_BLUE);

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
                       


  // Configuração do TimerA0
  TA0CTL = 	TASSEL__ACLK | MC__UP;
	TA0CCR0 = 32768 - 1;				// Conta 1 segundo
  TA0CCTL0 = CCIE;

  // Configura os pinos GPIO para I2C
  // UCB0: P3.0 = SDA, P3.1 = SCL
  P3SEL |= BIT0 | BIT1;                   // Atribui os pinos ao módulo I2C UCB0

  // Inicializa o módulo I2C
  initI2C_Master();

  // Esperar um pouco antes de realizar outra ação
  __delay_cycles(20000);

  // Inicializar LCD
  lcdInit();

  // Limpeza do LCD
  lcdClear();

  // Habilitação Global das Interrupções
  __enable_interrupt(); 

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
    start_timer_flag = 0;
    pause_timer_flag = 0;
    operation_mode_flag = 0;
    if (reset_timer_flag){
      reset_timer_flag = 0;
    }
    else{
      lcdClear();
      lcdWrite("Escolha o modo");
    }
  }
}

void initI2C_Master(void) {
  UCB0CTL1 |= UCSWRST;                    // Reseta para iniciar a configuração
  UCB0CTL0 = UCMST | UCMODE_3 | UCSYNC;   // Mestre, I2C (MODE = 3), síncrono
  UCB0CTL1 = UCSSEL_2 | UCSWRST;          // Usa SMCLK (1 MHz), mantém reset
  UCB0BR0 = 10;                           // SMCLK/10 = 100 kHz
  UCB0BR1 = 0;
  UCB0CTL1 &= ~UCSWRST;                   // Limpa reset para terminar a configuração
}


uint8_t i2cSend(uint8_t slaveAddr, uint8_t data) {
    UCB0IFG &= ~(UCTXIFG);
    UCB0I2CSA = slaveAddr;
    UCB0CTL1 |= UCTR|UCTXSTT; // Mestre como transmissor e mandando START
    while((UCB0IFG & UCTXIFG) == 0); // Enquanto o buffer está cheio
    UCB0TXBUF = data;
    while((UCB0CTL1 & UCTXSTT) == 1);
    int answer;
    if((UCB0IFG & UCNACKIFG) == 0){
      while((UCB0IFG & UCTXIFG) == 0);
    }
    answer = UCB0IFG & UCNACKIFG;
    UCB0CTL1 |= UCTXSTP;
    while ((UCB0CTL1 & UCTXSTP) == 1);
    return answer;  
}

uint8_t lcdWriteNibble (uint8_t nibble, uint8_t isChar){
  // Vamos convencionar que os 4 bits que serão enviados são os menos significativos da variável nibble
  // Lembre-se de que os bits de dados são os 4 bits altos da mensagem
  // A variável global backlight_flag indica se queremos acender a luz de fundo ou não
  uint8_t message = (nibble << 4) | backlight_flag ;

  // Se isChar = 1, RS=1
  if (isChar){
    message |= LCD_RS;
  }

  // Preparar para transmissão
  uint8_t nack = i2cSend(LCD_I2C_ADDR, message);

  // Se o módulo LCD respondeu, continuar
  if(!nack){
    message |= LCD_EN;
    nack = i2cSend(LCD_I2C_ADDR, message);

    if(!nack){
      message &= ~LCD_EN;
      nack = i2cSend(LCD_I2C_ADDR, message);
    }
  }

  return nack;
}

uint8_t lcdWriteByte (uint8_t byte, uint8_t isChar){
  // Enviar bits mais significativos primeiro
  uint8_t nibble = (byte >> 4);

  uint8_t nack = lcdWriteNibble(nibble, isChar);

  // Se bem sucedido, enviar os bits menos significativos
  if (!nack){
    nibble = byte;
    nack = lcdWriteNibble(nibble, isChar);
  }

  return nack;
}

void lcdInit (void){
  // Enviar esse nibble três vezes para garantir que chegamos no modo de 8 bits
  uint8_t nibble = 3;

  lcdWriteNibble(nibble, 0);
  __delay_cycles(20000);
  lcdWriteNibble(nibble, 0);
  __delay_cycles(20000);
  lcdWriteNibble(nibble, 0);
  __delay_cycles(20000);

  // Na quarta vez, enviar enviar 2 para colocar no modo 4 bits
  nibble = 2;
  lcdWriteNibble(nibble, 0);
  __delay_cycles(20000);

  // Configurar display (display ligado, cursor piscando)
  uint8_t byte = 0x0F;
  lcdWriteByte(byte, 0);
  __delay_cycles(20000);

  // Apagar tudo e levar o cursor para o início
  byte = 0x01;
  lcdWriteByte(byte, 0);
  __delay_cycles(20000);
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

/* void lcdWrite(char* str){
  uint8_t row = 0;
  uint8_t col = 0;

  while (*str != '\0') {
    lcdWriteByte(*str, 1); 

    col++;
    if (col >= 16) {
      col = 0;
      row++;
      if (row >= 2) {
        break; 
      }
    }

    str++;
  }
    
}
*/

void lcdClear(void) {
  lcdWriteByte(0x01, 0);     
  __delay_cycles(20000);         
}

void lcdClear_sec(void) {
  int i;
  lcdWriteByte(0xC0, 0);
  // Apaga o que estava escrito na segunda linha do LCD
  for (i = 0; i < 16; i++){
    lcdWriteByte(' ', 1);
  }
  lcdWriteByte(0xC0, 0);
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
