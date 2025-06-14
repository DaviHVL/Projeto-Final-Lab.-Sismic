// Bibliotecas importadas
#include <msp430f5529.h>
#include <msp430.h>
#include <stdint.h>
#include "intrinsics.h"

// Definição dos pinos do LED RGB
#define LEDRGB_RED BIT4
#define LEDRGB_GREEN BIT4
#define LEDRGB_BLUE BIT2

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

uint8_t active_time = 1; // Tempo de Estudo Total em Minutos
uint8_t rest_time = 1;   // Tempo de Descanso Total em Minutos

volatile uint16_t counting_active_time = 0; // Tempo de Estudo Passado em Segundos
volatile uint16_t counting_rest_time = 0;   // Tempo de Descanso Passado em Segundos

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

  // Loop Principal
  for(;;){
    lcdClear();
    lcdWrite("Estudo");
    P1OUT |= LEDRGB_GREEN;
    while((active_time*60) != counting_active_time);
    lcdClear();
    lcdWrite("Descanso");
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

void lcdClear(void) {
  lcdWriteByte(0x01, 0);     
  __delay_cycles(20000);         
}

// Interrupção realizada a cada 1 segundo
# pragma vector = TIMER0_A0_VECTOR
__interrupt void TA0CCR0_ISR(){
  if (operating_mode){
    counting_active_time += 1;
  } else{
    counting_rest_time += 1;
  }
}
