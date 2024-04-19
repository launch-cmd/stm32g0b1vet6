#include "board_config.h"
#include "log.h"

void uartInit()
{
  // setup TX pin
  // setup gpio
  RCC->IOPENR |= RCC_IOPENR_GPIOFEN;

  GPIOF->MODER &= ~GPIO_MODER_MODE5_0;
  GPIOF->MODER |= GPIO_MODER_MODE5_1;

  // TX and RX are swapped so PF5 is used instead of PF4 for TX.
  // set AF1 FOR USE AS UART TX according to datasheet page 63
  // to set AF1 program GPIO_AFRL_AFSEL5 to 0b0001 according to manual page 245
  GPIOF->AFR[0] |= GPIO_AFRL_AFSEL5_0;
  GPIOF->AFR[0] &= ~GPIO_AFRL_AFSEL5_1;
  GPIOF->AFR[0] &= ~GPIO_AFRL_AFSEL5_2;
  GPIOF->AFR[0] &= ~GPIO_AFRL_AFSEL5_3;
  GPIOF->OTYPER &= ~GPIO_OTYPER_OT4;         // set output type to output "push-pull (reset-state)"
  GPIOF->OSPEEDR |= GPIO_OSPEEDR_OSPEED4_0;  // set output speed to slow, bit 0 is "1"
  GPIOF->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED4_1; // set output speed to slow, bit 1 is "0"
  GPIOF->PUPDR &= ~GPIO_PUPDR_PUPD4;         // set port pull-up/pull-down to No pull-up, pull-down
  GPIOF->AFR[0] |= GPIO_AFRL_AFSEL4;         // set alternative function low register for gpio4

  // setup LPUART1
  // SET LPUART1SEL to 0b0 for PCLK clock source
  RCC->CCIPR &= ~RCC_CCIPR_LPUART1SEL_0;
  RCC->CCIPR &= ~RCC_CCIPR_LPUART1SEL_1;
  RCC->APBENR1 |= RCC_APBENR1_LPUART1EN; // enable LPUART1 clock
  LPUART1->CR1 &= ~USART_CR1_UE;         // disable LPUART1 to be able to change settings
  // SET USART_CR1_M to ‘0b00’: 1 Start bit, 8 Data bits, n Stop bit
  LPUART1->CR1 &= ~USART_CR1_M0;
  LPUART1->CR1 &= ~USART_CR1_M1;
  LPUART1->BRR = 35556;             // set baud rate to 115200
  LPUART1->CR1 |= USART_CR1_FIFOEN; // enable fifo
  // SET USART_CR3_TXFTCFG to '0b011' to reach 3/4 depth before sending data
  LPUART1->CR3 |= USART_CR3_TXFTCFG_0;
  LPUART1->CR3 |= USART_CR3_TXFTCFG_1;
  LPUART1->CR3 &= ~USART_CR3_TXFTCFG_2;
  LPUART1->CR1 |= USART_CR1_TE;   // enable transmitter
  LPUART1->CR2 |= USART_CR2_SWAP; // swap RX and TX pins
}

void uartEnable()
{
  LPUART1->CR1 |= USART_CR1_UE; // enable LPUART1

  while (!(LPUART1->ISR & USART_ISR_TEACK)) // wait for transmit ready
  {
  }
}

void uartWriteByte(char b)
{
  while (!(LPUART1->ISR & USART_ISR_TXE_TXFNF)) // wait for FIFO space to be available
  {
  }
  LPUART1->TDR = (uint8_t)b; // write byte
}

void uartWriteLine(const char *bytes)
{
  int charCounter = 0;

  while (bytes[charCounter] != '\0')
  {
    uartWriteByte(bytes[charCounter]);
    charCounter++;
  }
  uartWriteByte('\r');
  uartWriteByte('\n');
}