#include "board_config.h"
#include "spi.h"

void spiInit()
{
  RCC->APBENR2 |= RCC_APBENR2_SPI1EN; // enable SPI1 clock
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;  // enable GPIOA clock

  // PA1 -> SPI1_SCK
  GPIOA->MODER &= ~GPIO_MODER_MODE1_0;       // set mode to "alternate-function push-pull", bit 0 is "0"
  GPIOA->MODER |= GPIO_MODER_MODE1_1;        // set mode to "alternate-function push-pull", bit 1 is "1"
  GPIOA->AFR[0] &= GPIO_AFRL_AFSEL1_0;       // set alternate-function to SPI1(AF0), bit 0 is "0"
  GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL1_1;      // set alternate-function to SPI1(AF0), bit 1 is "0"
  GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL1_2;      // set alternate-function to SPI1(AF0), bit 2 is "0"
  GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL1_3;      // set alternate-function to SPI1(AF0), bit 3 is "0"
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT1;         // set output type to "push-pull (reset-state)"
  GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED1_0;  // set output speed to slow, bit 0 is "1"
  GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED1_1; // set output speed to slow, bit 1 is "0"

  // PA2 -> SPI1_MOSI
  GPIOA->MODER &= ~GPIO_MODER_MODE2_0;       // set mode to "alternate-function push-pull", bit 0 is "0"
  GPIOA->MODER |= GPIO_MODER_MODE2_1;        // set mode to "alternate-function push-pull", bit 1 is "1"
  GPIOA->AFR[0] &= GPIO_AFRL_AFSEL2_0;       // set alternate-function to SPI1(AF0), bit 0 is "0"
  GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL2_1;      // set alternate-function to SPI1(AF0), bit 1 is "0"
  GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL2_2;      // set alternate-function to SPI1(AF0), bit 2 is "0"
  GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL2_3;      // set alternate-function to SPI1(AF0), bit 3 is "0"
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT2;         // set output type to "push-pull (reset-state)"
  GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED2_0;  // set output speed to slow, bit 0 is "1"
  GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED2_1; // set output speed to slow, bit 1 is "0"

  // PA6 -> SPI1_MISO
  GPIOA->MODER &= ~GPIO_MODER_MODE6_0;       // set mode to "alternate-function push-pull", bit 0 is "0"
  GPIOA->MODER |= GPIO_MODER_MODE6_1;        // set mode to "alternate-function push-pull", bit 1 is "1"
  GPIOA->AFR[0] &= GPIO_AFRL_AFSEL6_0;       // set alternate-function to SPI1(AF0), bit 0 is "0"
  GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL6_1;      // set alternate-function to SPI1(AF0), bit 1 is "0"
  GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL6_2;      // set alternate-function to SPI1(AF0), bit 2 is "0"
  GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL6_3;      // set alternate-function to SPI1(AF0), bit 3 is "0"
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT6;         // set output type to "push-pull (reset-state)"
  GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED6_0;  // set output speed to slow, bit 0 is "1"
  GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED6_1; // set output speed to slow, bit 1 is "0"

  // setup SPI1
  SPI1->CR1 &= ~SPI_CR1_SPE;      // disable SPI before changing settings
  SPI1->CR1 &= ~SPI_CR1_BIDIMODE; // enable 2-line unidirectional data mode
  SPI1->CR1 &= ~SPI_CR1_CRCEN;    // disable hardware CRC
  SPI1->CR1 &= ~SPI_CR1_LSBFIRST; // send data MSB first
  SPI1->CR1 |= SPI_CR1_BR_0;      // set baudrate scaler to 256, bit 0 is "1"
  SPI1->CR1 |= SPI_CR1_BR_1;      // set baudrate scaler to 256, bit 1 is "1"
  SPI1->CR1 |= SPI_CR1_BR_2;      // set baudrate scaler to 256, bit 2 is "1"
  SPI1->CR2 |= SPI_CR2_DS_0;      // set data size to 8 bit (0111), bit 0 is "1"
  SPI1->CR2 |= SPI_CR2_DS_1;      // set data size to 8 bit (0111), bit 1 is "1"
  SPI1->CR2 |= SPI_CR2_DS_2;      // set data size to 8 bit (0111), bit 2 is "1"
  SPI1->CR2 &= ~SPI_CR2_DS_3;     // set data size to 8 bit (0111), bit 3 is "0"
  SPI1->CR1 |= SPI_CR1_MSTR;      // configure as SPI master
  SPI1->CR1 &= ~SPI_CR1_CPOL;     // clock line is low when idle
  SPI1->CR1 &= ~SPI_CR1_CPHA;     // first clock transition is the first edge
  SPI1->CR1 |= SPI_CR1_SSM;       // set software management of slave (needed for master mode)
  SPI1->CR1 |= SPI_CR1_SSI;       // internal slave select (needed for master mode)
  SPI1->CR2 |= SPI_CR2_SSOE;      // enable slave select output
  SPI1->CR2 &= ~SPI_CR2_FRF;      // use Motorola SPI mode
  SPI1->CR2 |= SPI_CR2_NSSP;      // generate NSS pulse on slave select line
  SPI1->CR1 |= SPI_CR1_SPE;       // enable SPI1
}

void spiTranferByte(uint8_t b)
{
  while (!(SPI1->SR & SPI_SR_TXE))
  {
    // wait for transmit register empty
  }
  *((volatile uint8_t *)&(SPI1->DR)) = b;
}