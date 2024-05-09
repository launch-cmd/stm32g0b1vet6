#include <FreeRTOS.h>
#include "task.h"
#include "board_config.h"
#include "spi.h"

void spiInit()
{
  RCC->IOPENR |= RCC_IOPENR_GPIOBEN; // enable GPIOB clock

  // PB10 -> SPI2_SCK
  GPIOB->MODER &= ~GPIO_MODER_MODE10_0;      // set mode to "alternate-function push-pull", bit 0 is "0"
  GPIOB->MODER |= GPIO_MODER_MODE10_1;       // set mode to "alternate-function push-pull", bit 1 is "1"
  GPIOB->AFR[1] |= GPIO_AFRH_AFSEL10_0;      // set alternate-function to SPI2(AF5), bit 0 is "1"
  GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL10_1;     // set alternate-function to SPI2(AF5), bit 1 is "0"
  GPIOB->AFR[1] |= GPIO_AFRH_AFSEL10_2;      // set alternate-function to SPI2(AF5), bit 2 is "1"
  GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL10_3;     // set alternate-function to SPI2(AF5), bit 3 is "0"
  GPIOB->OTYPER &= ~GPIO_OTYPER_OT10;        // set output type to "push-pull (reset-state)"
  GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED10_0; // set output speed to high speed, bit 0 is "1"
  GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED10_1; // set output speed to high speed, bit 1 is "1"
  GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD10_0;      // enable pull-down(0b10), bit 0 is "0"
  GPIOB->PUPDR |= GPIO_PUPDR_PUPD10_1;       // enable pull-down(0b10), bit 1 is "1"
  GPIOB->ODR &= ~GPIO_ODR_OD10;              // set low signal to match SPI settings

  // PB11 -> SPI2_MOSI
  GPIOB->MODER &= ~GPIO_MODER_MODE11_0;      // set mode to "alternate-function push-pull", bit 0 is "0"
  GPIOB->MODER |= GPIO_MODER_MODE11_1;       // set mode to "alternate-function push-pull", bit 1 is "1"
  GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL11_0;     // set alternate-function to SPI2(AF0), bit 0 is "0"
  GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL11_1;     // set alternate-function to SPI2(AF0), bit 1 is "0"
  GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL11_2;     // set alternate-function to SPI2(AF0), bit 2 is "0"
  GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL11_3;     // set alternate-function to SPI2(AF0), bit 3 is "0"
  GPIOB->OTYPER &= ~GPIO_OTYPER_OT11;        // set output type to "push-pull (reset-state)"
  GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED11_0; // set output speed to high speed, bit 0 is "1"
  GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED11_1; // set output speed to high speed, bit 1 is "1"

  // Configure SPI2 (see 35.5.7)
  RCC->APBENR1 |= RCC_APBENR1_SPI2EN;    // enable SPI2 clock
  RCC->APBRSTR1 |= RCC_APBRSTR1_SPI2RST; // reset SPI2 peripheral
  for (int i = 0; i < 10000; i++)
  {
    // small delay before clearing reset
  }
  RCC->APBRSTR1 &= ~RCC_APBRSTR1_SPI2RST; // clear reset SPI2 peripheral
  SPI2->CR1 &= ~SPI_CR1_SPE;              // disable SPI before changing settings

  // CR1
  SPI2->CR1 &= ~SPI_CR1_BR_0;     // set baudrate scaler to 8(0b010), bit 0 is "0"
  SPI2->CR1 &= ~SPI_CR1_BR_1;     // set baudrate scaler to 8(0b010), bit 1 is "1"
  SPI2->CR1 &= ~SPI_CR1_BR_2;     // set baudrate scaler to 8(0b010), bit 2 is "0"
  SPI2->CR1 &= ~SPI_CR1_CPOL;     // clock line is low when idle
  SPI2->CR1 &= ~SPI_CR1_CPHA;     // first clock transition is the first edge
  SPI2->CR1 |= SPI_CR1_BIDIMODE;  // 1-line bidirectional data mode selected
  SPI2->CR1 |= SPI_CR1_BIDIOE;    // transmit only (MOSI) is used
  SPI2->CR1 &= ~SPI_CR1_LSBFIRST; // send data MSB first
  SPI2->CR1 &= ~SPI_CR1_CRCEN;    // disable hardware CRC
  SPI2->CR1 |= SPI_CR1_SSM;       // set software management of slave (needed for master mode)
  SPI2->CR1 |= SPI_CR1_SSI;       // internal slave select (needed for master mode)
  SPI2->CR1 |= SPI_CR1_MSTR;      // configure as SPI master

  // CR2
  SPI2->CR2 |= SPI_CR2_DS_0;  // set data size to 8 bit (0b0111), bit 0 is "1"
  SPI2->CR2 |= SPI_CR2_DS_1;  // set data size to 8 bit (0b0111), bit 1 is "1"
  SPI2->CR2 |= SPI_CR2_DS_2;  // set data size to 8 bit (0b0111), bit 2 is "1"
  SPI2->CR2 &= ~SPI_CR2_DS_3; // set data size to 8 bit (0b0111), bit 3 is "0"
  SPI2->CR2 &= ~SPI_CR2_SSOE; // disable slave select output
  SPI2->CR2 &= ~SPI_CR2_FRF;  // use Motorola SPI mode
  SPI2->CR2 &= ~SPI_CR2_NSSP; // don't generate NSS pulse on slave select line
  SPI2->CR1 |= SPI_CR1_SPE;   // enable SPI2
}

void spiTranferByte(uint8_t b)
{
  *((volatile uint8_t *)&(SPI2->DR)) = b;
}

void spiWaitForIdle()
{
  while ((SPI2->SR & SPI_SR_TXE) == 0)
  {
    // wait for transmit register empty
  }
  while ((SPI2->SR & SPI_SR_FTLVL) != 0)
  {
    // wait for TX fifo to be empty
  }
  while ((SPI2->SR & SPI_SR_BSY))
  {
    // wait for busy flag to clear
  }
}