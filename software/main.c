#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>
#include <FreeRTOS.h>
#include "FreeRTOS_IP.h"
#include "gpio.h"
#include "task.h"
#include "log.h"
#include "board_config.h"

static void spiInit()
{

  RCC->IOPENR |= RCC_IOPENR_GPIOAEN; // enable GPIOA clock

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
  // Data Size: 8 bits
  // First bit: MSB first
  // Prescaler: 2
  // Clock Polarity: low
  // Clock phase: 1 edge
  // CRC: disabled
  // NSSP Mode: Enabled
  // NSS Signal Type: Software
  RCC->APBENR2 |= RCC_APBENR2_SPI1EN; // enable SPI1 clock

  SPI1->CR1 &= ~SPI_CR1_SPE;      // disable SPI before changing settings
  SPI1->CR1 &= ~SPI_CR1_BIDIMODE; // enable 2-line unidirectional data mode
  SPI1->CR1 &= ~SPI_CR1_CRCEN;    // disable hardware CRC
  SPI1->CR1 &= ~SPI_CR1_LSBFIRST; // send data MSB first
  SPI1->CR1 &= ~SPI_CR1_BR_0;     // set baudrate divider to 2, bit 0 is "0"
  SPI1->CR1 &= ~SPI_CR1_BR_1;     // set baudrate divider to 2, bit 1 is "0"
  SPI1->CR1 &= ~SPI_CR1_BR_2;     // set baudrate divider to 2, bit 2 is "0"
  SPI1->CR1 |= SPI_CR1_MSTR;      // configure as SPI master
  SPI1->CR1 &= ~SPI_CR1_CPOL;     // clock line is low when idle
  SPI1->CR1 &= ~SPI_CR1_CPHA;     // first clock transition is the first edge
  SPI1->CR1 |= SPI_CR1_SSM;       // set software management of slave (needed for master mode)
  SPI1->CR1 |= SPI_CR1_SSI;       // internal slave select (needed for master mode)
  SPI1->CR1 |= SPI_CR1_SPE;       // enable SPI1
}

static StaticTask_t mainTaskBuffer;
static StackType_t mainTaskStack[configMINIMAL_STACK_SIZE * 2];

/* Function that implements the task being created. */
void mainTask(void *pvParameters)
{
  LOG_INFO("Starting mainTask.");
  uint16_t led = PIN('F', 13);        // Blue LED
  gpioSetMode(led, GPIO_MODE_OUTPUT); // Set blue LED to output mode
  gpioWrite(led, true);               // active low

  bool statusLedIsOn = false;
  uint32_t counter = 0;
  double counter1 = 1.23456789;
  for (;;)
  {
    gpioWrite(led, statusLedIsOn);
    statusLedIsOn = !statusLedIsOn;

    LOG_DEBUG("counting: %ld  %lf.", counter, counter1);
    LOG_INFO("counting: %ld  %lf.", counter, counter1);
    LOG_WARN("counting: %ld  %lf.", counter, counter1);
    LOG_ERROR("counting: %ld  %lf.", counter, counter1);

    SPI1->DR = (uint8_t)'h';

    counter++;
    counter1++;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

static const uint8_t ucIPAddress[4] = {192, 168, 1, 250};
static const uint8_t ucNetMask[4] = {255, 255, 255, 0};
static const uint8_t ucGatewayAddress[4] = {192, 168, 1, 1};

/* The following is the address of an OpenDNS server. */
static const uint8_t ucDNSServerAddress[4] = {208, 67, 222, 222};

/* The MAC address array is not declared const as the MAC address will normally
be read from an EEPROM and not hard coded (in real deployed applications).*/
static uint8_t ucMACAddress[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};

void main(void)
{
  uartInit();
  uartEnable();
  spiInit();
  LOG_INFO("Starting main.");

  /* Initialise the TCP/IP stack. */
  FreeRTOS_IPInit(ucIPAddress,
                  ucNetMask,
                  ucGatewayAddress,
                  ucDNSServerAddress,
                  ucMACAddress);

  xTaskCreateStatic(mainTask, "Main", configMINIMAL_STACK_SIZE * 2,
                    NULL, tskIDLE_PRIORITY + 1, mainTaskStack, &mainTaskBuffer);

  vTaskStartScheduler();

  for (;;)
  {
    LOG_ERROR("After vTaskStartScheduler(), this should not happen!");
  }
}