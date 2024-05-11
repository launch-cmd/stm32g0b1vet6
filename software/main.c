#include <FreeRTOS.h>
#include "FreeRTOS_IP.h"
#include "gpio.h"
#include "task.h"
#include "log.h"
#include "board_config.h"
#include "display.h"
#include "dht22.h"

static StaticTask_t mainTaskBuffer;
static StackType_t mainTaskStack[configMINIMAL_STACK_SIZE * 2];

/* Function that implements the task being created. */
void mainTask(void *pvParameters)
{
  LOG_INFO("Starting mainTask.");
  uint16_t led = PIN('F', 13);        // Blue LED
  gpioSetMode(led, GPIO_MODE_OUTPUT); // Set blue LED to output mode
  gpioWrite(led, true);               // active low

  // start other tasks
  initDisplayTask();
  initDht22Task();

  bool statusLedIsOn = false;
  for (;;)
  {
    gpioWrite(led, statusLedIsOn);
    statusLedIsOn = !statusLedIsOn;
    vTaskDelay(pdMS_TO_TICKS(500));
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