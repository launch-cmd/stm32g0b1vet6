#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>
#include <FreeRTOS.h>
#include "FreeRTOS_IP.h"
#include "gpio.h"
#include "task.h"
#include "log.h"
#include "board_config.h"
#include "spi.h"
#include "lv_conf.h"

#define LV_CONF_INCLUDE_SIMPLE
#include "lvgl.h"

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

    spiTranferByte('W');
    spiTranferByte('A');
    spiTranferByte('N');
    spiTranferByte('D');
    spiTranferByte('E');
    spiTranferByte('R');

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
  
  // init display
  spiInit();
  lv_init();
  const uint32_t horRes = 240;
  const uint32_t vertRes = 135;
  lv_color_t dispBuff[(horRes * vertRes) / 10]; // render in 10 lines
  lv_display_t *display = lv_display_create(horRes, vertRes);
  lv_display_set_buffers(display, dispBuff, NULL, sizeof(dispBuff), LV_DISP_RENDER_MODE_PARTIAL);

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