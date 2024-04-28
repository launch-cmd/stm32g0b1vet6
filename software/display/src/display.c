#include <FreeRTOS.h>
#include "task.h"
#include "log.h"
#include "lv_conf.h"
#define LV_CONF_INCLUDE_SIMPLE
#include "lvgl.h"
#include "spi.h"
#include "board_config.h"
#include "display.h"

static StaticTask_t displayTaskBuffer;
static StackType_t displayTaskStack[configMINIMAL_STACK_SIZE * 2];

const uint32_t horRes = 240;
const uint32_t vertRes = 135;
#define DISP_BUF_SIZE (32400) // 240 * 135

void displaySelect()
{
    GPIOA->ODR &= ~GPIO_ODR_OD3; // select display
    while ((GPIOA->IDR & GPIO_IDR_ID3))
    {
        // wait for CS to be low
    }
    for (int i = 0; i < 10; i++)
    {
        // wait for io
    }
}

void displayDeselect()
{
    GPIOA->ODR |= GPIO_ODR_OD3; // deselect display
    while ((GPIOA->IDR & GPIO_IDR_ID3) == 0)
    {
        // wait for CS to be high
    }
    for (int i = 0; i < 10; i++)
    {
        // wait for io
    }
}

void displayCommandMode()
{
    GPIOA->ODR &= ~GPIO_ODR_OD4; // set DC pin low to signal data
    while (GPIOA->IDR & GPIO_IDR_ID4)
    {
        // wait for DC to be low
    }
    for (int i = 0; i < 10; i++)
    {
        // wait for io
    }
}

void displayDataMode()
{
    GPIOA->ODR |= GPIO_ODR_OD4; // set DC pin high to signal data
    while ((GPIOA->IDR & GPIO_IDR_ID4) == 0)
    {
        // wait for DC to be high
    }
    for (int i = 0; i < 10; i++)
    {
        // wait for io
    }
}

void displaySendCommand(uint8_t data)
{
    displaySelect();
    displayCommandMode();
    spiTranferByte(data);
    spiWaitForIdle();
    displayDeselect();
}

void displaySendData(uint8_t data)
{
    displaySelect();
    displayDataMode();
    spiTranferByte(data);
    spiWaitForIdle();
    displayDeselect();
}

void displayReset()
{
    // send reset sequence
    GPIOA->ODR &= ~GPIO_ODR_OD0;
    vTaskDelay(pdMS_TO_TICKS(50));
    GPIOA->ODR |= GPIO_ODR_OD0;
    vTaskDelay(pdMS_TO_TICKS(50));
}

void displayPinsInit()
{
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN; // enable GPIOA clock

    // PA0 -> LCD_RST
    GPIOA->MODER |= GPIO_MODER_MODE0_0;        // set mode to "general purpose output", bit 0 is "1"
    GPIOA->MODER &= ~GPIO_MODER_MODE0_1;       // set mode to "general purpose output", bit 0 is "0"
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT0;         // set output type to "push-pull (reset-state)"
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED0_0; // set output speed to slow, bit 0 is "0"
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED0_1; // set output speed to slow, bit 1 is "0"

    // PA3 -> LCD_CS
    GPIOA->MODER |= GPIO_MODER_MODE3_0;        // set mode to "general purpose output", bit 0 is "1"
    GPIOA->MODER &= ~GPIO_MODER_MODE3_1;       // set mode to "general purpose output", bit 0 is "0"
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT3;         // set output type to "push-pull (reset-state)"
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED3_0; // set output speed to slow, bit 0 is "0"
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED3_1; // set output speed to slow, bit 1 is "0"
    GPIOA->ODR |= GPIO_ODR_OD3;                // deselect display

    // PA4 -> LCD_DC
    GPIOA->MODER |= GPIO_MODER_MODE4_0;        // set mode to "general purpose output", bit 0 is "1"
    GPIOA->MODER &= ~GPIO_MODER_MODE4_1;       // set mode to "general purpose output", bit 0 is "0"
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT4;         // set output type to "push-pull (reset-state)"
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED4_0; // set output speed to slow, bit 0 is "0"
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED4_1; // set output speed to slow, bit 1 is "0"
    GPIOA->ODR &= ~GPIO_ODR_OD4;               // enter command mode

    // PA5 -> LCD_BL
    GPIOA->MODER |= GPIO_MODER_MODE5_0;        // set mode to "general purpose output", bit 0 is "1"
    GPIOA->MODER &= ~GPIO_MODER_MODE5_1;       // set mode to "general purpose output", bit 0 is "0"
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;         // set output type to "push-pull (reset-state)"
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED5_0; // set output speed to slow, bit 0 is "0"
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED5_1; // set output speed to slow, bit 1 is "0"
    GPIOA->ODR |= GPIO_ODR_OD5;                // enable backlight
}

void displayInit()
{
    displaySendCommand(0x36); // memory data access control
    displaySendData(0x70);

    displaySendCommand(0x3A); // interface pixel format
    displaySendData(0x05);

    displaySendCommand(0xB2); // porch control
    displaySendData(0x0C);
    displaySendData(0x0C);
    displaySendData(0x00);
    displaySendData(0x33);
    displaySendData(0x33);

    displaySendCommand(0xB7); // gate control
    displaySendData(0x35);

    displaySendCommand(0xBB); // vcom setting
    displaySendData(0x19);

    displaySendCommand(0xC0); // lcm control
    displaySendData(0x2C);

    displaySendCommand(0xC2); // vdv and vrh command enable
    displaySendData(0x01);

    displaySendCommand(0xC3); // vrh set
    displaySendData(0x12);

    displaySendCommand(0xC4); // vdv setting
    displaySendData(0x20);

    displaySendCommand(0xC6); // FR control 2
    displaySendData(0x0F);

    displaySendCommand(0xD0); // power control 1
    displaySendData(0xA4);
    displaySendData(0xA1);

    displaySendCommand(0xE0); // positive voltage gamma control
    displaySendData(0xD0);
    displaySendData(0x04);
    displaySendData(0x0D);
    displaySendData(0x11);
    displaySendData(0x13);
    displaySendData(0x2B);
    displaySendData(0x3F);
    displaySendData(0x54);
    displaySendData(0x4C);
    displaySendData(0x18);
    displaySendData(0x0D);
    displaySendData(0x0B);
    displaySendData(0x1F);
    displaySendData(0x23);

    displaySendCommand(0xE1); // negative voltage gamma control
    displaySendData(0xD0);
    displaySendData(0x04);
    displaySendData(0x0C);
    displaySendData(0x11);
    displaySendData(0x13);
    displaySendData(0x2C);
    displaySendData(0x3F);
    displaySendData(0x44);
    displaySendData(0x51);
    displaySendData(0x2F);
    displaySendData(0x1F);
    displaySendData(0x1F);
    displaySendData(0x20);
    displaySendData(0x23);

    displaySendCommand(0x21); // display inversion on

    displaySendCommand(0x11); // sleep out

    displaySendCommand(0x29); // display on
}

void displayWriteDataWord(uint16_t data)
{
    uint8_t lsb = (data >> 8) & 0xFF;
    displaySelect();
    displayDataMode();
    spiTranferByte(lsb);
    spiTranferByte(data);
    spiWaitForIdle();
    displayDeselect();
}

void displaySetCursor(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd)
{
    displaySendCommand(0x2a);
    displaySendData(xStart + 40);
    displaySendData(xEnd + 40);
    displaySendCommand(0x2b);
    displaySendData(yStart + 53);
    displaySendData(yEnd + 53);
    displaySendCommand(0x2c);
}

#define WHITE (0xFFFF)
void displayClear(uint16_t color)
{
    displaySetCursor(0, 0, (horRes - 1), (vertRes - 1));
    for (uint16_t i = 0; i < horRes; i++)
    {
        for (uint16_t j = 0; j < vertRes; j++)
        {
            displayWriteDataWord(color);
        }
    }
}

void displayTask()
{
    LOG_INFO("Starting displayTask.");

    // lv_color_t dispBuff[DISP_BUF_SIZE];

    // init display
    // spiInit();
    // displayPinsInit();

    // GPIOA->ODR &= ~GPIO_ODR_OD3; // select display
    // displayReset();
    // displayInit();
    // displayClear(WHITE);
    // GPIOA->ODR |= GPIO_ODR_OD3; // deselect display

    // spiInit();
    // displayPinsInit();
    // // send dummy byte to sync clocks
    // spiTranferByte(0x0);
    // spiWaitForIdle();
    // vTaskDelay(1);

    spiInit();
    displayPinsInit();
    displayReset();
    displayInit();
    displayClear(WHITE);
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    // lv_init();
    // lv_display_t *display = lv_display_create(horRes, vertRes);
    // lv_display_set_buffers(display, dispBuff, NULL, DISP_BUF_SIZE, LV_DISP_RENDER_MODE_FULL);
}

void initDisplayTask()
{
    xTaskCreateStatic(displayTask, "Disp", configMINIMAL_STACK_SIZE * 2,
                      NULL, tskIDLE_PRIORITY + 2, displayTaskStack, &displayTaskBuffer);
}