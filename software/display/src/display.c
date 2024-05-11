#include <FreeRTOS.h>
#include "task.h"
#include "log.h"
#include "lv_conf.h"
#define LV_CONF_INCLUDE_SIMPLE
#include "lvgl.h"
#include "spi.h"
#include "board_config.h"
#include "display.h"
#include "ui.h"
#include "timers.h"

static StaticTask_t displayTaskBuffer;
static StackType_t displayTaskStack[700];

const uint32_t horRes = 240;
const uint32_t vertRes = 135;
#define DISP_BUF_SIZE (3240) // 240 * 135 / 10
static lv_color_t dispBuff[DISP_BUF_SIZE];
static lv_disp_draw_buf_t drawBuff;
static lv_disp_drv_t dispDriver;

TimerHandle_t timer;
StaticTimer_t timerBuff;

void vTimerCallback(TimerHandle_t xTimer)
{
    lv_tick_inc(1);
}

void displayPinsInit()
{
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN; // enable GPIOB clock

    // PB6 -> LCD_BL
    GPIOB->MODER |= GPIO_MODER_MODE6_0;       // set mode to "general purpose output", bit 0 is "1"
    GPIOB->MODER &= ~GPIO_MODER_MODE6_1;      // set mode to "general purpose output", bit 0 is "0"
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT6;        // set output type to "push-pull (reset-state)"
    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED6_0; // set output speed to high speed, bit 0 is "0"
    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED6_1; // set output speed to high speed, bit 1 is "0"
    GPIOB->ODR |= GPIO_ODR_OD6;               // enable backlight

    // PB7 -> LCD_RST
    GPIOB->MODER |= GPIO_MODER_MODE7_0;       // set mode to "general purpose output", bit 0 is "1"
    GPIOB->MODER &= ~GPIO_MODER_MODE7_1;      // set mode to "general purpose output", bit 0 is "0"
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT7;        // set output type to "push-pull (reset-state)"
    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED7_0; // set output speed to high speed, bit 0 is "1"
    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED7_1; // set output speed to high speed, bit 1 is "1"

    // PB8 -> LCD_DC
    GPIOB->MODER |= GPIO_MODER_MODE8_0;       // set mode to "general purpose output", bit 0 is "1"
    GPIOB->MODER &= ~GPIO_MODER_MODE8_1;      // set mode to "general purpose output", bit 0 is "0"
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT8;        // set output type to "push-pull (reset-state)"
    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED8_0; // set output speed to high speed, bit 0 is "1"
    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED8_1; // set output speed to high speed, bit 1 is "1"
    GPIOB->ODR &= ~GPIO_ODR_OD8;              // enter command mode

    // PB9 -> LCD_CS
    GPIOB->MODER |= GPIO_MODER_MODE9_0;       // set mode to "general purpose output", bit 0 is "1"
    GPIOB->MODER &= ~GPIO_MODER_MODE9_1;      // set mode to "general purpose output", bit 0 is "0"
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT9;        // set output type to "push-pull (reset-state)"
    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED9_0; // set output speed to high speed, bit 0 is "1"
    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED9_1; // set output speed to high speed, bit 1 is "1"
    GPIOB->ODR |= GPIO_ODR_OD9;               // deselect display

    // PB12 -> LCD_GND
    GPIOB->MODER |= GPIO_MODER_MODE12_0;        // set mode to "general purpose output", bit 0 is "1"
    GPIOB->MODER &= ~GPIO_MODER_MODE12_1;       // set mode to "general purpose output", bit 0 is "0"
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT12;         // set output type to "push-pull (reset-state)"
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD12_0;       // enable pull-down(0b10), bit 0 is "0"
    GPIOB->PUPDR |= GPIO_PUPDR_PUPD12_1;        // enable pull-down(0b10), bit 1 is "1"
    GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED12_0; // set output speed to slow speed, bit 0 is "0"
    GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED12_1; // set output speed to slow speed, bit 1 is "0"
    GPIOB->ODR &= ~GPIO_ODR_OD12;               // use pin as ground

    // PB13 -> LCD_VCC
    GPIOB->MODER |= GPIO_MODER_MODE13_0;        // set mode to "general purpose output", bit 0 is "1"
    GPIOB->MODER &= ~GPIO_MODER_MODE13_1;       // set mode to "general purpose output", bit 0 is "0"
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT13;         // set output type to "push-pull (reset-state)"
    GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED13_0; // set output speed to slow speed, bit 0 is "0"
    GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED13_1; // set output speed to slow speed, bit 1 is "0"
    GPIOB->ODR |= GPIO_ODR_OD13;                // enable VCC to display
}

void displaySelect()
{
    GPIOB->ODR &= ~GPIO_ODR_OD9; // select display
}

void displayDeselect()
{
    GPIOB->ODR |= GPIO_ODR_OD9; // deselect display
}

void displayCommandMode()
{
    GPIOB->ODR &= ~GPIO_ODR_OD8; // set DC pin low to signal data
}

void displayDataMode()
{
    GPIOB->ODR |= GPIO_ODR_OD8; // set DC pin high to signal data
}

void displayReset()
{
    // send reset sequence
    GPIOB->ODR &= ~GPIO_ODR_OD7;
    vTaskDelay(pdMS_TO_TICKS(50));
    GPIOB->ODR |= GPIO_ODR_OD7;
    vTaskDelay(pdMS_TO_TICKS(50));
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

void displayInit()
{
    displayReset();

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
    displayWriteDataWord(xStart + 40);
    displayWriteDataWord(xEnd + 40);
    displaySendCommand(0x2b);
    displayWriteDataWord(yStart + 53);
    displayWriteDataWord(yEnd + 53);
    displaySendCommand(0x2c);
}

#define WHITE 0xFFFF
#define BLACK 0x0000
#define BLUE 0x001F
#define BRED 0XF81F
#define GRED 0XFFE0
#define GBLUE 0X07FF
#define RED 0xF800
#define MAGENTA 0xF81F
#define GREEN 0x07E0
#define CYAN 0x7FFF
#define YELLOW 0xFFE0
#define BROWN 0XBC40
#define BRRED 0XFC07
#define GRAY 0X8430
#define DARKBLUE 0X01CF
#define LIGHTBLUE 0X7D7C
#define GRAYBLUE 0X5458
#define LIGHTGREEN 0X841F
#define LGRAY 0XC618
#define LGRAYBLUE 0XA651
#define LBBLUE 0X2B12
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

void displayClearLvgl(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    int32_t x, y;
    displaySetCursor(area->x1, area->y1, area->x2, area->y2);
    for (y = area->y1; y <= area->y2; y++)
    {
        for (x = area->x1; x <= area->x2; x++)
        {
            displayWriteDataWord(color_p->full);
            color_p++;
        }
    }
    lv_disp_flush_ready(disp);
}

void displayTask()
{
    LOG_INFO("Starting displayTask.");

    spiInit();
    displayPinsInit();
    displayInit();
    lv_init();
    lv_disp_draw_buf_init(&drawBuff, &dispBuff, NULL, DISP_BUF_SIZE);

    lv_disp_drv_init(&dispDriver);
    dispDriver.flush_cb = displayClearLvgl;
    dispDriver.draw_buf = &drawBuff;
    dispDriver.hor_res = horRes;
    dispDriver.ver_res = vertRes;
    lv_disp_drv_register(&dispDriver);

    timer = xTimerCreateStatic("lvglTimer", pdMS_TO_TICKS(1), pdTRUE, 0, vTimerCallback, &timerBuff);
    configASSERT(timer);
    xTimerStart(timer, 0);

    ui_init();

    while (true)
    {
        lv_timer_handler_run_in_period(5);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void initDisplayTask()
{
    xTaskCreateStatic(displayTask, "Disp", 700,
                      NULL, tskIDLE_PRIORITY + 2, displayTaskStack, &displayTaskBuffer);
}