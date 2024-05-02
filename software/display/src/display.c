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
static StackType_t displayTaskStack[1000];

const uint32_t horRes = 240;
const uint32_t vertRes = 135;
#define DISP_BUF_SIZE (32400) // 240 * 135
#define PART_BUF_SIZE (3240)  // 240 * 135 / 10
static lv_color_t dispBuff1[PART_BUF_SIZE];
static lv_color_t dispBuff2[PART_BUF_SIZE];

void displaySelect()
{
    GPIOA->ODR &= ~GPIO_ODR_OD3; // select display
}

void displayDeselect()
{
    GPIOA->ODR |= GPIO_ODR_OD3; // deselect display
}

void displayCommandMode()
{
    GPIOA->ODR &= ~GPIO_ODR_OD4; // set DC pin low to signal data
}

void displayDataMode()
{
    GPIOA->ODR |= GPIO_ODR_OD4; // set DC pin high to signal data
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
    GPIOA->MODER |= GPIO_MODER_MODE0_0;       // set mode to "general purpose output", bit 0 is "1"
    GPIOA->MODER &= ~GPIO_MODER_MODE0_1;      // set mode to "general purpose output", bit 0 is "0"
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT0;        // set output type to "push-pull (reset-state)"
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED0_0; // set output speed to high speed, bit 0 is "1"
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED0_1; // set output speed to high speed, bit 1 is "1"

    // PA3 -> LCD_CS
    GPIOA->MODER |= GPIO_MODER_MODE3_0;       // set mode to "general purpose output", bit 0 is "1"
    GPIOA->MODER &= ~GPIO_MODER_MODE3_1;      // set mode to "general purpose output", bit 0 is "0"
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT3;        // set output type to "push-pull (reset-state)"
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED3_0; // set output speed to high speed, bit 0 is "1"
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED3_1; // set output speed to high speed, bit 1 is "1"
    GPIOA->ODR |= GPIO_ODR_OD3;               // deselect display

    // PA4 -> LCD_DC
    GPIOA->MODER |= GPIO_MODER_MODE4_0;       // set mode to "general purpose output", bit 0 is "1"
    GPIOA->MODER &= ~GPIO_MODER_MODE4_1;      // set mode to "general purpose output", bit 0 is "0"
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT4;        // set output type to "push-pull (reset-state)"
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED4_0; // set output speed to high speed, bit 0 is "1"
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED4_1; // set output speed to high speed, bit 1 is "1"
    GPIOA->ODR &= ~GPIO_ODR_OD4;              // enter command mode

    // PA5 -> LCD_BL
    GPIOA->MODER |= GPIO_MODER_MODE5_0;       // set mode to "general purpose output", bit 0 is "1"
    GPIOA->MODER &= ~GPIO_MODER_MODE5_1;      // set mode to "general purpose output", bit 0 is "0"
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;        // set output type to "push-pull (reset-state)"
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED5_0; // set output speed to high speed, bit 0 is "0"
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED5_1; // set output speed to high speed, bit 1 is "0"
    GPIOA->ODR |= GPIO_ODR_OD5;               // enable backlight
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

void clearPattern(uint16_t color)
{
    // displaySendCommand(0x29); // display on
    displayClear(color);
    // vTaskDelay(pdMS_TO_TICKS(50));
    // displaySendCommand(0x28); // display off
}

void lvDisplayFlush(lv_display_t *disp, const lv_area_t *area, uint8_t *color_p)
{
    // displaySetCursor(0, 0, (horRes - 1), (vertRes - 1));
    // uint32_t x, y;
    // for (y = area->y1; y < area->y2; y++)
    // {
    //     for (x = area->x1; x < area->x2; x++)
    //     {
    //         displaySendData(*color_p);
    //     }
    // }

    lv_display_flush_ready(disp);
}

static lv_obj_t *ui_Screen1;
static lv_obj_t *ui_Hello_World;
static lv_obj_t *ui_Button2;
void ui_Screen1_screen_init(void)
{
    ui_Screen1 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE); /// Flags

    ui_Hello_World = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_Hello_World, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_Hello_World, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_Hello_World, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Hello_World, "Hello World");

    ui_Button2 = lv_btn_create(ui_Screen1);
    lv_obj_set_width(ui_Button2, 50);
    lv_obj_set_height(ui_Button2, 25);
    lv_obj_set_x(ui_Button2, 0);
    lv_obj_set_y(ui_Button2, -5);
    lv_obj_set_align(ui_Button2, LV_ALIGN_BOTTOM_MID);
    lv_obj_add_flag(ui_Button2, LV_OBJ_FLAG_SCROLL_ON_FOCUS); /// Flags
    lv_obj_clear_flag(ui_Button2, LV_OBJ_FLAG_SCROLLABLE);    /// Flags
}

void displayTask()
{
    LOG_INFO("Starting displayTask.");

    spiInit();
    displayPinsInit();
    displayInit();

    // lv_init();
    // lv_display_create(horRes, vertRes);
    // lv_display_set_buffers(lv_disp_get_default(), &dispBuff1, dispBuff2, DISP_BUF_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);
    // lv_display_set_flush_cb(lv_disp_get_default(), lvDisplayFlush);
    // lv_tick_set_cb(xTaskGetTickCount);

    // lv_disp_set_theme(display, lv_theme_default_get());
    // ui_Screen1_screen_init();
    // lv_disp_load_scr(ui_Screen1);

    while (true)
    {
        // uint32_t time = lv_timer_handler();
        // vTaskDelay(pdMS_TO_TICKS(time));
        clearPattern(WHITE);
        clearPattern(BLACK);
        clearPattern(BLUE);
        clearPattern(BRED);
        clearPattern(GRED);
        clearPattern(GBLUE);
        clearPattern(RED);
        clearPattern(MAGENTA);
        clearPattern(GREEN);
        clearPattern(CYAN);
        clearPattern(YELLOW);
        clearPattern(BROWN);
        clearPattern(BRRED);
        clearPattern(GRAY);
        clearPattern(DARKBLUE);
        clearPattern(LIGHTBLUE);
        clearPattern(GRAYBLUE);
        clearPattern(LIGHTGREEN);
        clearPattern(LGRAY);
        clearPattern(LGRAYBLUE);
        clearPattern(LBBLUE);
    }
}

void initDisplayTask()
{
    xTaskCreateStatic(displayTask, "Disp", 1000,
                      NULL, tskIDLE_PRIORITY + 1, displayTaskStack, &displayTaskBuffer);
}