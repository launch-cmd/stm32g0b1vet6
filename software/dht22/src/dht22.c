#include <FreeRTOS.h>
#include <stdbool.h>
#include "task.h"
#include "board_config.h"
#include "log.h"
#include "dht22.h"
#include "ui.h"
#include "lvgl.h"

static StaticTask_t dht22TaskBuffer;
static StackType_t dht22TaskStack[200];
static uint16_t temp, rhum = 0;
static uint8_t csum = 0;
static bool isNegativeTemp = false;
char tempLabelBuffer[32];
char rhumLabelBuffer[32];

static void dht22PinsInit()
{
    RCC->IOPENR |= RCC_IOPENR_GPIOCEN; // enable GPIOC clock

    // PC0 -> DHT_GND
    GPIOC->MODER |= GPIO_MODER_MODE0_0;        // set mode to "general purpose output", bit 0 is "1"
    GPIOC->MODER &= ~GPIO_MODER_MODE0_1;       // set mode to "general purpose output", bit 0 is "0"
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT0;         // set output type to "push-pull (reset-state)"
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD0_0;       // enable pull-down(0b10), bit 0 is "0"
    GPIOC->PUPDR |= GPIO_PUPDR_PUPD0_1;        // enable pull-down(0b10), bit 1 is "1"
    GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED0_0; // set output speed to slow speed, bit 0 is "0"
    GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED0_1; // set output speed to slow speed, bit 1 is "0"
    GPIOC->ODR &= ~GPIO_ODR_OD0;               // use pin as ground

    // PC1 -> DHT_DATA
    GPIOC->MODER |= GPIO_MODER_MODE1_0;       // set mode to "general purpose output", bit 0 is "1"
    GPIOC->MODER &= ~GPIO_MODER_MODE1_1;      // set mode to "general purpose output", bit 0 is "0"
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT1;        // set output type to "push-pull (reset-state)"
    GPIOC->PUPDR |= GPIO_PUPDR_PUPD1_0;       // enable pull-up(0b01), bit 0 is "1"
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD1_1;      // enable pull-up(0b01), bit 1 is "0"
    GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED1_0; // set output speed to high speed, bit 0 is "1"
    GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED1_1; // set output speed to high speed, bit 1 is "1"
    GPIOC->ODR |= GPIO_ODR_OD1;

    // PC2 -> DHT_VCC
    GPIOC->MODER |= GPIO_MODER_MODE2_0;        // set mode to "general purpose output", bit 0 is "1"
    GPIOC->MODER &= ~GPIO_MODER_MODE2_1;       // set mode to "general purpose output", bit 0 is "0"
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT2;         // set output type to "push-pull (reset-state)"
    GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED2_0; // set output speed to slow speed, bit 0 is "0"
    GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED2_1; // set output speed to slow speed, bit 1 is "0"
    GPIOC->ODR |= GPIO_ODR_OD2;                // enable VCC to dht22

    vTaskDelay(pdMS_TO_TICKS(2000)); // wait 2 seconds for DHT22 to stabalize
}

static void getDataBit(uint16_t *data, uint8_t shift)
{
    /*
        0 bit is a high period of ~28ms
        1 bit is a high period of ~50ms
        Wait approximately 40 us.
        when pin is high after waiting, bit is 1.
        At 64 MHz waiting 1 us would be 6.4 cycles.
        Waiting for 40us takes 256 cycles.
    */
    for (int i = 0; i < 256; i++)
    {
        __ASM("NOP");
    }

    if (GPIOC->IDR & GPIO_IDR_ID1)
    {
        // data bit is '1'
        while (GPIOC->IDR & GPIO_IDR_ID1)
        {
            // check if pin is still high, if it is we wait for it to go low
        }
        *data |= (1 << shift); // set i'th bit of data
    }
    else
    {
        // data bit is '0'
        *data &= ~(1 << shift); // clear i'th bit of data
    }
}

static void getChecksumBit(uint8_t *data, uint8_t shift)
{
    /*
        0 bit is a high period of ~28ms
        1 bit is a high period of ~50ms
        Wait approximately 40 us.
        when pin is high after waiting, bit is 1.
        At 64 MHz waiting 1 us would be 6.4 cycles.
        Waiting for 40us takes 256 cycles.
    */
    for (int i = 0; i < 256; i++)
    {
        __ASM("NOP");
    }

    if (GPIOC->IDR & GPIO_IDR_ID1)
    {
        // data bit is '1'
        while (GPIOC->IDR & GPIO_IDR_ID1)
        {
            // check if pin is still high, if it is we wait for it to go low
        }
        *data |= (1 << shift); // set i'th bit of data
    }
    else
    {
        // data bit is '0'
        *data &= ~(1 << shift); // clear i'th bit of data
    }
}

static void dht22Measure()
{
    __disable_irq();
    // set data pin as output
    GPIOC->MODER |= GPIO_MODER_MODE1_0;  // set mode to "general purpose output", bit 0 is "1"
    GPIOC->MODER &= ~GPIO_MODER_MODE1_1; // set mode to "general purpose output", bit 0 is "0"
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT1;   // set output type to "push-pull (reset-state)"
    GPIOC->PUPDR |= GPIO_PUPDR_PUPD1_0;  // enable pull-up(0b01), bit 0 is "1"
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD1_1; // enable pull-up(0b01), bit 1 is "0"

    // pull low for at least 1ms
    GPIOC->ODR &= ~GPIO_ODR_OD1;
    /*
        At 64 MHz waiting 1 us would be 6.4 cycles.
        Waiting for 1ms takes 6400 cycles.
    */
    for (int i = 0; i < 6400; i++)
    {
        __ASM("NOP");
    }

    // pull high for ~50 ms
    GPIOC->ODR |= GPIO_ODR_OD1;
    for (int i = 0; i < 320; i++)
    {
        // busy wait for ~50 us
    }

    // set data pin as input
    GPIOC->MODER &= ~GPIO_MODER_MODE1_0; // set mode to input, bit 0 is "0"
    GPIOC->MODER &= ~GPIO_MODER_MODE1_1; // set mode to input, bit 1 is "0"
    GPIOC->OTYPER |= GPIO_OTYPER_OT1;    // set output type to "open drain"
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD1_0; // no pull-up / pull down, bit 0 is "0"
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD1_1; // no pull-up / pull down, bit 1 is "0"

    while (!(GPIOC->IDR & GPIO_IDR_ID1))
    {
        // wait for pull up by DHT22
    }
    while (GPIOC->IDR & GPIO_IDR_ID1)
    {
        // wait for drop by DHT22 (signalling start of data)
    }

    // decode 16 Relative Humidity bits
    for (int i = 16; i-- > 0;)
    {
        // look for start bit signal (down period of ~53us)
        while (!(GPIOC->IDR & GPIO_IDR_ID1))
        {
            // wait for startbit to end
        }
        getDataBit(&rhum, i);
    }

    // decode 16 Temperature bits
    for (int i = 16; i-- > 0;)
    {
        // look for start bit signal (down period of ~53us)
        while (!(GPIOC->IDR & GPIO_IDR_ID1))
        {
            // wait for startbit to end
        }
        getDataBit(&temp, i);
    }

    // check 16'th bit for sign
    isNegativeTemp = (temp & (1 << 15));

    // decode 8 Checksum bits
    for (int i = 8; i-- > 0;)
    {
        // look for start bit signal (down period of ~53us)
        while (!(GPIOC->IDR & GPIO_IDR_ID1))
        {
            // wait for startbit to end
        }
        getChecksumBit(&csum, i);
    }
    __enable_irq();
}

static void dht22Task()
{
    LOG_INFO("Starting dht22Task.");

    dht22PinsInit();

    while (true)
    {
        dht22Measure();
        // create label, show temperature with degrees C symbol and 1 decimal.
        // show humidity with percent symbol and 1 decimal.
        snprintf(tempLabelBuffer, sizeof(tempLabelBuffer), "%d.%d C", (temp / 10), (temp % 10));
        lv_label_set_text_static(ui_tempLabel, tempLabelBuffer);
        snprintf(rhumLabelBuffer, sizeof(rhumLabelBuffer), "%d.%d %%", (rhum / 10), (rhum % 10));
        lv_label_set_text_static(ui_rhumLabel, rhumLabelBuffer);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void initDht22Task()
{
    xTaskCreateStatic(dht22Task, "DHT22", 200,
                      NULL, tskIDLE_PRIORITY + 3, dht22TaskStack, &dht22TaskBuffer);
}