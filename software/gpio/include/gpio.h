#ifndef GPIO_H__
#define GPIO_H__

#include <stdint.h>
#include <stdbool.h>

// used to define a pin using the name of the bank and a pin number between 0 and 15
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))

enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };

// set mode for a pin
void gpioSetMode(uint16_t pin, uint8_t mode);

// don't know
void gpioSetAf(uint16_t pin, uint8_t af_num);

// set pin to high or low state by changing val from true to false
void gpioWrite(uint16_t pin, bool val);
#endif