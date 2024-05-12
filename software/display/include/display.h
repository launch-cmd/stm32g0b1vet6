#ifndef DISPLAY_H__
#define DISPLAY_H__
#include <stdint.h>

void initDisplayTask();

void updateTempChartScreen(uint16_t newVal, uint16_t *minValue, uint16_t *maxValue);
void updateRhumChartScreen(uint16_t newVal);
void updateSummaryScreen(uint16_t temp, uint16_t rhum);
#endif