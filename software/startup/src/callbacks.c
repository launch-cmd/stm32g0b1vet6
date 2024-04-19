#include <FreeRTOS.h>
#include "task.h"

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *puxTimerTaskStackSize)
{
  static StaticTask_t xTimerTaskTCB;
  static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;
  *puxTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *puxIdleTaskStackSize)
{
  static StaticTask_t xIdleTaskTCB;
  static StackType_t uxIdleTaskStack[configTIMER_TASK_STACK_DEPTH];
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;
  *puxIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

// overflow hook
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  while (1)
  {
    vTaskDelay(1);
  }
}


// hardfault hook
void HardFault_Handler(void)
{
  while (1)
  {
    vTaskDelay(1);
  }
}