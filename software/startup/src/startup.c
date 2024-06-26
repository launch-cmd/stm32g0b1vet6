#include <FreeRTOS.h>
#include "task.h"
#include "board_config.h"

// Initialize the clocks to their default values.
// This function should be called before main() is entered.
static void clocksInit()
{
  FLASH->ACR &= ~FLASH_ACR_LATENCY_0; // set 2 wait states for flash(0b010), bit 0 is "0"
  FLASH->ACR |= FLASH_ACR_LATENCY_1;  // set 2 wait states for flash(0b010), bit 1 is "1"
  FLASH->ACR &= ~FLASH_ACR_LATENCY_2; // set 2 wait states for flash(0b010), bit 2 is "0"
  while ((FLASH->ACR & FLASH_ACR_LATENCY) != 0x2)
  {
    // wait for flash latency to be set
  }

  // RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN; // enable SYSCFG, COMP and VREFBUF clock
  // RCC->APBENR1 |= RCC_APBENR1_PWREN;    // enable power interface clock
  // RCC->APBENR1 |= RCC_APBENR1_DBGEN;    // enable debug support clock

  // RCC->CFGR |= RCC_CFGR_PPRE_0; // set prescaler to 16(0b111), bit "0" is "1"
  // RCC->CFGR |= RCC_CFGR_PPRE_1; // set prescaler to 16(0b111), bit "1" is "1"
  // RCC->CFGR |= RCC_CFGR_PPRE_2; // set prescaler to 16(0b111), bit "2" is "1"

  RCC->CR |= RCC_CR_HSION; // enable HSI
  while (!(RCC->CR & RCC_CR_HSION))
  {
    // wait for HSI to be ready
  }

  // setup PLL
  RCC->CR &= ~RCC_CR_PLLON; // disable PLL
  while ((RCC->CR & RCC_CR_PLLRDY))
  {
    // wait for PLL to be off
  }
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC_0; // set input clock source to HSI16(0b10), bit 0 is "0"
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_1;  // set input clock source to HSI16(0b10), bit 1 is "1"
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_0;   // set input division factor to 1 (0b000), bit 0 is "0"
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_1;   // set input division factor to 1 (0b000), bit 1 is "0"
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_2;   // set input division factor to 1 (0b000), bit 2 is "0"
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN_0;   // set multiplication factor to 8 (0b0001000), bit 0 is "0"
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN_1;   // set multiplication factor to 8 (0b0001000), bit 1 is "0"
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN_2;   // set multiplication factor to 8 (0b0001000), bit 2 is "0"
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_3;    // set multiplication factor to 8 (0b0001000), bit 3 is "1"
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN_4;   // set multiplication factor to 8 (0b0001000), bit 4 is "0"
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN_5;   // set multiplication factor to 8 (0b0001000), bit 5 is "0"
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN_6;   // set multiplication factor to 8 (0b0001000), bit 6 is "0"
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLR_0;    // set VCO division factor to 2 (0b001), bit 0 is "1"
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLR_1;   // set VCO division factor to 2 (0b001), bit 1 is "0"
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLR_2;   // set VCO division factor to 2 (0b001), bit 2 is "0"

  RCC->PLLCFGR |= RCC_PLLCFGR_PLLQ_0;    // set VCO division factor to 2 (0b001), bit 0 is "1"
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQ_1;   // set VCO division factor to 2 (0b001), bit 1 is "0"
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQ_2;   // set VCO division factor to 2 (0b001), bit 2 is "0"

  RCC->PLLCFGR |= RCC_PLLCFGR_PLLP_0;    // set VCO division factor to 2 (0b00001), bit 0 is "1"
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP_1;   // set VCO division factor to 2 (0b00001), bit 1 is "0"
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP_2;   // set VCO division factor to 2 (0b00001), bit 2 is "0"
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP_3;   // set VCO division factor to 2 (0b00001), bit 2 is "0"
  RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP_4;   // set VCO division factor to 2 (0b00001), bit 2 is "0"


  RCC->CR |= RCC_CR_PLLON;               // enable PLL
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;    // enable clock output to R domain of PLL
  while (!(RCC->CR & RCC_CR_PLLRDY))
  {
    // wait for PLL to be ready
  }

  RCC->CFGR &= ~RCC_CFGR_HPRE_0; // set AHB prescaler to 1(0b0xxx), bit 0 is "x"
  RCC->CFGR &= ~RCC_CFGR_HPRE_1; // set AHB prescaler to 1(0b0xxx), bit 1 is "x"
  RCC->CFGR &= ~RCC_CFGR_HPRE_2; // set AHB prescaler to 1(0b0xxx), bit 2 is "x"
  RCC->CFGR &= ~RCC_CFGR_HPRE_3; // set AHB prescaler to 1(0b0xxx), bit 3 is "0"

  RCC->CFGR &= ~RCC_CFGR_PPRE_0; // set APB prescaler to 1(0b0xx), bit 0 is "x"
  RCC->CFGR &= ~RCC_CFGR_PPRE_1; // set APB prescaler to 1(0b0xx), bit 1 is "x"
  RCC->CFGR &= ~RCC_CFGR_PPRE_2; // set APB prescaler to 1(0b0xx), bit 2 is "0"

  RCC->CFGR &= ~RCC_CFGR_SW_0; // set PLL as system clock source(0b010), bit 0 is "0"
  RCC->CFGR |= RCC_CFGR_SW_1;  // set PLL as system clock source(0b010), bit 1 is "1"
  RCC->CFGR &= ~RCC_CFGR_SW_2; // set PLL as system clock source(0b010), bit 2 is "0"

  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLLRCLK)
  {
    // wait for PLL to become the active system clock source
  }
}

// reset hook
__attribute__((naked, noreturn)) void Reset_Handler(void)
{
  // memset .bss to zero, and copy .data section to RAM region
  extern long _sbss, _ebss, _sdata, _edata, _sidata;
  for (long *dst = &_sbss; dst < &_ebss; dst++)
  {
    *dst = 0;
  }
  for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;)
  {
    *dst++ = *src++;
  }
  clocksInit();
  extern void main(void);
  main(); // Call main()
  for (;;)
    (void)0; // Infinite loop in the case if main() returns
}

// easy way to force a bus fault (hard fault)
// src: https://stackoverflow.com/a/62815936
__attribute__((unused)) static void forceHardFault()
{
  typedef void (*fn_t)();
  fn_t foo = (fn_t)(0x8004000);
  foo();
}

#define WEAK_ALIAS __attribute__((weak, alias("DefaultIRQHandler")))

WEAK_ALIAS void NMI_Handler(void);
void HardFault_Handler(void) __attribute__((weak));
WEAK_ALIAS void SVC_Handler(void);
WEAK_ALIAS void PendSV_Handler(void);
WEAK_ALIAS void SysTick_Handler(void);
WEAK_ALIAS void WWDG_Handler(void);
WEAK_ALIAS void PVD_Handler(void);
WEAK_ALIAS void RTC_TAMP_Handler(void);
WEAK_ALIAS void FLASH_Handler(void);
WEAK_ALIAS void RCC_CRS_Handler(void);
WEAK_ALIAS void EXTI0_1_Handler(void);
WEAK_ALIAS void EXTI2_3_Handler(void);
WEAK_ALIAS void EXTI4_15_Handler(void);
WEAK_ALIAS void UCPD1_UCPD2_USB_Handler(void);
WEAK_ALIAS void DMA1_Channel1_Handler(void);
WEAK_ALIAS void DMA1_Channel2_3_Handler(void);
WEAK_ALIAS void DMA1_DMA2_Other_Handler(void);
WEAK_ALIAS void ADC_COMP_Handler(void);
WEAK_ALIAS void TIM1_BRK_UP_TRG_COM_Handler(void);
WEAK_ALIAS void TIM1_CC_Handler(void);
WEAK_ALIAS void TIM2_Handler(void);
WEAK_ALIAS void TIM3_TIM4_Handler(void);
WEAK_ALIAS void TIM6_DAC_LPTIM1_Handler(void);
WEAK_ALIAS void TIM7_LPTIM2_Handler(void);
WEAK_ALIAS void TIM14_Handler(void);
WEAK_ALIAS void TIM15_Handler(void);
WEAK_ALIAS void TIM16_FDCAN_IT0_Handler(void);
WEAK_ALIAS void TIM17_FDCAN_IT1_Handler(void);
WEAK_ALIAS void I2C1_Handler(void);
WEAK_ALIAS void I2C2_I2C3_Handler(void);
WEAK_ALIAS void SPI1_Handler(void);
WEAK_ALIAS void SPI2_SPI3_Handler(void);
WEAK_ALIAS void USART1_Handler(void);
WEAK_ALIAS void USART2_LPUART2_Handler(void);
WEAK_ALIAS void LPUART1_USART3_4_5_6_Handler(void);
WEAK_ALIAS void CEC_Handler(void);
WEAK_ALIAS void AES_RNG_Handler(void);

extern void _estack(void); // Defined in advanced.ld

// 16 standard and 32 STM32-specific handlers
__attribute__((section(".isr_vector"))) void (*const tab[16 + 32])(void) =
    {
        _estack, Reset_Handler, NMI_Handler, HardFault_Handler, 0, 0, 0, 0, 0, 0, 0,
        SVC_Handler, 0, 0, PendSV_Handler, SysTick_Handler,

        WWDG_Handler, PVD_Handler, RTC_TAMP_Handler, FLASH_Handler, RCC_CRS_Handler,
        EXTI0_1_Handler, EXTI2_3_Handler, EXTI4_15_Handler, UCPD1_UCPD2_USB_Handler,
        DMA1_Channel1_Handler, DMA1_Channel2_3_Handler, DMA1_DMA2_Other_Handler,
        ADC_COMP_Handler, TIM1_BRK_UP_TRG_COM_Handler, TIM1_CC_Handler, TIM2_Handler,
        TIM3_TIM4_Handler, TIM6_DAC_LPTIM1_Handler, TIM7_LPTIM2_Handler, TIM14_Handler,
        TIM15_Handler, TIM16_FDCAN_IT0_Handler, TIM17_FDCAN_IT1_Handler, I2C1_Handler,
        I2C2_I2C3_Handler, SPI1_Handler, SPI2_SPI3_Handler, USART1_Handler,
        USART2_LPUART2_Handler, LPUART1_USART3_4_5_6_Handler, CEC_Handler, AES_RNG_Handler};

void __attribute__((weak)) DefaultIRQHandler(void)
{
  for (;;)
    (void)0;
}