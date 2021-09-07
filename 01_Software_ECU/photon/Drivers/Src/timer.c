/*
 * timer.c
 *
 *  Created on: 20.08.2021
 *      Author: Maximilian Altrichter
 */

#include "timer.h"

/*******************************************************************
 * @function			- SystemCoreClockUpdate
 * @brief				- Update SystemCoreClock variable according to Clock Register Values.
  *                       The SystemCoreClock variable contains the core clock (HCLK), it can
  *                       be used by the user application to setup the SysTick timer or configure
  *                       other parameters.
 * @Note				- none
 */

//copied 
uint32_t SystemCoreClockUpdate(void)
{
    uint32_t tmp = 0;
    uint32_t SystemCoreClock = 16000000;
    tmp = RCC->CFGR & (0x3UL << 2U);

    switch(tmp)
    {
        case 0x00:
            SystemCoreClock=HSI_VALUE;
            break;
        case 0x04:
            SystemCoreClock = HSE_VALUE;
            break;
        case 0x08:
            //TODO: dont know whats going on here dude
            break;
        default:
            SystemCoreClock = HSI_VALUE;
    }
    return SystemCoreClock;
}

void __NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->IP[((uint32_t)IRQn)] = (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
  }
  else
  {
    SCB->SHP[(((uint32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
  }
}

void SysTick_Config(uint32_t ticks)
{
    if ((ticks - 1UL) > 0xFFFFFFUL)
    {
        return (1UL);                                                   /* Reload value impossible */
    }

    SysTick->LOAD  = (uint32_t)(ticks - 1UL);   
    __NVIC_SetPriority (-1, (1UL << 4U) - 1UL);
    SysTick->VAL   = 0UL;  
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                   SysTick_CTRL_TICKINT_Msk   |
                   SysTick_CTRL_ENABLE_Msk;  
    return (0UL);
}







