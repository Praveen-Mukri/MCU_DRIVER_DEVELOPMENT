/*
 * 003Led_Button_Interrupt.c
 *
 *  Created on: 28-Sep-2020
 *      Author: praveen mukri
 */

#include "stm32f446re.h"
#include "STM32F446RE_gpio_driver.h"

int main()
{
    GPIO_Handle_t GpioLed1;
    GPIO_Handle_t GpioButton;

    // To avoid garbage values
    memset(&GpioLed1, 0, sizeof(GpioLed1));
    memset(&GpioButton, 0, sizeof(GpioButton));

    // Push-Pull Configuration
    GpioLed1.pGPIOx = GPIOA;
    GpioLed1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6; // PA6
    GpioLed1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    // User Button is at PC13
    GpioButton.pGPIOx = GPIOC;
    GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13; // PC13 on board User Button
    GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
    GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_PeriClockControl(GPIOC, ENABLE);
    GPIO_Init(&GpioLed1);
    GPIO_Init(&GpioButton);

    // IRQ Configuration
    //GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
    GPIO_IRQConfig(IRQ_NO_EXTI0, ENABLE);
    GPIO_IRQConfig(IRQ_NO_EXTI1, ENABLE);
    GPIO_IRQConfig(IRQ_NO_EXTI2, ENABLE);
    GPIO_IRQConfig(IRQ_NO_EXTI3, ENABLE);
    GPIO_IRQConfig(IRQ_NO_EXTI4, ENABLE);
    GPIO_IRQConfig(IRQ_NO_EXTI9_5, ENABLE);
    GPIO_IRQConfig(IRQ_NO_EXTI15_10, ENABLE);



    while(1);

    return 0;
}


void EXTI15_10_IRQHandler(void)
{
    GPIO_IRQHandling(GPIO_PIN_NO_13);
    GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_6);
}
