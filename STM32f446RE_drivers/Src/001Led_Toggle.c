/*
 * 001Led_Toggle.c
 *
 *  Created on: 26-Sep-2020
 *      Author: praveen mukri
 */


#include "stm32f446re.h"
#include "STM32F446RE_gpio_driver.h"

void delay(void)
{
    for(uint32_t i = 0; i < 50000 * 2; i++);
}

int main(void)
{
    GPIO_Handle_t GpioLed;

    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&GpioLed);

    while(1)
    {
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_6);
        delay();
    }
    return 0;
}
