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
    GPIO_Handle_t GpioLed1;
    GPIO_Handle_t GpioLed2;

    // Push-Pull Configuration
    GpioLed1.pGPIOx = GPIOA;
    GpioLed1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GpioLed1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    // Open-Drain Configuration ( intensity will be very low)
    GpioLed2.pGPIOx = GPIOA;
    GpioLed2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5; // PA5 on board Green LED
    GpioLed2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    GpioLed2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&GpioLed1);
    GPIO_Init(&GpioLed2);

    while(1)
    {
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_6);
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
        delay();
    }
    return 0;
}
