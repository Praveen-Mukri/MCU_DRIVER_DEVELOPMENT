/*
 * 002Led_Button.c
 *
 *  Created on: 26-Sep-2020
 *      Author: praveen mukri
 */

#include "stm32f446re.h"
#include "STM32F446RE_gpio_driver.h"

#define LOW     0
#define BUTTON_PRESSED LOW

void delay(void)
{
    for(uint32_t i = 0; i < 50000 * 2; i++);
}

int main(void)
{
    GPIO_Handle_t GpioLed1;
    GPIO_Handle_t GpioButton;

    // Push-Pull Configuration
    GpioLed1.pGPIOx = GPIOA;
    GpioLed1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GpioLed1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    // User Button is at PC13
    GpioButton.pGPIOx = GPIOC;
    GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13; // PC13 on board User Button
    GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_PeriClockControl(GPIOC, ENABLE);
    GPIO_Init(&GpioLed1);
    GPIO_Init(&GpioButton);

    while(1)
    {
        if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BUTTON_PRESSED)
        {
            // Delay is to avoid debouncing condition
            delay();
            GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_6);
        }


    }
    return 0;
}

