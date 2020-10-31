/*
 * 005Spi_TxOnly_Arduino.c
 *
 *  Created on: 10-Oct-2020
 *      Author: praveen mukri
 */


#include "STM32F446RE_gpio_driver.h"
#include "STM32F446RE_spi_driver.h"
#include "string.h"
/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 *  ALT function mode : 5
 */
void SPI2_GPIOInits(void)
{
    GPIO_Handle_t SPIPins;

    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunction = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    //SCLK
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPIPins);

    //MOSI
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPIPins);

/*    //MISO
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_Init(&SPIPins);*/

    //NSS
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPIPins);
}

void SPI2_Init()
{

    SPI_Handle_t SPI2Handle;

    SPI2Handle.pSPIx = SPI2;
    SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
    SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // Hardware slave management enabled for NSS

    SPI_Init(&SPI2Handle);
}

void GPIO_ButtonInit(void)
{
    GPIO_Handle_t GpioButton;

    // User Button is at PC13
    GpioButton.pGPIOx = GPIOC;
    GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13; // PC13 on board User Button
    GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GpioButton);
}

void delay(void)
{
    for(uint32_t i = 0; i < 50000; i++);
}

int main(void)
{
    char user_Data[] = "Hello world";

    GPIO_ButtonInit();

    // initialize the GPIO pins to behave as a SPI pins
    SPI2_GPIOInits();

    // Initialize SPI configuration
    SPI2_Init();

    /*
     * making SSOE 1 does NSS output enable
     * The NSS pin is automatically managed by hardware
     * i.e when SPE=1, NSS will be pulled to low
     * and NSS pin will be high when SPE=0
     */
    SPI_SSOEConfig(SPI2, ENABLE);


    while(1)
    {
        while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

        delay(); // to avoid debouncing

        // Enable SPI peripheral
        SPI_PeripheralControl(SPI2,ENABLE);

        // firts send length information
        uint8_t datalen = strlen(user_Data);
        SPI_SendData(SPI2, &datalen, 1);

        // Send data
        SPI_SendData(SPI2, (uint8_t*) user_Data, strlen(user_Data));


        //lets confirm SPI is not busy
        while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

        //Disable the SPI2 peripheral
        SPI_PeripheralControl(SPI2,DISABLE);

    }


    return 0;
}


