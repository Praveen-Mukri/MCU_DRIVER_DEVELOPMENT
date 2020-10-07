/*
 * STM32F446RE_spi_driver.h
 *
 *  Created on: 08-Oct-2020
 *      Author: praveen mukri
 */

#ifndef INC_STM32F446RE_SPI_DRIVER_H_
#define INC_STM32F446RE_SPI_DRIVER_H_

/*
 * Configuration structure for SPIx peripheral
 */
typedef struct
{
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BusConfig;
    uint8_t SPI_SclkSpeed;
    uint8_t SPI_DFF;
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;
}SPI_Config_t;


/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
    SPI_RegDef_t *pSPIx;            /*< This holds the base address of the SPIx(x:0,1,2) peripheral */
    SPI_Config_t SPIConfig;
}SPI_Handle_t;


/*****************************************************************************************************************
 *                                   APIs supported by this driver
 *****************************************************************************************************************/




#endif /* INC_STM32F446RE_SPI_DRIVER_H_ */
