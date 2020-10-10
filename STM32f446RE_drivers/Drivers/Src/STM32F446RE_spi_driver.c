/*
 * STM32F446RE_spi_driver.c
 *
 *  Created on: 08-Oct-2020
 *      Author: praveen mukri
 */

#include "STM32F446RE_spi_driver.h"


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
    if(pSPIx->SR & FlagName)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t ENorDI)
{
    if(ENorDI == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}


/*********************************************************************
 * @function                - SPI_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 ***********************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
    if(ENorDI == ENABLE)
    {
        if(pSPIx == SPI1)
        {
            SPI1_PLCK_EN();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PLCK_EN();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PLCK_EN();
        }
        // TODO: check SPI4 base address
        /*
        else if (pSPIx == SPI4)
        {
            SPI4_PLCK_EN();
        }
        */
    }
    else
    {
        if(pSPIx == SPI1)
        {
            SPI1_PLCK_DI();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PLCK_DI();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PLCK_DI();
        }
        // TODO: check SPI4 base address
        /*
        else if (pSPIx == SPI4)
        {
            SPI4_PLCK_DI();
        }
        */

    }
}


/*********************************************************************
 * @function                - SPI_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 ***********************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

    // enable peripheral clock
    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    // first configure the SPI_CR1 register

    uint32_t tempReg = 0;

    // configure device mode
    tempReg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

    // configure bus config
    if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
    {
        // BIDI mode should be cleared
        tempReg &= ~(1 << SPI_CR1_BIDIMODE);
    }else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
        // BIDI should be set
        tempReg |= (1 << SPI_CR1_BIDIMODE);
    }else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLE_RXONLY)
    {
        // BIDI should be cleared
        tempReg &= ~(1 << SPI_CR1_BIDIMODE);
        //RXONLY bit must be set
        tempReg |= (1 << SPI_CR1_RXONLY);
    }

    // 3. configure the spi serial clock speed ( baud rate)
    tempReg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

    // 4. configure DFF
    tempReg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

    // 5. configure CPOL
    tempReg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

    // 6. configure CPHA
    tempReg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

    pSPIHandle->pSPIx->CR1 = tempReg;
}


/*********************************************************************
 * @function                - SPI_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  This is a blocking call, polling based data send
 ***********************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
    while( Len > 0)
    {
        // 1. wait until TXE is set
        while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG));

        //2. check the DFF bit in CR1
        if( (pSPIx->CR1 & (1 << SPI_CR1_DFF) ) )
        {
            // 16bit DFF
            //1. load the data into the DR
            pSPIx->DR = *((uint16_t*)pTxBuffer);
            Len--;
            Len--;
            (uint16_t*)pTxBuffer++;
        }
        else
        {
            //8bit DFF
            pSPIx->DR = *pTxBuffer;
            Len--;
            pTxBuffer++;
        }
    }
}
