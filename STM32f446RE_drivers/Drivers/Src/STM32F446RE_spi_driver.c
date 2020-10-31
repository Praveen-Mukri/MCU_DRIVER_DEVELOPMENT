/*
 * STM32F446RE_spi_driver.c
 *
 *  Created on: 08-Oct-2020
 *      Author: praveen mukri
 */

#include "STM32F446RE_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ove_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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
        while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)  == FLAG_RESET );

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


/*********************************************************************
 * @function                - SPI_ReceiveData
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
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
    while( Len > 0)
    {
        // 1. wait until RXNE is set
        while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)  == FLAG_RESET );

        //2. check the DFF bit in CR1
        if( (pSPIx->CR1 & (1 << SPI_CR1_DFF) ) )
        {
            // 16bit DFF
            //1. load the data from DR to RxBuffer address
            *((uint16_t*)pRxBuffer) = pSPIx->DR;
            Len--;
            Len--;
            (uint16_t*)pRxBuffer++;
        }
        else
        {
            //8bit DFF
            *((uint16_t*)pRxBuffer) = pSPIx->DR;
            Len--;
            pRxBuffer++;
        }
    }
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
 * @return            -  state
 *
 * @Note              -  This is interrupt based
 ***********************************************************************/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->TxState;

    if( state != SPI_BUSY_IN_TX)
    {
        //1. Save the Tx Buffer address and Len information in some global variable
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = Len;

        //2. Mask SPI state as busy
        pSPIHandle->TxState = SPI_BUSY_IN_TX;

        //3. Enable the TXEIE control bit to get intterupt whenever TXE flag is set in SR
        pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE);


    }

    return state;

}

/*********************************************************************
 * @function                - SPI_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  Interrupt based
 ***********************************************************************/
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->RxState;

    if( state != SPI_BUSY_IN_RX)
    {
        //1. Save the Rx Buffer address and Len information in some global variable
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = Len;

        //2. Mask SPI state as busy
        pSPIHandle->RxState = SPI_BUSY_IN_RX;

        //3. Enable the TXEIE control bit to get intterupt whenever TXE flag is set in SR
        pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );


    }

    return state;
}

void  SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
    }else
    {
        pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
    }


}



void  SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        pSPIx->CR2 |=  (1 << SPI_CR2_SSOE);
    }else
    {
        pSPIx->CR2 &=  ~(1 << SPI_CR2_SSOE);
    }


}


void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{

    uint8_t temp1, temp2;

    // Check for TXE
    temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
    temp2 = pSPIHandle->pSPIx->SR & ( 1 << SPI_CR2_TXEIE);

    if ( temp1 && temp2)
    {
        // handle TXE
        spi_txe_interrupt_handle(pSPIHandle);
    }

    // Check for RXNE
    temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
    temp2 = pSPIHandle->pSPIx->SR & ( 1 << SPI_CR2_RXNEIE);

    if ( temp1 && temp2)
    {
        // handle TXE
        spi_rxe_interrupt_handle(pSPIHandle);
    }

    // Check for over flag
    temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
    temp2 = pSPIHandle->pSPIx->SR & ( 1 << SPI_CR2_ERRIE);

    if ( temp1 && temp2)
    {
        // handle TXE
        spi_ove_err_interrupt_handle(pSPIHandle);
    }
}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    //2. check the DFF bit in CR1
    if( (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) ) )
    {
        // 16bit DFF
        //1. load the data into the DR
        pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
        pSPIHandle->TxLen--;
        pSPIHandle->TxLen--;
        (uint16_t*)pSPIHandle->pTxBuffer++;
    }
    else
    {
        //8bit DFF
        pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
        pSPIHandle->TxLen--;
        pSPIHandle->pTxBuffer++;
    }

    if( !pSPIHandle->TxLen)
    {
        //this prevents interrupts from setting up of TXE flag
        SPI_CloseTransmisson(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
    }
}

static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    //do rxing as per the dff
    if(pSPIHandle->pSPIx->CR1 & ( 1 << 11))
    {
        //16 bit
        *((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen -= 2;
        pSPIHandle->pRxBuffer++;
        pSPIHandle->pRxBuffer++;

    }else
    {
        //8 bit
        *(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen--;
        pSPIHandle->pRxBuffer++;
    }

    if(! pSPIHandle->RxLen)
    {
        //reception is complete
        SPI_CloseReception(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
    }

}
static void spi_ove_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    uint8_t temp;
    //1. clear the ovr flag
    if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
    {
        temp = pSPIHandle->pSPIx->DR;
        temp = pSPIHandle->pSPIx->SR;
    }
    (void)temp;
    //2. inform the application
    SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

}

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
    pSPIHandle->pTxBuffer = NULL;
    pSPIHandle->TxLen = 0;
    pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
    pSPIHandle->pRxBuffer = NULL;
    pSPIHandle->RxLen = 0;
    pSPIHandle->RxState = SPI_READY;

}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
    uint8_t temp;
    temp = pSPIx->DR;
    temp = pSPIx->SR;
    (void)temp;

}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

    //This is a weak implementation . the user application may override this function.
}
