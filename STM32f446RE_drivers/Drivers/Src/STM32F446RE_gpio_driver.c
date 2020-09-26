/*
 * STM32F446RE_gpio_driver.c
 *
 *  Created on: 26-Sep-2020
 *      Author: praveen mukri
 */

#include "STM32F446RE_gpio_driver.h"

/*********************************************************************
 * @function                - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 ***********************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI)
{
    if(ENorDI == ENABLE)
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_EN();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_EN();
        }
    }
    else
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DI();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DI();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DI();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_DI();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_DI();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_DI();
        }
    }
}


/*********************************************************************
 * @function                - GPIO_Init
 *
 * @brief             - This function initialize GPIO pins
 *
 * @param[in]         - GPIO handle structure
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 ***********************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp;
    //1.Configure the mode of pin
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER |= temp;
    }
    else
    {
        //TODO: interrupt mode
    }
    temp = 0;
    //2. Configure speed
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDER |= temp;

    temp = 0;

    //3. Configure pupd
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    temp = 0;

    //4. Configure optype
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    //5. Configure alternative functionality
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        //configure the alt function registers.
        uint8_t temp1, temp2;

        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber  % 8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunction << ( 4 * temp2 ) );
    }
}


/*********************************************************************
 * @function                - GPIO_DeInit
 *
 * @brief             - This function Reinitialize GPIO pins
 *
 * @param[in]         - GPIO base address
 *
 * @return            -  none
 *
 * @Note              -  none
 ***********************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if(pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    else if (pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    else if (pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    else if (pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }
    else if (pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET();
    }
    else if (pGPIOx == GPIOG)
    {
        GPIOG_REG_RESET();
    }
    else if (pGPIOx == GPIOH)
    {
        GPIOH_REG_RESET();
    }

}


/*********************************************************************
 * @function                - GPIO_ReadFromInputPin
 *
 * @brief             - This function reads the value of perticular pin
 *
 * @param[in]         - GPIO base address
 * @param[in]         - GPIO pin number
 *
 * @return            -  GPIO pin value
 *
 * @Note              -  none
 ***********************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t value;

    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
    return value;
}


/*********************************************************************
 * @function                - GPIO_ReadFromInputPort
 *
 * @brief             - This function reads the value of perticular port
 *
 * @param[in]         - GPIO base address
 *
 * @return            -  GPIO port value
 *
 * @Note              -  none
 ***********************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t value;

    value = (uint16_t)pGPIOx->IDR;

    return value;
}


/*********************************************************************
 * @function                - GPIO_WriteToOutputPin
 *
 * @brief             - This function write the value to perticular pin
 *
 * @param[in]         - GPIO base address
 * @param[in]         - GPIO pin number
 * @param[in]         - value
 *
 *
 * @return            -  none
 *
 * @Note              -  none
 ***********************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint16_t Value)
{
    if(Value == GPIO_PIN_SET)
    {
        //write 1 to the output data register at the bit field corresponding to the pin number
        pGPIOx->ODR |= ( 1 << PinNumber);
    }else
    {
        //write 0
        pGPIOx->ODR &= ~( 1 << PinNumber);
    }
}


/*********************************************************************
 * @function                - GPIO_WriteToOutputPort
 *
 * @brief             - This function write the value to perticular port
 *
 * @param[in]         - GPIO base address
 * @param[in]         - value
 *
 *
 * @return            -  none
 *
 * @Note              -  none
 ***********************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR  = Value;

}


/*********************************************************************
 * @function                - GPIO_ToggleOutputPin
 *
 * @brief             - This function toggles the value of given pin
 *
 * @param[in]         - GPIO base address
 * @param[in]         - PinNumber
 *
 *
 * @return            -  none
 *
 * @Note              -  none
 ***********************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR  ^= ( 1 << PinNumber);
}
