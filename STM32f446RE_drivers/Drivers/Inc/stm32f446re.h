/*
 * stm32f446re.h
 *
 *  Created on: Sep 26, 2020
 *      Author: praveen mukri
 */

#ifndef INC_STM32F446RE_H_
#define INC_STM32F446RE_H_

#include "stdint.h"

/****************************** Processor specific Details *******************************
 * ARM Cortex M4 Processor NVIC ISERx register Addresses
 *****************************************************************************************/
#define NVIC_ISER0          ((volatile uint32_t*)0xE000E100 )
#define NVIC_ISER1          ((volatile uint32_t*)0xE000E104 )
#define NVIC_ISER2          ((volatile uint32_t*)0xE000E108 )
#define NVIC_ISER3          ((volatile uint32_t*)0xE000E10c )

/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0          ((volatile uint32_t*)0XE000E180 )
#define NVIC_ICER1          ((volatile uint32_t*)0XE000E184 )
#define NVIC_ICER2          ((volatile uint32_t*)0XE000E188 )
#define NVIC_ICER3          ((volatile uint32_t*)0XE000E18C )

/*
 * ARM Cortex M4 Processor priority Register Address
 */
#define NVIC_PR_BASE_ADDR   ((volatile uint32_t*)0xE000E400 )

#define NO_PR_BITS_IMPLEMENTED  4

/*
 * Base address of Flash and SRAM memories
 */
#define FLASH_BASEADDR              0x08000000U
#define SRAM1_BASEADDR              0x20000000U
#define SRAM2_BASEADDR              0x20001C00U
#define ROM_BASEADDR                0x1FFF0000U
#define SRAM                        SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral Base addresses
 */
#define PERIPH_BASEADDR             0x40000000U
#define APB1PERIPH_BASEADDR         PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR         0x40010000U
#define AHB1PERIPH_BASEADDR         0x40020000U
#define AHB2PERIPH_BASEADDR         0x50000000U

/*
 * Base address of peripherals which are hanging on AHB1
 */
#define GPIOA_BASEADDR              (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR              (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR              (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR              (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR              (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR              (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR              (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR              (AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR                (AHB1PERIPH_BASEADDR + 0x3800)

/*
 *  Base address of peripherals which are hanging on APB1
 */
#define I2C1_BASEADDR               (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR               (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR               (APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR               (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR               (APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR             (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR             (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR              (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR              (APB1PERIPH_BASEADDR + 0x5000)

/*
 *  Base address of peripherals which are hanging on APB2
 */
#define EXTI_BASEADDR               (APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR               (APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR             (APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR             (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR             (APB2PERIPH_BASEADDR + 0x1400)


/*************************** PERIPHERAL REGISTER DEFINITION STRUCTURE *************************/

/*
 * Note : Register of peripherals are specific to MCU
 */

/*
 * GPIO Register structure
 */
typedef struct
{
    volatile uint32_t MODER;        /*< GPIO port mode register                                 OffSet: 0x00 */
    volatile uint32_t OTYPER;       /*< GPIO port output type register                          OffSet: 0x04 */
    volatile uint32_t OSPEEDER;     /*< GPIO port output speed register                         OffSet: 0x08 */
    volatile uint32_t PUPDR;        /*< GPIO port pull-up/pull-down register                    OffSet: 0x0C */
    volatile uint32_t IDR;          /*< GPIO port input data register value                     OffSet: 0x10 */
    volatile uint32_t ODR;          /*< GPIO port output data register                          OffSet: 0x14 */
    volatile uint32_t BSRR;         /*< GPIO port bit set/reset register                        OffSet: 0x18 */
    volatile uint32_t LCKR;         /*< GPIO port configuration lock register                   OffSet: 0x1c */
    volatile uint32_t AFR[2];       /*< GPIO alternate function low and High register           OffSet: 0x20 */
}GPIO_RegDef_t;

/*
 * RCC Register structure
 */
typedef struct
{
    volatile uint32_t CR;           /*< RCC clock control register                              OffSet: 0x00 */
    volatile uint32_t PLLCFGR;      /*< RCC PLL configuration register                          OffSet: 0x04 */
    volatile uint32_t CFGR;         /*< RCC clock configuration register                        OffSet: 0x08 */
    volatile uint32_t CIR;          /*< RCC clock interrupt register                            OffSet: 0x0C */
    volatile uint32_t AHB1RSTR;     /*< RCC AHB1 peripheral reset register                      OffSet: 0x10 */
    volatile uint32_t AHB2RSTR;     /*< RCC AHB2 peripheral reset register                      OffSet: 0x14 */
    volatile uint32_t AHB3RSTR;     /*< RCC AHB3 peripheral reset register                      OffSet: 0x18 */
    uint32_t RESERVED0;             /*< RESERVED                                                OffSet: 0x1C */
    volatile uint32_t APB1RSTR;     /*< RCC APB1 peripheral reset register                      OffSet: 0x20 */
    volatile uint32_t APB2RSTR;     /*< RCC APB2 peripheral reset register                      OffSet: 0x24 */
    uint32_t RESERVED1[2];          /*< RESERVED                                                OffSet: 0x28 */
    volatile uint32_t AHB1ENR;      /*< RCC AHB1 peripheral clock enable register               OffSet: 0x30 */
    volatile uint32_t AHB2ENR;      /*< RCC AHB2 peripheral clock enable register               OffSet: 0x34 */
    volatile uint32_t AHB3ENR;      /*< RCC AHB3 peripheral clock enable register               OffSet: 0x38 */
    volatile uint32_t RESERVED2;    /*< RESERVED                                                OffSet: 0x3C */
    volatile uint32_t APB1ENR;      /*< RCC APB1 peripheral clock enable register               OffSet: 0x40 */
    volatile uint32_t APB2ENR;      /*< RCC APB2 peripheral clock enable register               OffSet: 0x44 */
    uint32_t RESERVED3[2];          /*< RESERVED                                                OffSet: 0x48 */
    volatile uint32_t AHB1LPENR;    /*< RCC AHB1 peripheral clock enable in low power mode register     OffSet: 0x50 */
    volatile uint32_t AHB2LPENR;    /*< RCC AHB2 peripheral clock enable in low power mode register     OffSet: 0x54 */
    volatile uint32_t AHB3LPENR;    /*< RCC AHB3 peripheral clock enable in low power mode register     OffSet: 0x58 */
    volatile uint32_t RESERVED4;    /*< RESERVED                                                        OffSet: 0x5C */
    volatile uint32_t APB1LPENR;    /*< RCC APB1 peripheral clock enable in low power mode register     OffSet: 0x60 */
    volatile uint32_t APB2LPENR;    /*< RCC APB2 peripheral clock enabled in low power mode register    OffSet: 0x64 */
    uint32_t RESERVED5[2];          /*< RESERVED                                                OffSet: 0x68 */
    volatile uint32_t BDCR;         /*< RCC Backup domain control register                      OffSet: 0x70 */
    volatile uint32_t CSR;          /*< RCC clock control & status register                     OffSet: 0x74 */
    uint32_t RESERVED6[2];          /*< RESERVED                                                OffSet: 0x78 */
    volatile uint32_t SSCGR;        /*< RCC spread spectrum clock generation register           OffSet: 0x80 */
    volatile uint32_t PLLI2SCFGR;   /*< RCC PLLI2S configuration register                       OffSet: 0x84 */
    volatile uint32_t PLLSAICFGR;   /*< RCC PLL configuration register                          OffSet: 0x88 */
    volatile uint32_t DCKCFGR;      /*< RCC Dedicated Clock Configuration Register              OffSet: 0x8C */
    volatile uint32_t CKGATENR;     /*< RCC clocks gated enable register                        OffSet: 0x90 */
    volatile uint32_t DCKCFGR2;     /*< RCC dedicated clocks configuration register 2           OffSet: 0x94 */
}RCC_RegDef_t;


/*
 * EXTI Register structure
 */
typedef struct
{
    volatile uint32_t IMR;           /*< Interrupt mask register                                offset: 0x00 */
    volatile uint32_t EMR;           /*< Event mask register                                    offset: 0x04 */
    volatile uint32_t RTSR;          /*< Rising trigger selection register                      offset: 0x08 */
    volatile uint32_t FTSR;          /*< Falling trigger selection register                     offset: 0x0C */
    volatile uint32_t SWIER;         /*< Software interrupt event register                      offset: 0x10 */
    volatile uint32_t PR;            /*< Pending register                                       offset: 0x14 */

}EXTI_RegDef_t;

/*
 * SYSCFG Register structure
 */
typedef struct
{
    volatile uint32_t MEMRMP;       /*< SYSCFG memory remap register                            offset: 0x00 */
    volatile uint32_t PMC;          /*< SYSCFG peripheral mode configuration register           offset: 0x04 */
    volatile uint32_t EXTICR[4];    /*< SYSCFG external interrupt configuration register 1      offset: 0x08-0x14 */
    uint32_t RESERVED1[2];          /*< RESERVED                                                offset: 0x18-0x1C */
    volatile uint32_t CMPCR;        /*< Compensation cell control register                      offset: 0x20 */
    uint32_t RESERVED2[2];          /*< RESERVED                                                offset: 0x24-0x28  */
    volatile uint32_t CFGR;         /*< SYSCFG configuration register                           offset: 0x2C */
} SYSCFG_RegDef_t;


/*
 * SPI Register structure
 */
typedef struct
{
    volatile uint32_t CR1;          /*< SPI control register 1                                  offset: 0x00 */
    volatile uint32_t CR2;          /*< SPI control register 2                                  offset: 0x04 */
    volatile uint32_t SR;           /*< SPI status register                                     offset: 0x08 */
    volatile uint32_t DR;           /*< SPI data register                                       offset: 0x0C */
    volatile uint32_t CRCPR;        /*< SPI CRC polynomial register                             offset: 0x10 */
    volatile uint32_t RXCRCR;       /*< SPI RX CRC register                                     offset: 0x14 */
    volatile uint32_t TXCRCR;       /*< SPI TX CRC register                                     offset: 0x18 */
    volatile uint32_t I2SCFGR;      /*< SPI_I2S configuration register                          offset: 0x1C */
    volatile uint32_t I2SPR;        /*< SPI_I2S prescaler register                              offset: 0x20 */

} SPI_RegDef_t;

/*
 * Peripherals definitions ( Peripheral base addresses typecasted to xxx_RegDef_t )
 */
#define GPIOA ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC     ((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI    ((EXTI_RegDef_t*) EXTI_BASEADDR)

#define SYSCFG  ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1    ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2    ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3    ((SPI_RegDef_t*)SPI3_BASEADDR)
/*
 * Clock enable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 0 ))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 1 ))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 2 ))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 3 ))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 4 ))
#define GPIOF_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 5 ))
#define GPIOG_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 6 ))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 7 ))

/*
 * Clock disable macro for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 0 ))
#define GPIOB_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 1 ))
#define GPIOC_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 2 ))
#define GPIOD_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 3 ))
#define GPIOE_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 4 ))
#define GPIOF_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 5 ))
#define GPIOG_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 6 ))
#define GPIOH_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 7 ))

/*
 * Clock enable macro for I2Cx peripherals
 */
#define I2C1_PLCK_EN() (RCC->APB1ENR |= ( 1 << 21 ))
#define I2C2_PLCK_EN() (RCC->APB1ENR |= ( 1 << 22 ))
#define I2C3_PLCK_EN() (RCC->APB1ENR |= ( 1 << 23 ))

/*
 * Clock disable macro for I2Cx peripherals
 */
#define I2C1_PLCK_DI()	(RCC->AHB1ENR &= ~( 1 << 21 ))
#define I2C2_PLCK_DI()	(RCC->AHB1ENR &= ~( 1 << 22 ))
#define I2C3_PLCK_DI()	(RCC->AHB1ENR &= ~( 1 << 23 ))

/*
 * Clock enable macro for SPI peripherals
 */
#define SPI1_PLCK_EN() (RCC->APB2ENR |= ( 1 << 12 ))
#define SPI2_PLCK_EN() (RCC->APB1ENR |= ( 1 << 14 ))
#define SPI3_PLCK_EN() (RCC->APB1ENR |= ( 1 << 15 ))
#define SPI4_PLCK_EN() (RCC->APB2ENR |= ( 1 << 13 ))

/*
 * Clock disable macro for SPI peripherals
 */
#define SPI1_PLCK_DI() (RCC->APB2ENR &= ~( 1 << 12 ))
#define SPI2_PLCK_DI() (RCC->APB1ENR &= ~( 1 << 14 ))
#define SPI3_PLCK_DI() (RCC->APB1ENR &= ~( 1 << 15 ))
#define SPI4_PLCK_DI() (RCC->APB2ENR &= ~( 1 << 13 ))

/*
 * Clock enable macro for SYSCFG peripherals
 */
#define SYSCFG_PLCK_EN() (RCC->APB2ENR |= ( 1 << 14 ))

/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

/*
 * returns port code for given GPIOx base address
 */
#define GPIO_BASEADDT_TO_CODE(x)      ( (x == GPIOA) ? 0:\
                                        (x == GPIOB) ? 1:\
                                        (x == GPIOC) ? 2:\
                                        (x == GPIOD) ? 3:\
                                        (x == GPIOE) ? 4:\
                                        (x == GPIOF) ? 5:\
                                        (x == GPIOG) ? 6:\
                                        (x == GPIOH) ? 7:0)

/*
 * IRQ Number of STM32F446RE
 */
#define IRQ_NO_EXTI0        6
#define IRQ_NO_EXTI1        7
#define IRQ_NO_EXTI2        8
#define IRQ_NO_EXTI3        9
#define IRQ_NO_EXTI4        10
#define IRQ_NO_EXTI9_5      23
#define IRQ_NO_EXTI15_10    40

/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15

/*
 * Some generic macro
 */
#define ENABLE              1
#define DISABLE             0
#define SET                 ENABLE
#define RESET               DISABLE
#define GPIO_PIN_SET        ENABLE
#define GPIO_PIN_RESET      DISABLE

#endif /* INC_STM32F446RE_H_ */
