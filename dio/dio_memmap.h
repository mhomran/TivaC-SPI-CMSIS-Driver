/**
 * @file dio_memmap.h
 * @author Mohamed Hassanin
 * @brief The memory map of GPIO in TM4C1294NCPDT
 * @version 0.1
 * @date 2021-08-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef DIO_MEMMAP_H
#define DIO_MEMMAP_H
  
#define GPIOA_BASE 0x40058000
#define GPIOB_BASE 0x40059000
#define GPIOC_BASE 0x4005A000
#define GPIOD_BASE 0x4005B000
#define GPIOE_BASE 0x4005C000
#define GPIOF_BASE 0x4005D000
#define GPIOG_BASE 0x4005E000
#define GPIOH_BASE 0x4005F000
#define GPIOJ_BASE 0x40060000
#define GPIOK_BASE 0x40061000
#define GPIOL_BASE 0x40062000
#define GPIOM_BASE 0x40063000
#define GPION_BASE 0x40064000
#define GPIOP_BASE 0x40065000
#define GPIOQ_BASE 0x40066000

#define GPIOA ((volatile GpioHandle_t*)GPIOA_BASE)
#define GPIOB ((volatile GpioHandle_t*)GPIOB_BASE)
#define GPIOC ((volatile GpioHandle_t*)GPIOC_BASE)
#define GPIOD ((volatile GpioHandle_t*)GPIOD_BASE)
#define GPIOE ((volatile GpioHandle_t*)GPIOE_BASE)
#define GPIOF ((volatile GpioHandle_t*)GPIOF_BASE)
#define GPIOG ((volatile GpioHandle_t*)GPIOG_BASE)
#define GPIOH ((volatile GpioHandle_t*)GPIOH_BASE)
#define GPIOJ ((volatile GpioHandle_t*)GPIOJ_BASE)
#define GPIOK ((volatile GpioHandle_t*)GPIOK_BASE)
#define GPIOL ((volatile GpioHandle_t*)GPIOL_BASE)
#define GPIOM ((volatile GpioHandle_t*)GPIOM_BASE)
#define GPION ((volatile GpioHandle_t*)GPION_BASE)
#define GPIOP ((volatile GpioHandle_t*)GPIOP_BASE)
#define GPIOQ ((volatile GpioHandle_t*)GPIOQ_BASE)

typedef struct {
  uint32_t DATA;
  uint32_t reserved0[0xFF];
  uint32_t DIR;
  uint32_t IS;
  uint32_t IBE;
  uint32_t IEV;
  uint32_t IM;
  uint32_t RIS;
  uint32_t MIS;
  uint32_t ICR;
  uint32_t AFSEL;
  uint32_t reserved1[0x37];
  uint32_t R2R;
  uint32_t R4R;
  uint32_t R8R;
  uint32_t ODR;
  uint32_t PUR;
  uint32_t PDR;
  uint32_t SLR;
  uint32_t DEN;
  uint32_t LOCK;
  uint32_t CR;
  uint32_t AMSEL;
  uint32_t PCTL;
  uint32_t ADCCTL;
  uint32_t DMACTL;
  uint32_t SI;
  uint32_t DR12R;
  uint32_t WAKEPEN;
  uint32_t WAKELVL;
  uint32_t WAKESTAT;
  uint32_t reserved2[0x29D];
  uint32_t GPIOPP;
  uint32_t GPIOPC;
} GpioHandle_t;

#endif
