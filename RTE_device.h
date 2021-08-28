/**
 * @file RTE_device.h
 * @author Mohamed Hassanin Mohamed
 * @brief configure pin assignments and enable peripherals at run time
 * @version 0.1
 * @date 2021-08-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef RTE_DEVICE_H
#define RTE_DEVICE_H

#define SPI0_ENABLE 1
#define SPI1_ENABLE 0
#define SPI2_ENABLE 0
#define SPI3_ENABLE 0

// SPI0

#define SPI0_CLK_PIN PA2
#define SPI0_CLK_ALT DIO_MODE_15

#define SPI0_SS_PIN PA3
#define SPI0_SS_ALT DIO_MODE_15

#define SPI0_MOSI_PIN PA4
#define SPI0_MOSI_ALT DIO_MODE_15

#define SPI0_MISO_PIN PA5
#define SPI0_MISO_ALT DIO_MODE_15

//SPI1

#define SPI1_CLK_PIN PB5
#define SPI1_CLK_ALT DIO_MODE_15

#define SPI1_SS_PIN PB4
#define SPI1_SS_ALT DIO_MODE_15

#define SPI1_MOSI_PIN PE4
#define SPI1_MOSI_ALT DIO_MODE_15

#define SPI1_MISO_PIN PE5
#define SPI1_MISO_ALT DIO_MODE_15

//SPI2

#define SPI2_CLK_PIN PD3
#define SPI2_CLK_ALT DIO_MODE_15

#define SPI2_SS_PIN PD2
#define SPI2_SS_ALT DIO_MODE_15

#define SPI2_MOSI_PIN PD1
#define SPI2_MOSI_ALT DIO_MODE_15

#define SPI2_MISO_PIN PD0
#define SPI2_MISO_ALT DIO_MODE_15

//SPI3

// #define SPI3_CLK_PIN PQ0
// #define SPI3_CLK_ALT 14
#define SPI3_CLK_PIN PF3
#define SPI3_CLK_ALT DIO_MODE_14

// #define SPI3_SS_PIN PQ1
// #define SPI3_SS_ALT 14
#define SPI3_SS_PIN PF2
#define SPI3_SS_ALT DIO_MODE_14

// #define SPI3_MOSI_PIN PQ2
// #define SPI3_CLK_ALT 14
#define SPI3_MOSI_PIN PF1
#define SPI3_CLK_ALT DIO_MODE_14

// #define SPI3_MISO_PIN PQ3
// #define SPI3_SS_ALT 14
#define SPI3_MISO_PIN PF0
#define SPI3_SS_ALT DIO_MODE_14

#endif
