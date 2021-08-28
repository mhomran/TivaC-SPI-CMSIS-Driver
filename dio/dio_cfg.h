/** 
 * @file dio_cfg.h
 * @author Mohamed Hassanin
 * @brief This module contains interface definitions for the
 * Dio configuration. This is the header file for the definition of the
 * interface for retrieving the digital input/output configuration table.
 * @version 0.1
 * @date 2021-01-12
*/
#ifndef DIO_CFG_H_
#define DIO_CFG_H_
/**********************************************************************
* includes
**********************************************************************/
#include <inttypes.h>
/**********************************************************************
* Preprocessor Constants
**********************************************************************/
/**
* Defines the number of pins on each processor port.
*/
#define DIO_CHANNELS_PER_PORT 8U
/**
* Defines the number of ports on the processor.
*/
#define DIO_NUMBER_OF_PORTS 4U
/**********************************************************************
* Typedefs
**********************************************************************/
/**
* Defines the possible states for a digital output pin.
*/
typedef enum
{
  DIO_STATE_LOW, /**< Defines digital state ground */
  DIO_STATE_HIGH, /**< Defines digital state power */
  DIO_STATE_MAX /**< the maximum number of states */
}DioState_t;

/**
 * Defines the possible directions of the pin
 */
typedef enum 
{
  DIO_DIR_INPUT, 
  DIO_DIR_OUTPUT
}DioDirection_t;

/**
* Defines an enumerated list of all the channels (pins) on the MCU
* device. The last element is used to specify the maximum number of
* enumerated labels.
*/
typedef enum
{
  /* TODO: Populate this list based on available MCU pins */ 
  PA0 = 0  * DIO_CHANNELS_PER_PORT, PA1, PA2, PA3, PA4, PA5, PA6, PA7,
  PB0 = 1  * DIO_CHANNELS_PER_PORT, PB1, PB2, PB3, PB4, PB5, 
  PC0 = 2  * DIO_CHANNELS_PER_PORT, PC1, PC2, PC3, PC4, PC5, PC6, PC7,
  PD0 = 3  * DIO_CHANNELS_PER_PORT, PD1, PD2, PD3, PD4, PD5, PD6, PD7,
  PE0 = 4  * DIO_CHANNELS_PER_PORT, PE1, PE2, PE3, PE4, PE5,
  PF0 = 5  * DIO_CHANNELS_PER_PORT, PF1, PF2, PF3, PF4, 
  PG0 = 6  * DIO_CHANNELS_PER_PORT, PG1,
  PH0 = 7  * DIO_CHANNELS_PER_PORT, PH1, PH2, PH3,
  PJ0 = 8  * DIO_CHANNELS_PER_PORT, PJ1,
  PK0 = 9  * DIO_CHANNELS_PER_PORT, PK1, PK2, PK3, PK4, PK5, PK6, PK7,
  PL0 = 10 * DIO_CHANNELS_PER_PORT, PL1, PL2, PL3, PL4, PL5, PL6, PL7,
  PM0 = 11 * DIO_CHANNELS_PER_PORT, PM1, PM2, PM3, PM4, PM5, PM6, PM7,
  PN0 = 12 * DIO_CHANNELS_PER_PORT, PN1, PN2, PN3, PN4, PN5,
  PP0 = 13 * DIO_CHANNELS_PER_PORT, PP1, PP2, PP3, PP4, PP5,
  PQ0 = 14 * DIO_CHANNELS_PER_PORT, PQ1, PQ2, PQ3, PQ4,
  DIO_CHANNEL_MAX
}DioChannel_t;

typedef enum 
{
  DIO_MODE_0,
  DIO_MODE_1,
  DIO_MODE_2,
  DIO_MODE_3,
  DIO_MODE_4,
  DIO_MODE_5,
  DIO_MODE_6,
  DIO_MODE_7,
  DIO_MODE_8,
  DIO_MODE_9,
  DIO_MODE_10,
  DIO_MODE_11,
  DIO_MODE_12,
  DIO_MODE_13,
  DIO_MODE_14,
  DIO_MODE_15,
  DIO_MODE_GPIO,
  DIO_MODE_MAX
}DioMode_t;

/**
* Defines the digital input/output configuration table’s elements that are used
* by Dio_Init to configure the Dio peripheral.
*/
typedef struct
{
  DioChannel_t Channel; /**< The I/O pin */
  DioDirection_t Direction; /**< OUTPUT or INPUT */
  DioState_t State; /**< HIGH or LOW */
  DioMode_t Mode; /**< Mode */
}DioConfig_t;

/**
* Defines the digital input/output configuration table’s elements that are used
* by Dio_Init to configure the Dio peripheral.
*/
typedef struct
{
  uint16_t ChannelsSize; 
  const DioConfig_t* ChannelsConfig; /**< OUTPUT or INPUT */
}DioInitConfig_t;

/**********************************************************************
* Function Prototypes
**********************************************************************/
#ifdef __cplusplus
extern "C"{
#endif

const DioInitConfig_t* Dio_ConfigGet(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* DIO_CFG_H_*/
/************************* END OF FILE ********************************/
