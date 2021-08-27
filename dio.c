/** 
 * @file dio.c
 * @author Mohamed Hassanin
 * @brief The implementation for the dio.
 * @version 0.1
 * @date 2021-01-12
 */
/**********************************************************************
* Includes
**********************************************************************/
#include <inttypes.h>
#include "dio.h" /* For this modules definitions */
#include "dio_memmap.h" /* For Hardware definitions */
/**********************************************************************
* Macros
**********************************************************************/
#define SYS_CTL_BASE 0x400FE000
#define RCGCGPIO_OFFSET 0x608
#define PRGPIO_OFFSET 0xA08
#define RCGCGPIO *((volatile uint32_t*)(SYS_CTL_BASE+RCGCGPIO_OFFSET))
#define PRGPIO *((volatile uint32_t*)(SYS_CTL_BASE+PRGPIO_OFFSET))
/**********************************************************************
* Module Variable Definitions
**********************************************************************/
static volatile GpioHandle_t* gGPIOs[] =
{
  GPIOA,
  GPIOB,
  GPIOC,
  GPIOD,
  GPIOE,
  GPIOF,
  GPIOG,
  GPIOH,
  GPIOJ,
  GPIOK,
  GPIOL,
  GPIOM,
  GPION,
  GPIOP,
  GPIOQ
};
/**********************************************************************
* Function Prototypes
**********************************************************************/
static void Dio_EnableChannel(DioChannel_t);
static uint8_t Dio_EnableClk(uint8_t Port);
/**********************************************************************
* Function Definitions
**********************************************************************/
/*********************************************************************
* Function : Dio_Init()
*//**
* \b Description:
* This function is used to initialize the Dio based on the configuration
* table defined in dio_cfg module. <br>
* PRE-CONDITION: Configuration table needs to populated (sizeof > 0) <br>
* PRE-CONDITION: NUMBER_OF_CHANNELS_PER_PORT > 0 <br>
* PRE-CONDITION: NUMBER_OF_PORTS > 0 <br>
* PRE-CONDITION: The MCU clocks must be configured and enabled. <br>
* POST-CONDITION: The DIO peripheral is set up with the configuration settings.<br>
* @param Config is a pointer to the configuration table that
* contains the initialization for the peripheral.
* @return void
*
* \b Example:
* @code
* const DioConfig_t *DioConfig = Dio_ConfigGet();
* Dio_Init(DioConfig);
* @endcode
* @see Dio_ConfigGet
**********************************************************************/
void 
Dio_Init(const DioInitConfig_t * Config)
{
  uint16_t Port;
  const DioConfig_t* Channels = Config->ChannelsConfig;
  uint16_t ChannelsSize = Config->ChannelsSize;
  uint16_t i;

  for (i = 0; i < ChannelsSize; i++)
    {
      Port = Channels[i].Channel / DIO_CHANNELS_PER_PORT;

      if(Dio_EnableClk(Port) == 0)
  {
    while(1);
  }

      Dio_SetPinDirection(Channels[i].Channel, Channels[i].Direction);
      if(Channels[i].Direction == DIO_DIR_OUTPUT)
        {
          Dio_WriteChannel(Channels[i].Channel, Channels[i].State);
        }
      Dio_SetPinMode(Channels[i].Channel, Channels[i].Mode);
      Dio_EnableChannel(Channels[i].Channel);
    }
}

/**********************************************************************
* Function : Dio_ReadChannel()
*//**
* \b Description:
* This function is used to read the state of a dio channel (pin) <br>
* PRE-CONDITION: The channel is configured as INPUT <br>
* PRE-CONDITION: The channel is configured as GPIO <br>
* PRE-CONDITION: The channel is within the maximum DioChannel_t definition <br>
* POST-CONDITION: The channel state is returned.<br>
* @param Channel is the DioChannel_t that represents a pin
* @return The state of the channel as HIGH or LOW
*
* \b Example:
* @code
* uint8_t_t pin = Dio_ReadChannel(PORT1_0);
* @endcode
* @see Dio_Init
**********************************************************************/
DioState_t
Dio_ReadChannel(DioChannel_t Channel)
{
  uint16_t Port = 0; 
  volatile GpioHandle_t* GPIO;
  uint32_t PortState;
  uint32_t PinMask;
  Port = Channel / DIO_CHANNELS_PER_PORT;
  GPIO = gGPIOs[Port];

  PortState = GPIO->DATA;
  PinMask = (1UL << (Channel % DIO_CHANNELS_PER_PORT));

  return ((PortState & PinMask) ? DIO_STATE_HIGH : DIO_STATE_LOW);
}

/**********************************************************************
* Function : Dio_WriteChannel()
*//**
* \b Description:
* This function is used to write the state of a channel (pin) as either<br>
* logic high or low through the use of the DioChannel_t enum to select<br>
* the channel and the DioState_t define the desired state.<br>
* PRE-CONDITION: The channel is configured as OUTPUT <br>
* PRE-CONDITION: The channel is configured as GPIO <br>
* PRE-CONDITION: The channel is within the maximum DioChannel_t definition <br>
* POST-CONDITION: The channel state will be State <br>
* @param Channel is the pin to write using the DioChannel_t enum definition <br>
* @param State is HIGH or LOW as defined in the DioState_tum <br>
* @return void
*
* \b Example:
* @code
* Dio_WriteChannel(PORT1_0, LOW); // Set the PORT1_0 pin low
* Dio_WriteChannel(PORT1_0, HIGH); // Set the PORT1_0 pin high
* @endcode
* @see Dio_Init
**********************************************************************/
void 
Dio_WriteChannel(DioChannel_t Channel, DioState_t State)
{
  uint16_t Port = 0; 
  volatile GpioHandle_t* GPIO;
  uint32_t PinMask;
  Port = Channel / DIO_CHANNELS_PER_PORT;
  GPIO = gGPIOs[Port];

  PinMask = (1UL << (Channel % DIO_CHANNELS_PER_PORT));

  if (State == DIO_STATE_HIGH)
    {
      GPIO->DATA |= PinMask;
    }
  else
    {
      GPIO->DATA &= ~PinMask;
    }
}

/**************************************************************************
* Function : Dio_SetPinDirection()
*//**
* \b Description:
* This function is used to set the direction of a channel.<br>
* PRE-CONDITION: The channel is within the maximum DioChannel_t definition <br>
* POST-CONDITION: The direction of the channel is changed.<br>
* @param Channel is the pin from the DioChannel_t that is to be modified. <br>
* @return void
*
* \b Example:
* @code
* Dio_SetPinDirection(PORTA_1, INPUT);
* @endcode
* @see Dio_Init
**********************************************************************/
void 
Dio_SetPinDirection(DioChannel_t Channel, DioDirection_t Direction)
{
  uint16_t Port = 0; 
  volatile GpioHandle_t* GPIO;
  uint32_t PinMask;

  Port = Channel / DIO_CHANNELS_PER_PORT;
  GPIO = gGPIOs[Port];
  PinMask = (1UL << (Channel % DIO_CHANNELS_PER_PORT));

  if(Direction == DIO_DIR_OUTPUT)
    {
      GPIO->DIR |= PinMask;
    }
  else
    {
      GPIO->DIR &= ~PinMask;
    }
}

/**************************************************************************
* Function : Dio_SetPinMode()
*//**
* \b Description:
* This function is used to set the mode of a channel.<br>
* PRE-CONDITION: The channel is within the maximum DioChannel_t definition <br>
* POST-CONDITION: The mode of the channel is changed.<br>
* @param Channel is the pin from the DioChannel_t that is to be modified. <br>
* @param Mode <br>
* @return void
*
* \b Example:
* @code
* Dio_SetPinMode(PA0, DIO_MODE_0);
* @endcode
* @see Dio_Init
**********************************************************************/
void 
Dio_SetPinMode(DioChannel_t Channel, DioMode_t Mode)
{
  uint16_t Port = 0; 
  volatile GpioHandle_t* GPIO;
  uint32_t PinMask;

  Port = Channel / DIO_CHANNELS_PER_PORT;
  GPIO = gGPIOs[Port];

  PinMask = (1UL << (Channel % DIO_CHANNELS_PER_PORT));
  if(Mode != DIO_MODE_GPIO)
    {
      uint32_t PMCPostion; //PORT_MUX_CTL

      GPIO->AFSEL |= PinMask;
      PMCPostion = 4 * (Channel % DIO_CHANNELS_PER_PORT);
      GPIO->PCTL |= Mode << PMCPostion;
    }
  else
    {
      GPIO->AFSEL &= ~PinMask;
    }
}

/**************************************************************************
* Function : Dio_RegisterWrite()
*//**
* \b Description:
* This function is used to directly address and modify a Dio register. <br>
* The function should be used to access specialied functionality in the <br>
* Dio peripheral that is not exposed by any other function of the interface. <br>
* PRE-CONDITION: Address is within the boundaries of the Dio register 
* addresss space <br>
* POST-CONDITION: The register located at Address with be updated
* with Value <br>
* @param Address is a register address within the Dio
* peripheral map
* @param Value is the value to set the Dio register to
* @return void
*
* \b Example:
* @code
* Dio_RegisterWrite(VALID_DIO_ADDRESS, 0x15);
* @endcode
**********************************************************************/
void 
Dio_RegisterWrite(uint32_t volatile * const Address, uint32_t Value)
{
  //TODO: Assert that this address is in range of Dio addresses
  *Address = Value;
}
/**********************************************************************
* Function : Dio_RegisterRead()
*//**
* \b Description:
*
* This function is used to directly address a Dio register. The function <br>
* should be used to access specialied functionality in the Dio peripheral<br>
* that is not exposed by any other function of the interface.<br>
* PRE-CONDITION: Address is within the boundaries of the Dio register
* addresss space<br>
* POST-CONDITION: The value stored in the register is returned to the
* caller<br>
* @param Address is the address of the Dio register to read
* @return The current value of the Dio register.
*
* \b Example:
* @code
* DioValue = Dio_RegisterRead(VALID_DIO_ADDRESS);
* @endcode
**********************************************************************/
uint32_t
Dio_RegisterRead(const volatile uint32_t * const Address)
{
  //TODO: Assert that this address is in range of Dio addresses
  return *Address;
}


/**************************************************************************
* Function : Dio_EnableChannel()
*//**
* \b Description:
* Utility function to enable digital functions for this channel.
* PRE-CONDITION: The channel is within the maximum DioChannel_t definition <br>
* POST-CONDITION: The digital function of the channel is enabled.<br>
* @param Channel is the pin from the DioChannel_t that is to be modified. <br>
* @return void
*
* @see Dio_Init
**********************************************************************/
static void 
Dio_EnableChannel(DioChannel_t Channel)
{
  uint16_t Port = 0; 
  volatile GpioHandle_t* GPIO;
  uint32_t PinMask;

  Port = Channel / DIO_CHANNELS_PER_PORT;
  GPIO = gGPIOs[Port];
  PinMask = (1UL << (Channel % DIO_CHANNELS_PER_PORT));

  GPIO->DEN |= PinMask;
}

static uint8_t
Dio_EnableClk(uint8_t Port)
{
  if(RCGCGPIO & (1 << Port))
    {
      //DO NOTHING
      return 1;
    }
  else
    {
      uint16_t timeout = 0xFFFF;
      RCGCGPIO |= 1 << Port;
      while(!(PRGPIO & (1 << Port)) && timeout != 0)
        {
          timeout--;
        }
      return (timeout != 0);
    }
}
/*************** END OF FUNCTIONS ********************************/
