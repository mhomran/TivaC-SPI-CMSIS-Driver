/*
 * Copyright (c) 2013-2020 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**********************************************************************
* Includes
**********************************************************************/
#include "Driver_SPI.h"
#include "QSSI_MemMap.h"
/**********************************************************************
* Macros
**********************************************************************/
#define ARM_SPI_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0) /* driver version */
#define SYS_CLK (16000000ul)
#define NULL 0
/**********************************************************************
* Module Variable Definitions
**********************************************************************/
ARM_SPI_SignalEvent_t gSPI_CallBack[4];

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
    ARM_SPI_API_VERSION,
    ARM_SPI_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_SPI_CAPABILITIES DriverCapabilities = {
    0, /* Reserved (must be zero) */
    1, /* TI Synchronous Serial Interface */
    0, /* Microwire Interface */
    0, /* Signal Mode Fault event: \ref ARM_SPI_EVENT_MODE_FAULT */
    0  /* Reserved (must be zero) */
};
/**********************************************************************
* Function Prototypes
**********************************************************************/
static int32_t Spi_SetBaudrate(volatile QssiHandle_t* QSSI, uint32_t Baudrate);
static int32_t Spi_SetFrameFormat(volatile QssiHandle_t* QSSI, uint32_t control);
static int32_t Spi_SetMasterSSMode(volatile QssiHandle_t* QSSI, uint32_t control);
static int32_t Spi_SetSlaveSSMode(volatile QssiHandle_t* QSSI, uint32_t control);
static int32_t Spi_SetupAsMaster(volatile QssiHandle_t* QSSI, uint32_t control, uint32_t arg);
static int32_t ARM_SPI_Control(volatile QssiHandle_t* QSSI, uint32_t control, uint32_t arg);

/**********************************************************************
* Function Definitions
**********************************************************************/

/**
 * @brief Get the driver version
 * 
 * @return ARM_DRIVER_VERSION 
 */
static ARM_DRIVER_VERSION 
ARM_SPI_GetVersion(void)
{
  return DriverVersion;
}

/**
 * @brief Get the driver capabilities
 * 
 * @return ARM_SPI_CAPABILITIES 
 */
static ARM_SPI_CAPABILITIES 
ARM_SPI_GetCapabilities(void)
{
  return DriverCapabilities;
}

/**
 * @brief Assign the callback and any necessary hardware that SPI uses
 * (e.g. GPIO)
 * @param cb_event 
 * @return int32_t 
 */
static int32_t 
ARM_SPI_Initialize(uint8_t QSSIID, ARM_SPI_SignalEvent_t cb_event)
{
  gSPI_CallBack[QSSIID] = cb_event;
  return ARM_DRIVER_OK;
}

/**
 * @brief Assign the callback and any necessary hardware that SPI0 uses
 * (e.g. GPIO)
 * @param cb_event 
 * @return int32_t 
 */
static int32_t 
ARM_SPI0_Initialize(ARM_SPI_SignalEvent_t cb_event)
{
  return ARM_SPI_Initialize(0, cb_event);
}

/**
 * @brief Uninitialize any hardware that SPI uses
 * (e.g. GPIO)
 * @param cb_event 
 * @return int32_t 
 */
static int32_t 
ARM_SPI_Uninitialize(void)
{
    return ARM_DRIVER_OK;
}

/**
 * @brief Control the power of the SPI peripheral
 * @param state 
 * @return int32_t 
 */
static int32_t 
ARM_SPI_PowerControl(ARM_POWER_STATE state)
{
  uint16_t delay;

  switch (state)
  {
  case ARM_POWER_OFF:
    {
      RCGCSSI &= ~(1 << 0); //disable the clock
    }
    break;

  case ARM_POWER_LOW:
    break;

  case ARM_POWER_FULL:
    {
      RCGCSSI |= 1 << 0; //enable the clock
      for(delay = 0xFFFF; delay != 0; delay--);
    }
    break;
  }
  return ARM_DRIVER_OK;
}


/**
 * @brief 
 * 
 * @param data a pointer to the data items to be sent
 * @param num the number of items to be sent
 * @return int32_t 
 */
static int32_t 
ARM_SPI_Send(volatile QssiHandle_t* QSSI, const void *data, uint32_t num)
{
  QSSI->DR = *((const uint8_t*)data);
  return ARM_DRIVER_OK;
}

/**
 * @brief 
 * 
 * @param data a pointer to the data items to be sent
 * @param num the number of items to be sent
 * @return int32_t 
 */
static int32_t 
ARM_SPI0_Send(const void *data, uint32_t num)
{
  return ARM_SPI_Send(QSSI0, data, num);
}

/**
 * @brief 
 * 
 * @param data a pointer to the data items to be received
 * @param num the number of items to be sent
 * @return int32_t 
 */
static int32_t 
ARM_SPI_Receive(void *data, uint32_t num)
{
  *((uint8_t*)data) = (uint8_t)(QSSI0->DR);
  return ARM_DRIVER_OK;
}

/**
 * @brief 
 * 
 * @param data a pointer to the data items to be received
 * @param num the number of items to be sent
 * @return int32_t 
 */
static int32_t 
ARM_SPI0_Receive(const void *data, uint32_t num)
{
  return ARM_SPI_Receive(QSSI0, data, num);
}

static int32_t 
ARM_SPI_Transfer(const void *data_out, void *data_in, uint32_t num)
{
    return ARM_DRIVER_OK;
}

static uint32_t 
ARM_SPI_GetDataCount(void)
{
    return ARM_DRIVER_OK;
}

/**
 * @brief Set the baudrate
 * 
 * @param QSSI 
 * @param Baudrate 
 * @return int32_t 
 */
static int32_t 
Spi_SetBaudrate(volatile QssiHandle_t* QSSI, uint32_t Baudrate) 
{
  // SSICLK = SysClk / (CPSDVSR * (1 + SCR))
  // CPSDVSR = 2
  // CPSDVSR must be from 2 to 254 and even
  // SCR from (0 to 255)
  // SCR = (SysClk / (SSICLK * 2)) - 1 = (16M / (arg * 2)) - 1
  if(QSSI == NULL)
    {
      return ARM_DRIVER_ERROR_PARAMETER;
    }

  uint32_t SCR = (SYS_CLK / 2 / Baudrate - 1);
  if(SCR > 255) 
    {
      return ARM_DRIVER_ERROR_PARAMETER;
    }
  QSSI->CR0 |= SCR << QSSI_CR0_SCR_Pos;
  QSSI->CPSR |= 2 << QSSI_CPSR_CPSDVSR_Pos;

  return ARM_DRIVER_OK;
}

static int32_t 
Spi_SetFrameFormat(volatile QssiHandle_t* QSSI, uint32_t control)
{
  uint8_t FreescaleFrame = 1;

  switch (control & ARM_SPI_FRAME_FORMAT_Msk)
  {
  case ARM_SPI_CPOL0_CPHA0:
    {
      QSSI->CR0 &= ~(1 << QSSI_CR0_SPO_Pos); 
      QSSI->CR0 &= ~(1 << QSSI_CR0_SPH_Pos);
    }
    break;
  case ARM_SPI_CPOL0_CPHA1:
    {
      QSSI->CR0 &= ~(1 << QSSI_CR0_SPO_Pos); 
      QSSI->CR0 |= (1 << QSSI_CR0_SPH_Pos);
    }
    break;
  case ARM_SPI_CPOL1_CPHA0:
    {
      QSSI->CR0 |= (1 << QSSI_CR0_SPO_Pos);
      QSSI->CR0 &= ~(1 << QSSI_CR0_SPH_Pos); 
    }
    break;
  case ARM_SPI_CPOL1_CPHA1:
    {
      QSSI->CR0 |= (1 << QSSI_CR0_SPO_Pos);
      QSSI->CR0 |= (1 << QSSI_CR0_SPH_Pos);
    }
    break;
  case ARM_SPI_TI_SSI:
    {
      FreescaleFrame = 0;
    }
    break;
  default:
    return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  if(FreescaleFrame != 0)
    {
      uint32_t DataLength = (control & ARM_SPI_DATA_Msk) >> ARM_SPI_DATA_Pos;
      if(4 <= DataLength && DataLength <= 16)
        {
          QSSI->CR0 |= (DataLength-1) << QSSI_CR0_DSS_Pos; 
        }
      else
        {
          return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
      QSSI->CR0 &= ~(QSSI_CR0_FRF_Msk); //Freescale Format
    }
  else
    {
      QSSI->CR0 &= ~(QSSI_CR0_FRF_Msk); 
      QSSI->CR0 |= (QSSI_CR0_FRF_Pos); 
    }

  return ARM_DRIVER_OK;
}

static int32_t
Spi_SetMasterSSMode(volatile QssiHandle_t* QSSI, uint32_t control)
{
  switch (control & ARM_SPI_SS_MASTER_MODE_Msk)
  {
  case ARM_SPI_SS_MASTER_UNUSED:
    break;
  case ARM_SPI_SS_MASTER_SW:
    break;
  case ARM_SPI_SS_MASTER_HW_OUTPUT:
    break;
  case ARM_SPI_SS_MASTER_HW_INPUT:
    break;
  default:
    return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}

static int32_t
Spi_SetSlaveSSMode(volatile QssiHandle_t* QSSI, uint32_t control)
{
  switch(control & ARM_SPI_SS_SLAVE_MODE_Msk)
  {
  case ARM_SPI_SS_SLAVE_HW:
    break;
  case ARM_SPI_SS_SLAVE_SW:
    break;
  default:
    return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
  return ARM_DRIVER_OK;
}

/**
 * @brief setup the QSSI as a master
 * 
 * @param QSSI a handler to the QSSI peripheral
 * @param control control bits
 * @param arg the baudrate
 * @return int32_t 
 */
static int32_t 
Spi_SetupAsMaster(volatile QssiHandle_t* QSSI, uint32_t control, uint32_t arg)
{
  int32_t state;

  QSSI->CR1 &= ~(1 << QSSI_CR1_SSE_Pos); //disable the peripheral
  QSSI->CR1 &= ~(1 << QSSI_CR1_MS_Pos); //master
  
  state = Spi_SetFrameFormat(QSSI, control);
  if(state != ARM_DRIVER_OK)  
    {
      return state;
    }
  state = Spi_SetBaudrate(QSSI, arg);
  if(state != ARM_DRIVER_OK)
    {
      return state;
    }
  state = Spi_SetMasterSSMode(QSSI, control);
  if(state != ARM_DRIVER_OK)
    {
      return state;
    }

  //for debugging
  QSSI->CR1 |= 1 << QSSI_CR1_LBM_Pos;

  QSSI->CR1 |= (1 << QSSI_CR1_SSE_Pos); //enable the peripheral

  return ARM_DRIVER_OK;
}

static int32_t 
ARM_SPI0_Control(uint32_t control, uint32_t arg)
{
    return ARM_SPI_Control(QSSI0, control, arg);
}

static int32_t
ARM_SPI_Control(volatile QssiHandle_t* QSSI, uint32_t control, uint32_t arg)
{
  int32_t state = ARM_DRIVER_OK;

  switch (control & ARM_SPI_CONTROL_Msk)
  {
  case ARM_SPI_MODE_INACTIVE:             // SPI Inactive
    {
      QSSI->CR0 &= ~(1 << QSSI_CR0_SCR_Pos); //disable the peripheral
    }
    break;

  case ARM_SPI_MODE_MASTER:               // SPI Master (Output on MOSI, Input on MISO); arg = Bus Speed in bps
    {
      state = Spi_SetupAsMaster(QSSI, control, arg);
    }
    break;

  case ARM_SPI_MODE_SLAVE:                // SPI Slave  (Output on MISO, Input on MOSI)
    {
      state = Spi_SetSlaveSSMode(QSSI, control);
    }
    break;

  //Miscellaneous
  case ARM_SPI_SET_BUS_SPEED:
    {
      state = Spi_SetBaudrate(QSSI, arg);
    }
    break;
  case ARM_SPI_GET_BUS_SPEED:
    break;
  case ARM_SPI_SET_DEFAULT_TX_VALUE:
    break;
  case ARM_SPI_CONTROL_SS:
    break;
  case ARM_SPI_ABORT_TRANSFER:
    break;

  default:
    return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return state;
}

static ARM_SPI_STATUS 
ARM_SPI_GetStatus(void)
{
  ARM_SPI_STATUS status = {0};
  return status;
}

static void 
ARM_SPI0_SignalEvent(uint32_t event)
{
  // function body
  if(gSPI_CallBack[0] != NULL)
    {
      gSPI0_CallBack(event);
    }
}

// End SPI Interface

extern \
ARM_DRIVER_SPI Driver_SPI0;
ARM_DRIVER_SPI Driver_SPI0 = {
  ARM_SPI_GetVersion,
  ARM_SPI_GetCapabilities,
  ARM_SPI0_Initialize,
  ARM_SPI_Uninitialize,
  ARM_SPI_PowerControl,
  ARM_SPI0_Send,
  ARM_SPI0_Receive,
  ARM_SPI_Transfer,
  ARM_SPI_GetDataCount,
  ARM_SPI0_Control,
  ARM_SPI_GetStatus
};
