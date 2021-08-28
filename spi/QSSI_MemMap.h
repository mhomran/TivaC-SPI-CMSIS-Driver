#ifndef QSSI_MEMMAP_H
#define QSSI_MEMMAP_H
#include <inttypes.h>

#define QSSI0_BASE 0x40008000
#define QSSI1_BASE 0x40009000
#define QSSI2_BASE 0x4000A000
#define QSSI3_BASE 0x4000B000

#define QSSI0 ((volatile QssiHandle_t*) QSSI0_BASE) 
#define QSSI1 ((volatile QssiHandle_t*) QSSI1_BASE) 
#define QSSI2 ((volatile QssiHandle_t*) QSSI2_BASE) 
#define QSSI3 ((volatile QssiHandle_t*) QSSI3_BASE) 

#define SYS_CLK_BASE 0x400FE000
#define RCGCSSI_OFFSET 0x61C
#define RCGCSSI *((volatile uint32_t*)(SYS_CLK_BASE+RCGCSSI_OFFSET))

typedef struct {
  uint32_t CR0;
  uint32_t CR1;
  uint32_t DR;
  uint32_t SR;
  uint32_t CPSR;
  uint32_t IM;
  uint32_t RIS;
  uint32_t MIS;
  uint32_t ICR;
  uint32_t DMACTL;
  uint32_t RESERVED[0x3E6];
  uint32_t PP;
  uint32_t CC;
} QssiHandle_t;


// bits
#define QSSI_CR1_LBM_Pos 0
#define QSSI_CR1_SSE_Pos 1
#define QSSI_CR1_MS_Pos 2

#define QSSI_CR0_SCR_Pos 8
#define QSSI_CR0_SPH_Pos 7
#define QSSI_CR0_SPO_Pos 6
#define QSSI_CR0_FRF_Pos 4
#define QSSI_CR0_FRF_Msk (0b11 << 4)
#define QSSI_CR0_DSS_Pos 0

#define QSSI_CPSR_CPSDVSR_Pos 0

#endif 
