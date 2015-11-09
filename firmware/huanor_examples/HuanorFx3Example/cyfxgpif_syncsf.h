/*
 * Project Name: sync_slave_fifo_2bit.cyfx
 * Time : 12/16/2011 15:29:45
 * Device Type: FX3
 * Project Type: GPIF2
 *
 *
 *
 *
 * This is a generated file and should not be modified
 * This file is generated by Gpif2 designer tool version - 0.9.630.0
 * 
 */

#ifndef _INCLUDED_CYFXGPIF2CONFIG_
#define _INCLUDED_CYFXGPIF2CONFIG_
#include "cyu3types.h"
#include "cyu3gpif.h"

/* Summary
   Number of states in the state machine
 */
#define CY_NUMBER_OF_STATES 6

/* Summary
   Mapping of user defined state names to state indices
 */
#define SYNC_SLAVE_FIFO_2BIT_RESET 0x0
#define SYNC_SLAVE_FIFO_2BIT_IDLE 0x1
#define SYNC_SLAVE_FIFO_2BIT_READ 0x2
#define SYNC_SLAVE_FIFO_2BIT_WRITE 0x3
#define SYNC_SLAVE_FIFO_2BIT_SHORT_PKT 0x4
#define SYNC_SLAVE_FIFO_2BIT_ZLP 0x5


/* Summary
   Macros for pre alphas
 */
#define SYNC_SLAVE_FIFO_2BIT_ALPHA_RESET 0x8


/* Summary
   Transition function values which have to be stored GPIF II transition function registers.
 */
uint16_t Sync_Slave_Fifo_2Bit_CyFxGpifTransition[]  = {
    0x0000, 0x8888, 0x2222, 0x5555, 0x1111
};

/* Summary
   Table containing the transition information for various states. 
   This table has to be stored in the WAVEFORM Registers.
   This array consists of non-replicated waveform descriptors acts as a 
   waveform table. 
 */
CyU3PGpifWaveData Sync_Slave_Fifo_2Bit_CyFxGpifWavedata[]  = {
    {{0x1E706001,0x000100C4,0x80000000},{0x00000000,0x00000000,0x00000000}},
    {{0x3E738302,0x00000200,0x80000000},{0x00000000,0x00000000,0x00000000}},
    {{0x1E706001,0x000100C4,0x80000000},{0x3E738704,0x20000200,0xC0100000}},
    {{0x00000000,0x00000000,0x00000000},{0x00000000,0x00000000,0x00000000}},
    {{0x00000000,0x00000000,0x00000000},{0x3E738705,0x00000200,0xC0100000}},
    {{0x00000000,0x00000000,0x00000000},{0x4E702703,0x20010202,0x80000000}},
    {{0x00000000,0x00000000,0x00000000},{0x3E738704,0x20000200,0xC0100000}}
};

/* Summary
   Table that maps state indices to the descriptor table indices.
 */
uint8_t Sync_Slave_Fifo_2Bit_CyFxGpifWavedataPosition[]  = {
    0,1,0,2,0,0,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
    3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
    0,4,0,2,0,0,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
    3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
    0,5,0,2,0,0,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
    3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
    0,6,0,2,0,0
};

/* Summary
   GPIF II configuration register values.
 */
uint32_t Sync_Slave_Fifo_2Bit_CyFxGpifRegValue[]  = {
    0x80000380,  /*  PIB_GPIF_CONFIG */
#if (CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT == 0)
    0x000010A7,  /*  PIB_GPIF_BUS_CONFIG */
#else
	0x000010AC,  /*  PIB_GPIF_BUS_CONFIG */
#endif
    0x01070002,  /*  PIB_GPIF_BUS_CONFIG2 */
    0x00000044,  /*  PIB_GPIF_AD_CONFIG */
    0x00000000,  /*  PIB_GPIF_STATUS */
    0x00000000,  /*  PIB_GPIF_INTR */
    0x00000000,  /*  PIB_GPIF_INTR_MASK */
    0x00000082,  /*  PIB_GPIF_SERIAL_IN_CONFIG */
    0x00000782,  /*  PIB_GPIF_SERIAL_OUT_CONFIG */
    0x00000500,  /*  PIB_GPIF_CTRL_BUS_DIRECTION */
    0x0000FFCF,  /*  PIB_GPIF_CTRL_BUS_DEFAULT */
    0x000000BF,  /*  PIB_GPIF_CTRL_BUS_POLARITY */
    0x00000000,  /*  PIB_GPIF_CTRL_BUS_TOGGLE */
    0x00000000,  /*  PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  PIB_GPIF_CTRL_BUS_SELECT */
    0x00000018,  /*  PIB_GPIF_CTRL_BUS_SELECT */
    0x00000018,  /*  PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  PIB_GPIF_CTRL_BUS_SELECT */
    0x00000000,  /*  PIB_GPIF_CTRL_BUS_SELECT */
    0x00000006,  /*  PIB_GPIF_CTRL_COUNT_CONFIG */
    0x00000000,  /*  PIB_GPIF_CTRL_COUNT_RESET */
    0x0000FFFF,  /*  PIB_GPIF_CTRL_COUNT_LIMIT */
    0x0000010A,  /*  PIB_GPIF_ADDR_COUNT_CONFIG */
    0x00000000,  /*  PIB_GPIF_ADDR_COUNT_RESET */
    0x0000FFFF,  /*  PIB_GPIF_ADDR_COUNT_LIMIT */
    0x00000000,  /*  PIB_GPIF_STATE_COUNT_CONFIG */
    0x0000FFFF,  /*  PIB_GPIF_STATE_COUNT_LIMIT */
    0x0000010A,  /*  PIB_GPIF_DATA_COUNT_CONFIG */
    0x00000000,  /*  PIB_GPIF_DATA_COUNT_RESET */
    0x0000FFFF,  /*  PIB_GPIF_DATA_COUNT_LIMIT */
    0x00000000,  /*  PIB_GPIF_CTRL_COMP_VALUE */
    0x00000000,  /*  PIB_GPIF_CTRL_COMP_MASK */
    0x00000000,  /*  PIB_GPIF_DATA_COMP_VALUE */
    0x00000000,  /*  PIB_GPIF_DATA_COMP_MASK */
    0x00000000,  /*  PIB_GPIF_ADDR_COMP_VALUE */
    0x00000000,  /*  PIB_GPIF_ADDR_COMP_MASK */
    0x00000000,  /*  PIB_GPIF_DATA_CTRL */
    0x00000000,  /*  PIB_GPIF_INGRESS_DATA */
    0x00000000,  /*  PIB_GPIF_INGRESS_DATA */
    0x00000000,  /*  PIB_GPIF_INGRESS_DATA */
    0x00000000,  /*  PIB_GPIF_INGRESS_DATA */
    0x00000000,  /*  PIB_GPIF_EGRESS_DATA */
    0x00000000,  /*  PIB_GPIF_EGRESS_DATA */
    0x00000000,  /*  PIB_GPIF_EGRESS_DATA */
    0x00000000,  /*  PIB_GPIF_EGRESS_DATA */
    0x00000000,  /*  PIB_GPIF_INGRESS_ADDRESS */
    0x00000000,  /*  PIB_GPIF_INGRESS_ADDRESS */
    0x00000000,  /*  PIB_GPIF_INGRESS_ADDRESS */
    0x00000000,  /*  PIB_GPIF_INGRESS_ADDRESS */
    0x00000000,  /*  PIB_GPIF_EGRESS_ADDRESS */
    0x00000000,  /*  PIB_GPIF_EGRESS_ADDRESS */
    0x00000000,  /*  PIB_GPIF_EGRESS_ADDRESS */
    0x00000000,  /*  PIB_GPIF_EGRESS_ADDRESS */
    0x80010400,  /*  PIB_GPIF_THREAD_CONFIG */
    0x80010401,  /*  PIB_GPIF_THREAD_CONFIG */
    0x80010402,  /*  PIB_GPIF_THREAD_CONFIG */
    0x80010403,  /*  PIB_GPIF_THREAD_CONFIG */
    0x00000000,  /*  PIB_GPIF_LAMBDA_STAT */
    0x00000000,  /*  PIB_GPIF_ALPHA_STAT */
    0x00000000,  /*  PIB_GPIF_BETA_STAT */
    0x00080000,  /*  PIB_GPIF_WAVEFORM_CTRL_STAT */
    0x00000000,  /*  PIB_GPIF_WAVEFORM_SWITCH */
    0x00000000,  /*  PIB_GPIF_WAVEFORM_SWITCH_TIMEOUT */
    0x00000000,  /*  PIB_GPIF_CRC_CONFIG */
    0x00000000,  /*  PIB_GPIF_CRC_DATA */
    0xFFFFFFF1  /*  PIB_GPIF_BETA_DEASSERT */
};

/* Summary
   This structure holds configuration inputs for the GPIF II. 
 */
const CyU3PGpifConfig_t Sync_Slave_Fifo_2Bit_CyFxGpifConfig  = {
    (uint16_t)(sizeof(Sync_Slave_Fifo_2Bit_CyFxGpifWavedataPosition)/sizeof(uint8_t)),
    Sync_Slave_Fifo_2Bit_CyFxGpifWavedata,
    Sync_Slave_Fifo_2Bit_CyFxGpifWavedataPosition,
    (uint16_t)(sizeof(Sync_Slave_Fifo_2Bit_CyFxGpifTransition)/sizeof(uint16_t)),
    Sync_Slave_Fifo_2Bit_CyFxGpifTransition,
    (uint16_t)(sizeof(Sync_Slave_Fifo_2Bit_CyFxGpifRegValue)/sizeof(uint32_t)),
    Sync_Slave_Fifo_2Bit_CyFxGpifRegValue
};

#endif   /* _INCLUDED_CYFXGPIF2CONFIG_ */
