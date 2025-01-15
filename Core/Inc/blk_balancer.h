/**
  ******************************************************************************
  * @file    blkle contains all the function prototypes for
  *          the passive_balance.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 Rene Schoenrock.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BLK_BALANCER_H__
#define __BLK_BALANCER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"


#define BB_MAX_CHANNEL 7

typedef enum
{
	NO_BB,
	BB_RUN_MODE,
	BB_MODE_END
}_BLK_BALANCER_STATE;

/**
 * @struct	REG
 * @brief	Registersatz des Controllers.
 *
 * @note	Der Registersatz wird im RAM und im EEProm gehalten
 */
 typedef struct
 {
	 uint8_t	time_index;				//10ms index of running pb
	 uint8_t	ch_ebable_mask;
	 uint8_t	ch_dir_mask;
	 uint8_t	ch_val[BB_MAX_CHANNEL];
	 uint8_t	last_spi_buf[3];
 }_BB_CTRL_STRUCT;


void BlkBalancer_init(void);
void BlkBalancer_set(uint8_t channel, uint8_t value, uint8_t direction);
uint8_t	process_BlkBalancer(void);
void BB_OutputEnable(GPIO_PinState PinState);


#ifdef __cplusplus
}
#endif

#endif /* __NEEY_H__ */

