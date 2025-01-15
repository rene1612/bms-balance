/**
  ******************************************************************************
  * @file    blk_balance.c
  * @brief   This file provides code for the ctrl an rading of the passive-balancer
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 René Schoenrock.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "blk_balancer.h"
#include "string.h"
#include "main.h"
#include "spi.h"
#include "gpio.h"



_BB_CTRL_STRUCT bb_ctrl;


void BB_OutputEnable(GPIO_PinState PinState) {
	HAL_GPIO_WritePin(SPI1_OE_GPIO_Port, SPI1_OE_Pin, !PinState);
}

void BlkBalancer_init(void)
{
	HAL_StatusTypeDef result=HAL_OK;
	uint8_t spi_buffer[3]={0x00,0x00,0x00};

	BB_OutputEnable(0);


	bb_ctrl.time_index = 0;
	bb_ctrl.ch_ebable_mask = 0U;

	//send {0x00, 0x00, 0x00} to passive balancer (shut off everything)
	result = HAL_SPI_Transmit_IT(&hspi1, spi_buffer, 3);

	HAL_GPIO_WritePin(SPI1_DATA_STROBE_GPIO_Port, SPI1_DATA_STROBE_Pin, GPIO_PIN_SET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(SPI1_DATA_STROBE_GPIO_Port, SPI1_DATA_STROBE_Pin, GPIO_PIN_RESET);

	if (result == HAL_OK) {
		bb_ctrl.last_spi_buf[0] = 0x00;
		bb_ctrl.last_spi_buf[1] = 0x00;
		bb_ctrl.last_spi_buf[2] = 0x00;
	}
}


void BlkBalancer_set(uint8_t channel, uint8_t value, uint8_t direction)
{

	assert(channel < BB_MAX_CHANNEL);

	bb_ctrl.ch_val[channel] = value;

	if(value) {
		bb_ctrl.ch_ebable_mask |= 1<<channel;
	}
	else {
		bb_ctrl.ch_ebable_mask &= ~(1<<channel);
	}

	if(direction) {
		bb_ctrl.ch_dir_mask |= 1<<channel;
	}
	else {
		bb_ctrl.ch_dir_mask &= ~(1<<channel);
	}

}



uint8_t	process_BlkBalancer(void)
{

	//uint8_t ch_mask=0;

	if (bb_ctrl.ch_ebable_mask) {

		uint8_t ch;
		uint8_t spi_buffer[1]={0x00};
		HAL_StatusTypeDef result=HAL_OK;

		bb_ctrl.time_index++;

		for (ch=0; ch<PB_MAX_CHANNEL; ch++) {

			if (bb_ctrl.ch_ebable_mask & 1<<ch) {

				if (bb_ctrl.ch_val[ch]==0) {
					bb_ctrl.ch_ebable_mask &= ~(1<<ch);
				}else {

					if (!(bb_ctrl.time_index >= bb_ctrl.ch_val[ch])) {
						//set
						//ch_mask |= (1<<(ch%8));
						spi_buffer[(ch/8)] |= (1<<(ch%8));
					}
				}
			}
		}


		if (spi_buffer[0] != bb_ctrl.last_spi_buf[0] ) {

			//send {new data} to passive balancer (shut on or off everything)
			result = HAL_SPI_Transmit_IT(&hspi1, spi_buffer, 1);

			HAL_GPIO_WritePin(SPI1_DATA_STROBE_GPIO_Port, SPI1_DATA_STROBE_Pin, GPIO_PIN_SET);
			HAL_Delay(1);
			HAL_GPIO_WritePin(SPI1_DATA_STROBE_GPIO_Port, SPI1_DATA_STROBE_Pin, GPIO_PIN_RESET);

			if (result == HAL_OK) {
				bb_ctrl.last_spi_buf[0] = spi_buffer[0];
			}

		}

	}

	return 0;
}
