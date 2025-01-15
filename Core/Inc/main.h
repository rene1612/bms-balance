/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <dev_config.h>

#ifndef __BOARD_TYPE__
	#define __BOARD_TYPE__				BMS_BALANCE_BOARD
#endif

//
#define __BRD_ID__						0x01

#ifndef __DEV_ID__
	#define __DEV_ID__					(__BOARD_TYPE__ + __BRD_ID__)
#endif


#ifndef __BOARD_VERSION__
	#define __BOARD_VERSION__			(0x0100)
//	#define __BOARD_VERSION__			(0x0201)
#endif

#define __BOARD_NAME__ 					"BMS_BALANCE_BOARD"



#define __ALLERT_DEBUG__

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint8_t main_task_scheduler;
extern uint8_t alive_timer;
/* USER CODE END ET */

/* USER CODE BEGIN Private defines */

#define PROCESS_NO_TASK			0x00
#define PROCESS_CAN				0x02
#define PROCESS_10_MS_TASK		0x04
#define PROCESS_100_MS_TASK		0x08
#define PROCESS_1000_MS_TASK	0x10
#define PROCESS_STATUS			0x20
#define PROCESS_BLK_BALANCER	0x40

#define ALIVE_TIMEOUT_10MS		15
#define APP_CAN_BITRATE			500000UL

#define __DEV_SIGNATURE__		0x41
#define __SW_RELEASE__			0x0100
#define SW_RELEASE_DAY			14
#define SW_RELEASE_MONTH		01
#define SW_RELEASE_YEAR			2025
#define __SW_RELEASE_DATE__		((SW_RELEASE_DAY<<24 ) | (SW_RELEASE_MONTH<<18) | SW_RELEASE_YEAR)
#define __SW_NAME__				"BMS-BALANCE-APP"



 /**
  * Bit-Defines für das Controllregister
  */
#define REG_CTRL_ACTIVATE			0
#define REG_CTRL_DEACTIVATE			1
#define REG_CTRL_ENABLE_BB			2
#define REG_CTRL_CRIT_ALERT			6
#define REG_CTRL_RESET				7	//!<Reset des Controllers auslösen



#define REG_ALERT_HEAT_SINK_TEMP	0
#define REG_ALERT_BLK_VOLTAGE		6
#define REG_ALERT_BLK_DIFF_VOLTAGE	7



 /**
  * Zustände
  */
typedef enum
{
	STATE_OFF	=				0x00,	//!<Keine Blinken (LED aus)
	STATE_OK	=				0x4F,	//!<Zustand alles OK (gleichmäßiges "langsames" Blinken Tastverhältnis 50/50)
	STATE_WARN	=				0xCC,	//!<Zustand Warnung (gleichmäßiges "schnelles" Blinken Tastverhältnis 50/50)
	STATE_ERR_UNKNOWN=			0x1F,	//!<Zustand unbekannter Fehler (gleichmäßiges "sehr schnelles" Blinken Tastverhältnis 50/50)
	STATE_ERR_HEADSINK_TEMP=	0x02,	//!<Zustand Fehler Kühlkörper-Temperatur zu hoch (einmal kurzes blinken)
}_LED_STATE;


typedef enum
{
	ERR_NONE	=				0x00,	//!<Keine Blinken (LED aus)
	ERR_UNKNOWN=				0x8F,	//!<Zustand unbekannter Fehler (gleichmäßiges "sehr schnelles" Blinken Tastverhältnis 50/50)
	ERR_HEADSINK_TEMP=			0x02,	//!<Zustand Fehler Kühlkörper-Temperatur zu hoch (einmal kurzes blinken)
	ERR_ALIVE=					0x12,
	ERR_BLK_VOLTAGE=			0x42,	//!<Zustand Fehler  (einmal kurzes blinken)
	ERR_BLK_VOLTAGE_DIFF=		0x43,	//!<Zustand Fehler  (einmal kurzes blinken)
}_SYS_ERR_CODES;


#define MAX_LF280K_CELL_COUNT	22

/* USER CODE END Private defines */

//typedef enum
//{
//	ACK = 0x11,
//	NACK= 0x13
//}_REPLAY_TYPE;


typedef enum
{
	SYS_OK,
	SYS_ACTIVE_START,
	SYS_ACTIVE_FC,
	SYS_ACTIVE_SBC,
	SYS_ACTIVE_STOP,
	SYS_ERROR,
}_SYS_STATE;


#pragma pack(push,1)

typedef struct
{
	uint16_t		min;
	uint16_t		max;
	uint8_t			enable_mask;
}_MIN_MAX;


typedef struct
{
	_MIN_MAX		blk_voltage;
	_MIN_MAX		blk_voltage_diff;
	_MIN_MAX		heatsink_temperature;
}_ALERT_THRESHOLDS;


 /**
 * @struct	REG
 * @brief	Registersatz des Controllers.
 *
 * @note	Der Registersatz wird im RAM und im EEProm gehalten
 */
 typedef struct
 {
	uint8_t						alert_mask;
	uint8_t						crit_alert_mask;
	 _ALERT_THRESHOLDS			alert_thresholds;
 }_BMS_BALANCE_CONFIG_REGS;

#pragma pack(pop)


#define ENABLE_MAX_THRESHOLD	0x01
#define ENABLE_MIN_THRESHOLD	0x02


#define NO_LED					0x00
#define GREEN_LED				0x01
#define RED_LED					0x02
#define BLUE_LED				0x04
#define ALL_LED					(GREEN_LED+RED_LED+BLUE_LED)


 typedef enum
 {
 	OFF=				0x0000,
 	SLOW_FLASH=			0x00FF,
 	FAST_FLASH=			0x0F0F,
 	FVERY_FAST_FLASH=	0x3333,
 	HYPER_FAST_FLASH=	0x5555,
 	LED_100MS_FLASH=	0x0001,
 	LED_200MS_FLASH=	0x0003,
 	LED_300MS_FLASH=	0x0007,
 	LED_1_FLASH=		0x0001,
	LED_2_FLASH=		0x0005,
	LED_3_FLASH=		0x0015,
	LED_4_FLASH=		0x0055,
	LED_5_FLASH=		0x0155,
	ON=					0xFFFF,
 }_LED_SIGNAL_MASK;


 /**
  * @struct	REG
  * @brief	Registersatz des Controllers.
  *
  * @note	Der Registersatz wird im RAM und im EEProm gehalten
  */
  typedef struct
  {
 	uint16_t				mask;
 	_LED_SIGNAL_MASK		green_led_mask;
 	_LED_SIGNAL_MASK		red_led_mask;
 	_LED_SIGNAL_MASK		blue_led_mask;
  }_LED_SIGNAL_STATE;


/**
 * @struct	REG
 * @brief	Registersatz des Controllers.
 *
 * @note	Der Registersatz wird im RAM und im EEProm gehalten
 */
 typedef struct
 {
	uint8_t						ctrl;

	_SYS_STATE					sys_state;
	_SYS_ERR_CODES				sys_err;
	_LED_STATE					monitor_led_state;

	uint8_t						alive_timeout;

	uint32_t					can_rx_cmd_id;
	uint32_t					can_tx_data_id;
	uint32_t					can_tx_heartbeat_id;
	uint32_t					can_rx_brdc_cmd_id;
	uint32_t 					can_filterMask;
	uint32_t 					can_filterID; // Only accept bootloader CAN message ID

	_BMS_BALANCE_CONFIG_REGS	cfg_regs;
	_DEV_CONFIG_REGS			dev_config;	//copy of dev-config
	_SW_INFO_REGS				sw_info;	//copy of sw_info
	_BOARD_INFO_STRUCT			board_info;	//copy of board_info
 }_MAIN_REGS;


#pragma pack(push,1)

typedef struct
{
	uint8_t			bms_data_type;
	uint8_t			flags_ch_number;
	uint16_t		ave_voltage;
	uint16_t		div_voltage;
}_BMS_BLK_DATA2;

typedef struct
{
	uint8_t			bms_data_type;
	uint8_t			flags_ch_number;
	uint16_t		bal_current;
	int16_t			heat_sink_temperatur;
}_BMS_BLK_DATA3;

#pragma pack(pop)



 /**
  * @def		REG_ADDR_AUTO_INC
  * @brief	Flag im Registeradressbyte, welches eine automatische Incrementierung
  *			der Registeradresse nach einer Lese- oder Schreibaktion bewirkt.
  *
  * @note	Die Registeradresse wird vom Master bei jeder Schreibaktion als erstes Byte gesendet.
  * @see		fco_reg_addr
  * @see		TWI-WRITE-Mode
  */
  #define REG_ADDR_AUTO_INC	0x80

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Funktionen(Prototypes)
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void set_sys_state (_SYS_STATE sys_state);
void JumpToBtld(void);
void JumpToApp(void);

/* Private typedef -----------------------------------------------------------*/
typedef void (*pFunction)(void);


extern  _MAIN_REGS main_regs;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* Funktionen(Prototypes) --------------------------------------------------------*/
void set_sys_state (_SYS_STATE sys_state);
void DoAlert(uint8_t* p_msg, uint8_t len);
void set_signal_led(uint8_t led, _LED_SIGNAL_MASK mask);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_DATA_STROBE_Pin		GPIO_PIN_0
#define SPI1_DATA_STROBE_GPIO_Port	GPIOB
#define SPI1_OE_Pin					GPIO_PIN_1
#define SPI1_OE_GPIO_Port			GPIOB
#define LED_GREEN_Pin				GPIO_PIN_12
#define LED_GREEN_GPIO_Port			GPIOB
#define LED_RED_Pin					GPIO_PIN_13
#define LED_RED_GPIO_Port			GPIOB
#define LED_BLUE_Pin				GPIO_PIN_14
#define LED_BLUE_GPIO_Port			GPIOB

/* USER CODE BEGIN MYPD */
#undef LED_GREEN_Pin
#undef LED_RED_Pin
#undef LED_BLUE_Pin
#define LED_GREEN_Pin GPIO_PIN_14
#define LED_RED_Pin GPIO_PIN_12
#define LED_BLUE_Pin GPIO_PIN_13


/* USER CODE END MYPD */


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
