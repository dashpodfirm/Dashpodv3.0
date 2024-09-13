// Copyright (c) Acconeer AB, 2022-2023
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>//
#include <stdarg.h>//

 #include "acc_definitions_common.h"
#include "acc_hal_definitions_a121.h"
#include "acc_hal_integration_nrf52833.h"
#include "acc_integration.h"
#include "acc_integration_log.h"

#include "app_timer.h"//
#include "nrf_gpio.h"//
#include "nrf_pwr_mgmt.h"//
#include "nrf_soc.h"//
//#include "nrf_gpiote.h"//
#include "nrfx_gpiote.h"//
#include "nrf_drv_spi.h"//

#include "pca10100.h"//
#include "IO_expa_buzzer.h"
#define MODULE "driver_hal"//

#define SENSOR_COUNT 3                           /**< @brief The number of sensors available on the board */
//#define STM32_SPI_MAX_TRANSFER_SIZE (65535)    /**< @brief The maximum SPI transfer size for the STM32 */
#define SPI_MAX_TRANSFER_SIZE 255

/**
 * @brief The reference frequency used by this board
 *
 * This assumes 26 MHz on the Sparkfun A121 Board
 * if BOARD_PCA10056 is defined, otherwise XM122.
 */
#if defined(BOARD_PCA10100)
// Assuming sparkfun breakout card
#define ACC_BOARD_REF_FREQ           24000000
#define SENSOR_DEFAULT_SPI_FREQUENCY NRF_SPIM_FREQ_1M
#else
// XM122
#define ACC_BOARD_REF_FREQ           24000000
#define SENSOR_DEFAULT_SPI_FREQUENCY NRF_SPIM_FREQ_32M
#endif

#define SPI_INSTANCE 0                                            /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool     spi_done;
static volatile bool     timed_out;

APP_TIMER_DEF(int_timeout);


static void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context)
{
	(void)p_event;
	(void)p_context;
	spi_done = true;
}

  
static void spi_init()
{
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

	spi_config.frequency      = SENSOR_DEFAULT_SPI_FREQUENCY;
	spi_config.ss_pin         = A121_SPI_CS_N_Pin;
	spi_config.miso_pin       = A121_SPI_MISO_Pin;
	spi_config.mosi_pin       = A121_SPI_MOSI_Pin;
	spi_config.sck_pin        = A121_SPI_CLK_Pin;
        APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
}


static void int_pin_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	//printf("hello\n");
}


void timeout_handler(void *p_context)
{
  //printf("Timeout handler\n");
  
	timed_out = true;
}

static void gpio_init_a121(void)
{
	if (!nrfx_gpiote_is_init())
	{
		APP_ERROR_CHECK(nrfx_gpiote_init());
	}

	nrfx_gpiote_in_config_t int_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
	int_config.pull = NRF_GPIO_PIN_PULLUP;
	APP_ERROR_CHECK(nrfx_gpiote_in_init(A121_SENSOR_INTERRUPT_Pin, &int_config, int_pin_handler));

     

	nrfx_gpiote_in_event_enable(A121_SENSOR_INTERRUPT_Pin, true);
        //nrf_gpio_cfg_output(A121_PS_ENABLE_Pin);
	nrf_gpio_cfg_output(A121_ENABLE_Pin);
}


static void timer_init(void)
{
	//app_timer_init();
        APP_ERROR_CHECK(app_timer_create(&int_timeout, APP_TIMER_MODE_SINGLE_SHOT, timeout_handler));
}


//----------------------------------------
// Implementation of RSS HAL handlers
//----------------------------------------


static void acc_hal_integration_sensor_transfer(acc_sensor_id_t sensor_id, uint8_t *buffer, size_t buffer_size)
{
	
        (void)sensor_id;
	spi_done = false;
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, buffer, buffer_size, buffer, buffer_size));

	while (!spi_done)
	{
		nrf_pwr_mgmt_run();
	}
}


void  acc_hal_integration_sensor_supply_on(acc_sensor_id_t sensor_id)
//void acc_hal_integration_sensor_power_on(acc_sensor_id_t sensor_id)
{
	//if ((sensor_id == 0) || (sensor_id > SENSOR_COUNT))
	//{
	//	Error_Handler();
	//}
	// There is no power supply control on the XE121
        //acc_hal_integration_nrf52833_init();
        set_IO_pin(2);
        
        
	//nrf_gpio_pin_set(A121_PS_ENABLE_Pin);
}


void acc_hal_integration_sensor_supply_off(acc_sensor_id_t sensor_id)
//void acc_hal_integration_sensor_power_off(acc_sensor_id_t sensor_id)
{
	//if ((sensor_id == 0) || (sensor_id > SENSOR_COUNT))
	//{
	//	Error_Handler();
	//}
        
	//nrf_gpio_pin_clear(A121_PS_ENABLE_Pin);
        clear_IO_pin(2);
}


void acc_hal_integration_sensor_enable(acc_sensor_id_t sensor_id)
 {
	(void)sensor_id; // Ignore parameter
        
        acc_hal_integration_nrf52833_init();
        nrfx_gpiote_init();

      
	nrf_gpio_pin_set(A121_ENABLE_Pin);
        
       
	acc_integration_sleep_us(2300);
	// Wait 2 ms to make sure that the sensor crystal has time to stabilize

	spi_init();

}


void acc_hal_integration_sensor_disable(acc_sensor_id_t sensor_id)
{

	(void)sensor_id; // Ignore parameter
        nrf_gpio_pin_clear(A121_ENABLE_Pin);

	nrf_drv_spi_uninit(&spi);
        nrf_gpio_cfg_default(A121_SPI_MOSI_Pin);
        nrf_gpio_cfg_default(A121_SPI_MISO_Pin);
        nrfx_gpiote_uninit();

	// Wait after power off to leave the sensor in a known state
	// in case the application intends to enable the sensor directly
	acc_integration_sleep_us(2300);
}


bool acc_hal_integration_wait_for_sensor_interrupt(acc_sensor_id_t sensor_id, uint32_t timeout_ms)
{
	if (timeout_ms > 0)
	{
		uint32_t ticks = APP_TIMER_TICKS(timeout_ms);
		timed_out = false;
		if (ticks < APP_TIMER_MIN_TIMEOUT_TICKS)
		{
			ticks = APP_TIMER_MIN_TIMEOUT_TICKS;
		}
                ret_code_t err;
		err=app_timer_start(int_timeout, ticks, NULL);
                APP_ERROR_CHECK(err);
	}
	else
	{
		timed_out = true;
	}

	while (!nrf_gpio_pin_read(A121_SENSOR_INTERRUPT_Pin) && !timed_out)
	{
		nrf_pwr_mgmt_run();
	}
	APP_ERROR_CHECK(app_timer_stop(int_timeout));

	return nrf_gpio_pin_read(A121_SENSOR_INTERRUPT_Pin);
}


uint16_t acc_hal_integration_sensor_count(void)
{
	return SENSOR_COUNT;
}

bool acc_hal_integration_nrf52833_init(void)
{
	gpio_init_a121();
	timer_init();

	return true;
} 

const acc_hal_a121_t *acc_hal_rss_integration_get_implementation(void)
//const acc_hal_a121_t *acc_hal_integration_get_implementation(void)
{
	static const acc_hal_a121_t val =
	{
		.max_spi_transfer_size = SPI_MAX_TRANSFER_SIZE,

		.mem_alloc = malloc,
		.mem_free  = free,

		.transfer = acc_hal_integration_sensor_transfer,
		.log      = acc_integration_log,

		.optimization.transfer16 = NULL,
	};

	return &val;
}
