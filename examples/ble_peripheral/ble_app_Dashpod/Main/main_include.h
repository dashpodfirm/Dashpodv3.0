#include "sdk_config.h"
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"

#include "app_util.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "boards.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_twi.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"
#include "nrf_twi_sensor.h"
#include "nrf_twi_mngr.h"
#include "nrf_drv_pwm.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#include "LSM6DSR.h"
#include "fuel_guage.h"
#include "fuel_gauge.h"
#include "nrf_drv_twi.h"
#include "IO_expa_buzzer.h"
#include "acc_hal_definitions_a121.h"
#include "acc_hal_integration_nrf52833.h"
//#include "acc_hal_integration_nrf52833.h"
#include "example_bring_up.h"
//#include "example_assembly_test.h"
#include "example_detector_distance.h"
#include "ref_app_wave_to_exit.h"
#include "led_ws2812b.h"
#include "bq2505_battery.h"
//#include "twi_init.h"

#define DEAD_BEEF                       0xDEADBEEF  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define inten 15


#define LED_PIN NRF_GPIO_PIN_MAP(1,3)
#define LED_WS2812B NRF_GPIO_PIN_MAP(0,4)

#define FUEL_IN NRF_GPIO_PIN_MAP(0,17)
#define ACCEL_IN1 NRF_GPIO_PIN_MAP(0,2)
#define ACCEL_IN2 NRF_GPIO_PIN_MAP(0,3)




void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name);

void timers_init(void);

void bsp_event_handler(bsp_event_t event);

void buttons_leds_init(bool * p_erase_bonds);

void log_init(void);

void power_management_init(void);

void idle_state_handle(void);

void ir_init();

bool hal_test_spi_read_chipid(void);

bool app_sensor_all_setting_set(uint8_t *p_in, uint16_t in_len);

int _write(int file,char *ptr, int len);

void in_fuel_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

void int_fuel_pin_enable(void);

void int_fuel_pin_disable(void);

void deep_sleep_timeout_handler1(void * p_context);

void deep_sleep_exit_init();

void wakeup_int_start();

void wakeup_int_stop();

void deep_sleep_timer_restart();

void deep_sleep_timer_stop();

void gpio_init(void);

void TW_Dashpod_init(void);

void TW_dashpod_start_loop(void);

void TW_dashpod_wait_loop(void);

void Charge_timer_start(void);

void Charge_timer_stop(void);

void Set_Mode(void);

int Get_Mode(void);

void charge_pin_interrupt(void);

void battery_low_level_indication(void);