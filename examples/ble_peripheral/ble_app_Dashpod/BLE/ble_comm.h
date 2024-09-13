#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"

#include "ble_nus.h"
#include "ble_bas.h"
#include "ble_dfu.h"
#include "nrf_power.h"
#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"
#include "nrf_bootloader_info.h"

#include "bsp_btn_ble.h"
#include "nrf_log.h"
#include "nrf_drv_pwm.h"

#include "LSM6DSR.h"
#include "fuel_guage.h"
#include "nrf_gpio.h"
#include "IO_expa_buzzer.h"
#include "led_ws2812b.h"
#include "main_include.h"

#include "peer_manager.h"
#include "peer_manager_handler.h"

#include "app_timer.h"


#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define DEVICE_NAME                     "dashpod-v0.1"                               /**< Name of device. Will be included in the advertising data. */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                2460                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_DURATION                0                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */


#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define VERSION                         3.02
#define RELEASE_DATE                    "31-08-2024"

static ble_uuid_t m_adv_uuids[]  =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE}
};


static uint16_t length;
static uint8_t x,y,z;

extern  volatile bool ble_a111_status;
extern  volatile bool hit_fun;
extern volatile bool seq;
extern volatile int comp_score;

BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */


BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
BLE_BAS_DEF(m_bas);                                                                 /**< Battery service instance. */

static nrf_pwm_values_common_t m_seq_values1[2];
static nrf_drv_pwm_t m_pwm1 = NRF_DRV_PWM_INSTANCE(1);
static nrf_pwm_sequence_t const m_seq1 =
{
    .values.p_common     = m_seq_values1,
    .length              = NRF_PWM_VALUES_LENGTH(m_seq_values1),
    .repeats             = 0,
    .end_delay           = 0
};

static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3; 
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/




void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);

void ble_stack_init(void);

void pm_evt_handler(pm_evt_t const * p_evt);

void peer_manager_init();

void delete_bonds(void);

void gap_params_init(void);

void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt);


/**@brief Function for initializing the GATT library. */
void gatt_init(void);

void nus_data_handler(ble_nus_evt_t * p_evt);

void FW_version_send_ble(void);

void A111_cyc_count_send_ble(int count);

void A111_tap_send_ble(float *data);

void A111_wave_send_ble();

void A111_tap_fail_send_ble(void);

void A111_trigger_send_ble(void);

void nrf_qwr_error_handler(uint32_t nrf_error);

void services_init(void);

void sleep_mode_enter(void);

void on_adv_evt(ble_adv_evt_t ble_adv_evt);

void advertising_init(void);

void on_conn_params_evt(ble_conn_params_evt_t * p_evt);

void conn_params_error_handler(uint32_t nrf_error);

void conn_params_init(void);

void advertising_start(bool erase_bonds);

void advertising_restart();

void advertising_stop();

