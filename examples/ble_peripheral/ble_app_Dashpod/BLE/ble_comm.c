#include "ble_comm.h"
//#include "nrf_sdm.h"
#include "nrf_temp.h"

extern uint8_t seq_no;
extern uint8_t pod_color;
extern bool wait_loop;
extern volatile bool     tap_fun;
extern volatile bool     hit_cmd;
extern volatile bool     seq;
extern volatile bool     chg_led_lock;

extern volatile uint8_t  loop;

//extern volatile uint8_t Mode;
volatile bool     charge_lock = false;
volatile bool     ble_a111_status = false;

volatile uint8_t pod_action;
volatile uint8_t mod;
volatile uint8_t hit;

uint8_t base_code = 1;
uint8_t brightness = 4;
uint8_t bat_cnt = 0;
//extern bool ble_a111_status;
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event);


void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_DEBUG("Connected");
            printf("Connected\n");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            ble_a111_status   = true;
            wait_loop = false;
            seq_no =3 ;
            LED_LS(true);
            nrf_delay_ms(6);
            rgb_led_glow(0,255,0);
            rgb_led_glow(0,255,0);
            rgb_led_glow(0,0,0);
            rgb_led_glow(0,255,0);
            rgb_led_glow(0,255,0);
            rgb_led_glow(0,0,0);
            LED_LS(false);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_DEBUG("Disconnected");
            printf("Disonnected\n");
            ble_a111_status   = false;
            seq = true;
            //Mode = 4;
            Set_Mode();
            LED_LS(true);
            nrf_delay_ms(6);
            rgb_led_glow(255,0,0);
            rgb_led_glow(255,0,0);
            rgb_led_glow(0,0,0);
            rgb_led_glow(255,0,0);
            rgb_led_glow(255,0,0);
            chg_led_lock = false;
            delete_bonds();
            LED_LS(false);
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            in_fuel_pin_handler(0,0);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
/*
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;
*/
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}

void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            // YOUR_JOB: Get ready to reset into DFU mode
            //
            // If you aren't finished with any ongoing tasks, return "false" to
            // signal to the system that reset is impossible at this stage.
            //
            // Here is an example using a variable to delay resetting the device.
            //
            // if (!m_ready_for_reset)
            // {
            //      return false;
            // }
            // else
            //{
            //
            //    // Device ready to enter
            //    uint32_t err_code;
            //    err_code = sd_softdevice_disable();
            //    APP_ERROR_CHECK(err_code);
            //    err_code = app_timer_stop_all();
            //    APP_ERROR_CHECK(err_code);
            //}
            break;

        default:
            // YOUR_JOB: Implement any of the other events available from the power management module:
            //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
            //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
            //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

//lint -esym(528, m_app_shutdown_handler)
/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);


static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context)
{
    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        //Go to system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
{
    .handler = buttonless_dfu_sdh_state_observer,
};

static void advertising_config_get(ble_adv_modes_config_t * p_config)
{
    memset(p_config, 0, sizeof(ble_adv_modes_config_t));

    p_config->ble_adv_fast_enabled  = true;
    p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
    p_config->ble_adv_fast_timeout  = APP_ADV_DURATION;
}

static void disconnect(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    }
    else
    {
        NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
    }
}

// YOUR_JOB: Update this code if you want to do anything given a DFU event (optional).
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]        event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
        {
            NRF_LOG_DEBUG("Device is preparing to enter bootloader mode.");
            printf("Device is preparing to enter bootloader mode.\n");
            // Prevent device from advertising on disconnect.
            ble_adv_modes_config_t config;
            advertising_config_get(&config);
            config.ble_adv_on_disconnect_disabled = true;
            ble_advertising_modes_config_set(&m_advertising, &config);

            // Disconnect all other bonded devices that currently are connected.
            // This is required to receive a service changed indication
            // on bootup after a successful (or aborted) Device Firmware Update.
            uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
            NRF_LOG_DEBUG("Disconnected %d links.", conn_count);
            printf("Disconnected %d links.\n", conn_count);
            break;
        }

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            NRF_LOG_DEBUG("Device will enter bootloader mode.");
            printf("Device will enter bootloader mode.\n");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            printf("Request to enter bootloader mode failed asynchroneously.\n");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            printf("Request to send a response to client failed.\n");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            printf("Unknown event from ble_dfu_buttonless.\n");
            break;
    }
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);
}

/**@brief Function for the Peer Manager initialization.
 */
void peer_manager_init()
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/** @brief Clear bonding information from persistent storage.
 */
void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_DEBUG("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}

void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_DEBUG("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the command from the Nordic UART Service.
 *
 * @details This function will process the command received from the Nordic UART BLE Service.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the command received over Dashpod BLE app] */

void app_dashpod_command_handler(ble_nus_evt_t * p_evt)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN+1];
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        printf("Received data from BLE NUS. Using in app_dashpod_command_handler.\n");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
        printf("[");
        for (uint8_t i = 0; i < p_evt->params.rx_data.length; i++)
        {   
          printf("0x%X, ",data_array[i] = p_evt->params.rx_data.p_data[i]);
        }
        printf("\b\b]\n");

        /*Data handler Starting*/
        
        if(p_evt->params.rx_data.p_data[0]== 0x01)
        {
          chg_led_lock = true;
          charge_lock = true;
          printf("\nEnable command received\n");

          mod = data_array[1];

          brightness = data_array[4];
          //app_timer_pause();
          Charge_timer_stop();

          app_sensor_all_setting_set(data_array+1,6);

          memcpy((void*)data_array, &p_evt->params.rx_data.p_data[1], 6);

        }
        else if(p_evt->params.rx_data.p_data[0]== 0x02)
        {
           
            if(charge_lock)
            {
                  printf("\nHit command\n");
                  hit_cmd = true;
                  hit = 1;
                  printf("[");
                  for (uint8_t i = 0; i < 5; i++)
                  {   
                    printf("0x%X, ",data_array[i]);
                  }
                  printf("\b\b]\n");
            
                  set_color_ws2812b(data_array[1], brightness);

                  pod_action = p_evt->params.rx_data.p_data[4];

                  seq = false;
                  pod_color = data_array[1];

                  if(pod_action == 0 || pod_action == 2 || mod != 1)
                  {
                        LED_LS(true);
                        led_ws2812b_show();
                  }
            }
            else
            {
                  printf("\nGive Enable command first\n");
            }
        }
        else if(p_evt->params.rx_data.p_data[0]== 0x03)
        {
            //app_timer_resume();
            Charge_timer_start();
            chg_led_lock = false;
            charge_lock = false;
            //Mode = 4;
            Set_Mode();
            LED_LS(false);
            printf("\nDisable command\n");
            printf("[");
            for (uint8_t i = 0; i < 3; i++)
            {   
              printf("0x%X, ",data_array[i]);
            }
            printf("\b\b]\n");
            in_fuel_pin_handler(0,0);
            loop = 0;
            seq = true;
            ble_a111_status = false;
            tap_fun = false;
        }
        else if(p_evt->params.rx_data.p_data[0]== 0x04)
        {
          //app_timer_pause();
          Charge_timer_stop();
          
          chg_led_lock = true;
          charge_lock = false;
          printf("\nGroup distribution Enable command\n");
          printf("[");
          for (uint8_t i = 0; i < 3; i++)
          {   
            printf("0x%X, ",data_array[i]);
          }
          printf("\b\b]\n");
          LED_LS(true);
          set_color_ws2812b(data_array[2],brightness);
          if(data_array[1] == 2)
          {
            buzzer_tone1();
          }
          else
          {
            led_ws2812b_show();            
          }
    
        }
        else if((p_evt->params.rx_data.p_data[0]== 0x05))
        {
          //app_timer_resume();
          Charge_timer_start();
          chg_led_lock = false;
          charge_lock = false;
          printf("\nGroup distribution disable command\n");
          printf("[");
          for (uint8_t i = 0; i < 3; i++)
          {   
            printf("0x%X, ",data_array[i]);
          }
          printf("\b\b]\n");
          LED_LS(false);
          in_fuel_pin_handler(0,0);
        }
        else if((p_evt->params.rx_data.p_data[0]== 0x06))
        {
          //app_timer_resume();
          printf("\nMiss Command\n");
          printf("[");
          for (uint8_t i = 0; i < 5; i++)
          {   
            printf("0x%X, ",data_array[i]);
          }
          printf("\b\b]\n");
          Charge_timer_start();
          chg_led_lock = false;
          charge_lock = false;
          led_ws2812b_reset();
          LED_LS(false);
        }
        else if((p_evt->params.rx_data.p_data[0]== 0x07))
        {
                NVIC_SystemReset();
        }
        else if(p_evt->params.rx_data.p_data[0] == 0x00)
        {
          FW_version_send_ble();
        }
        else
        {
            length=12;
            printf("____Invalid command_____\n");        
        }

        /*Data handler Ending*/
        printf("\nDashpod app command handler is over\n");

    }
}

/*Response commands on wave or hit function*/
void FW_version_send_ble(void)
{
  length = 3;
  uint8_t op_snd[100];
  sprintf(op_snd,"dashpod_fw_v%.3f_%s",VERSION,RELEASE_DATE);
  length = strlen(op_snd);

  for (uint8_t i = 0; i < length; i++)
  {   
    printf("%c",op_snd[i]);
  }
  printf("\n");
  ble_nus_data_send(&m_nus, op_snd, &length, m_conn_handle);
}

void A111_tap_send_ble(float *data)
{
  length = 3;
  uint8_t op_snd[4] = {pod_color,0x01,base_code};
  uint8_t json_dat[100];
  sprintf(json_dat, "{\"X\":%.2f, \"Y\":%.2f, \"Z\":%.2f}",data[0], data[1], data[2]);
  printf("%s\n",json_dat);
  printf("[");
  for (uint8_t i = 0; i < length; i++)
  {   
    printf("0x%X, ",op_snd[i]);
  }
  printf("\b\b]\n");
  ble_nus_data_send(&m_nus, op_snd, &length, m_conn_handle);
  uint16_t str_len = strlen(json_dat);
  ble_nus_data_send(&m_nus, json_dat, &str_len, m_conn_handle);
}

void A111_wave_send_ble()
{
  length = 3;
  uint8_t op_snd[4] = {pod_color,0x01,base_code};
  uint8_t json_dat[100];
  printf("[");
  for (uint8_t i = 0; i < length; i++)
  {   
    printf("0x%X, ",op_snd[i]);
  }
  printf("\b\b]\n");
  ble_nus_data_send(&m_nus, op_snd, &length, m_conn_handle);
}

void A111_tap_fail_send_ble(void)
{
  length = 3;
  uint8_t op_snd[4]={pod_color,0x00,base_code};
  printf("[");
  for (uint8_t i = 0; i < 4; i++)
  {   
    printf("0x%X, ",op_snd[i]);
  }
  printf("\b\b]\n");
  ble_nus_data_send(&m_nus, op_snd, &length, m_conn_handle);
}

void on_bas_evt(ble_bas_t * p_bas, ble_bas_evt_t * p_evt)
{
    NRF_LOG_DEBUG("p_evt->evt_type: %d", p_evt->evt_type);
    printf("p_evt->evt_type: %d\n", p_evt->evt_type);
    switch (p_evt->evt_type)
    {
        case BLE_BAS_EVT_NOTIFICATION_ENABLED:
	{
          printf("BLE_BAS_EVT_NOTIFICATION_ENABLED\n");
          #ifdef DEBUG
          float val = read_batt_full_cap();
          printf("\tFull cap = %f\n",val);
          val = 100/val;
          float rem = read_batt_cap();
          
          printf("\tRem cap = %f\n",rem);
          uint8_t charge = rem * val;
          printf("percentage request = %d\n", charge);
          #endif
          ble_bas_battery_level_update(&m_bas, read_batt_per(),m_conn_handle);
        }break; // BLE_BAS_EVT_NOTIFICATION_ENABLED

        case BLE_BAS_EVT_NOTIFICATION_DISABLED:
	{
          printf("BLE_BAS_EVT_NOTIFICATION_DISABLED\n");
	}break; // BLE_BAS_EVT_NOTIFICATION_DISABLED
        default:
            // No implementation needed.
        break;
    }
}

void A111_cyc_count_send_ble(int count)
{
  char str[5];
  length=15;
  ble_nus_data_send(&m_nus,"\nBattery per = ", &length, m_conn_handle);
  sprintf(str,"%d",count);
  strcat(str,"\0");
  length=strlen(str);
  ble_nus_data_send(&m_nus,str ,&length, m_conn_handle);
}

void A111_trigger_send_ble(void)
{
  length=16;
  ble_nus_data_send(&m_nus,"\nWave_triggered\n", &length, m_conn_handle);
}

void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

void nus_init(void)
{
  uint32_t          err_code;
     // Initialize NUS.
  ble_nus_init_t     nus_init;
  memset(&nus_init, 0, sizeof(nus_init));

  nus_init.data_handler = app_dashpod_command_handler;

  err_code = ble_nus_init(&m_nus, &nus_init);
  APP_ERROR_CHECK(err_code);   
}

/**@brief Function for initializing the Battery Service.
 */
void bas_init(void)
{
    ret_code_t     err_code;
    ble_bas_init_t bas_init_obj;

    memset(&bas_init_obj, 0, sizeof(bas_init_obj));

    bas_init_obj.evt_handler          = on_bas_evt;
    bas_init_obj.support_notification = true;
    bas_init_obj.p_report_ref         = NULL;
    bas_init_obj.initial_batt_level   = 0;

    bas_init_obj.bl_rd_sec        = SEC_OPEN;
    bas_init_obj.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init_obj.bl_report_rd_sec = SEC_OPEN;
    err_code = ble_bas_init(&m_bas, &bas_init_obj);
    APP_ERROR_CHECK(err_code);
}

void  services_init(void)
{
    uint32_t           err_code;
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_dfu_buttonless_init_t dfus_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    dfus_init.evt_handler = ble_dfu_evt_handler;

    
    nus_init();
    bas_init();
    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);
}

void sleep_mode_enter(void)
{
    uint32_t err_code ;

 //Disable SoftDevice. It is required to be able to write to GPREGRET2 register (SoftDevice API blocks it).
  //GPREGRET2 register holds the information about skipping CRC check on next boot.
  err_code = nrf_sdh_disable_request();
  APP_ERROR_CHECK(err_code);
}

void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}

void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.x
    }
    else
    {
        uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);

        printf("advertising is started\n");

        NRF_LOG_DEBUG("advertising is started");
    }
}

void advertising_restart()
{
    //sd_ble_gap_adv_start(m_advertising.adv_handle, APP_BLE_CONN_CFG_TAG);
    //sd_softdevice_enable(NRF_CLOCK_LFCLK_Xtal, NULL);
    ble_a111_status = false;
    
    printf("advertising is restarted\n");

    NRF_LOG_DEBUG("advertising is restarted");
    APP_ERROR_CHECK(sd_ble_gap_adv_start(m_advertising.adv_handle, m_advertising.conn_cfg_tag));
    //NVIC_SystemReset();

    //printf("advertising is restarted\n");

    //NRF_LOG_DEBUG("advertising is restarted");
}

void advertising_stop()
{
        //advertising_start(true);
        if(ble_a111_status == true)
        {
          //APP_ERROR_CHECK(sd_ble_gap_disconnect(m_conn_handle,BLE_HCI_CONN_INTERVAL_UNACCEPTABLE));
          //APP_ERROR_CHECK(sd_ble_gap_adv_stop(m_advertising.adv_handle));
          
          //APP_ERROR_CHECK(sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_DEV_TERMINATION_DUE_TO_LOW_RESOURCES));
          disconnect(m_conn_handle,NULL);
          //while(ble_a111_status)
          //{
          // idle_state_handle(); 
          //}

          //APP_ERROR_CHECK(sd_ble_gap_adv_stop(m_advertising.adv_handle));
        }
        else
        {
          APP_ERROR_CHECK(sd_ble_gap_adv_stop(m_advertising.adv_handle));
        }
        //APP_ERROR_CHECK(sd_ble_gap_adv_stop(m_advertising.adv_handle));
        
        //sd_softdevice_disable();

        printf("advertising is stoped\n");
        
        NRF_LOG_DEBUG("advertising is stoped");
    
}