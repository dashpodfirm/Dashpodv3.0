#include "main_include.h"
#include "ble_comm.h"
#include "nrf_temp.h"
#include <nrf_log.h>
#include <ble_dfu.h>
#include <app_error.h>
#include <nrf_bootloader_info.h>
#include "twi_init.h"


//extern volatile bool     ble_a111_status;
extern volatile bool     tap_fun;
extern volatile bool     hit_cmd;
extern volatile bool     seq;
extern volatile uint8_t  loop;
extern volatile bool     first;
volatile bool   chrg_in_en = false;
volatile bool   chrg_in_dis = false;


uint8_t pod_color;
const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

uint8_t sample_data[2];

volatile uint8_t Mode = 4; 
volatile uint32_t charge = 0, chrg = 0;
volatile uint16_t chg_cnt = 1;
volatile uint8_t seq_no = 0;
volatile bool chg_led_lock = true;
volatile bool wait_loop = true;
volatile bool  chgl = true ;
volatile bool charge_conn = true;

#define BATT_MEAS_TIME APP_TIMER_TICKS(30000)
#define CHRG_CHK_TIME APP_TIMER_TICKS(2000)

APP_TIMER_DEF(charg_cycle);

//APP_TIMER_DEF(batt_tim_id);
//APP_TIMER_DEF(chrg_chk_tim_id);
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
     __disable_irq();
    NRF_LOG_FINAL_FLUSH();

    printf("Fatal error\n");
    switch (id)
    {
//#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
//        case NRF_FAULT_ID_SD_ASSERT:
//            NRF_LOG_ERROR("SOFTDEVICE: ASSERTION FAILED");
//            break;
//        case NRF_FAULT_ID_APP_MEMACC:
//            NRF_LOG_ERROR("SOFTDEVICE: INVALID MEMORY ACCESS");
//            break;
//#endif
        case NRF_FAULT_ID_SDK_ASSERT:
        {
            assert_info_t * p_info = (assert_info_t *)info;
            printf("ASSERTION FAILED at %s:%u\n",
                          p_info->p_file_name,
                          p_info->line_num);
            break;
        }
        case NRF_FAULT_ID_SDK_ERROR:
        {
            error_info_t * p_info = (error_info_t *)info;
            printf("ERROR %u [%s] at %s:%u\r\nPC at: 0x%08x\n",
                          p_info->err_code,
                          nrf_strerror_get(p_info->err_code),
                          p_info->p_file_name,
                          p_info->line_num,
                          pc);
             printf("End of error report\n");
            break;
        }
        default:
            printf("UNKNOWN FAULT at 0x%08X\n", pc);
            break;
    }

   // NRF_BREAKPOINT_COND;
    // On assert, the system can only recover with a reset.

#ifndef DEBUG
    printf("System reset\n");
    twi_uninit();
    app_timer_stop_all();
    nrfx_gpiote_uninit();
    sd_nvic_SystemReset();
    NVIC_SystemReset();
#else
    app_error_save_and_stop(id, pc, info);
#endif // DEBUG
}


void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}



/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
void idle_state_handle(void)
{
    #ifdef DEBUG
    printf("In sleep mode\n");
    #endif
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


bool hal_test_spi_read_chipid(void)
{
      const uint32_t sensor = 1;
      const acc_hal_a121_t *hal = acc_hal_rss_integration_get_implementation() ;
      uint8_t buffer[6] = {0x30,0x0,0x0,0x0,0x0,0x0};
      acc_hal_integration_sensor_supply_on(sensor);
      acc_hal_integration_sensor_enable(sensor);
      hal->transfer(sensor,buffer,sizeof(buffer));
      acc_hal_integration_sensor_disable(sensor);
      acc_hal_integration_sensor_supply_off(sensor);
      for(int i=0;i<6;i++)
      {
          printf("0x%X\t",buffer[i]);
      }
      printf("\n");
      if (buffer[4] == 0x12 && buffer[5] == 0x10 )
      {
          printf( " Test OK !\n ");
          return true ;
      }
      printf( " Cannot read chip id !\n " );
      return false ;
}

bool app_sensor_all_setting_set(uint8_t *p_in, uint16_t in_len)
{

        bool status=true;
        uint8_t A111_accel_both = p_in[0];
        #ifdef DEBUG
        printf("In app sensor setting function\n");
        #endif
        printf("[");
        for (uint8_t i = 0; i < in_len; i++)
        {   
          printf("0x%X, ",p_in[i]);
        }
        printf("\b\b]\n");
  
        pod_color = p_in[2];
        //setup_accel_sensitivity(&m_twi,0x6b,p_in[1]);
        set_color_ws2812b(p_in[2],p_in[3]);
        LED_LS(true);
        if(A111_accel_both == 0x01)
        {
          printf("Wave function\n");
          Mode = 3;
          //tap_fun = false;
          //ble_a111_status = true;
        }
        else if (A111_accel_both == 0x02)
        {
          //setup_gyro_accel(&m_twi,0x6b);
          setup_accel_sensitivity(p_in[1]);
          //ble_a111_status = true;
          //tap_fun = true;
          Mode = 1;
          printf("Hit function\n");
        }

        else if (A111_accel_both == 0x03)
        {
          //setup_gyro_accel(&m_twi,0x6b);
          setup_accel_sensitivity(p_in[1]);
          //ble_a111_status = true;
          //tap_fun = true;
          Mode = 2;
          printf("wave and hit function\n");
        }
        else if (A111_accel_both == 0x04)
        {
          Mode = 3;
          //ble_a111_status = true;
          //tap_fun = true;
          printf("wave function with change speed\n");
        }
        else
        {
          status = false;
          return status;
        }


        return status;
}

int _write(int file,char *ptr, int len)
{
	int i = 0;
	for(i = 0 ; i < len ; i++)
	{
		ITM_SendChar((*ptr++));
	}
	return len;
}


/**@brief Application main function.
 */
void Batt_level_meas_timeout_handler(void * p_context)
{
  #ifdef DEBUG
  printf("\nBattery cycle\n");
  #endif
  UNUSED_PARAMETER(p_context);
  uint8_t batt_level = read_batt_per();
  ret_code_t err = ble_bas_battery_level_update(&m_bas,batt_level,m_conn_handle);
  //A111_cyc_count_send_ble(read_batt_per());
  #ifdef DEBUG
  printf("\nBattery percentage = %d%%\n",batt_level);
  printf("\nBattery Full capacity = %.2f\n",read_batt_full_cap());
  printf("Battery Rema capacity = %.2f\n",read_batt_cap());
  printf("\n\n");
  #endif
}



void charg_cycle_timeout_handler1(void * p_context)
{
    #ifdef DEBUG
    printf("\nCharge cycle\n");
    #endif
    //if(chrg_pin_read())
    //{
      //led_ws2812b_reset();
      first = false;
      uint8_t led = 3;
      //if(charge == 0 && !chg_led_lock)
      //{
      //  if(!chg_led_lock)
      //  {
      //    LED_LS(false);
      //  }
      //}
      if(chrg_pin_read())
      {
        #ifdef DEBUG
        printf("Charg in cycle connected\n");
        #endif
        if(chrg >= 0 && chrg <= 30)
        {
        //printf("0-50\n");
        
          if(!chg_led_lock)
          {
            for(int i=0;i<NR_OF_PIXELS;i++)
            {
              if(i == led || i == led+4 || i == led+8 || i == led+12 || i == led+16)
               led_ws2812b_set_pixel(i, &led_red);
              else
               led_ws2812b_set_pixel(i,&led_black);
            }
            //led_ws2812b_show();
          }
        }
        else if(chrg > 30 && chrg <= 80)
        {
        //printf("50-80\n");
          if(!chg_led_lock)
          {
            for(int i=0;i<NR_OF_PIXELS;i++)
            {
              if(i == led || i == led+4 || i == led+8 || i == led+12 || i == led+16)
               led_ws2812b_set_pixel(i, &led_blue);
              else
               led_ws2812b_set_pixel(i,&led_black);
            }
            //led_ws2812b_show();
          }
        }
        else if(chrg >= 81  && chrg <= 99)
        {
        //printf("80-100\n");
          if(!chg_led_lock)
          {
            for(int i=0;i<NR_OF_PIXELS;i++)
            {
              if(i == led || i == led+4 || i == led+8 || i == led+12 || i == led+16)
               led_ws2812b_set_pixel(i, &led_green);
              else
               led_ws2812b_set_pixel(i,&led_black);
            }
            //led_ws2812b_show();
          }
        }
        else if(chrg == 100)
        {
        //printf("100\n");
          if(!chg_led_lock)
          {
              LED_LS(true);
              rgb_led_glow(0,255,0);
              sound1(20);
              rgb_led_glow(0,0,255);
              sound1(20);
              rgb_led_glow(0,255,0);
              sound1(20);
              rgb_led_glow(0,0,0);
              int_fuel_pin_enable();
              Charge_timer_stop();
              charge_conn=true;
              LED_LS(false);
          }
        }
        if(chgl)
        {
          if(!chg_led_lock)
            LED_LS(true);
          led_ws2812b_show();
          chgl = false;
        }
        else
        {
          if(!chg_led_lock)
            LED_LS(false);
          //app_timer_pause();
          Charge_timer_stop();
          chrg = read_batt_per();
          Charge_timer_start();
          //app_timer_resume();
          //printf("Remainnig charge = %d%%\n", chrg);
          chgl = true;
        }
      }
      else
      {
        #ifdef DEBUG
        printf("Charg in cycle disconnected\n");
        #endif
        #ifndef DEBUG
        printf("Charge is disconnected\n");
        #endif
        //app_timer_stop(charg_cycle);
        int_fuel_pin_enable();
        if(!chg_led_lock)
        in_fuel_pin_handler(0,0);
        //charge_pin_interrupt();
      }
}

void in_fuel_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    #ifdef DEBUG
    printf("Fuel gauge charge pin interrupt is activated\n");
    #endif
    if(chrg_pin_read())
    {
      printf("\nCharge is connected\n\n");
      charge_conn=false;
      if(!chg_led_lock)
      {
        LED_LS(true);
        led_ws2812b_reset();
      }
      chrg = read_batt_per();
      //charg_cycle_timeout_handler1(NULL);
      int_fuel_pin_disable();
      APP_ERROR_CHECK(app_timer_start(charg_cycle, CHRG_CHK_TIME, NULL));
      chgl = true;
    }
    else
    {
      APP_ERROR_CHECK(app_timer_stop(charg_cycle));
      if(!chg_led_lock)
      {
        LED_LS(false);
      }
      charge_conn=true;
      #ifdef DEBUG
      printf("Charge is disconnected\n");
      #endif
    }

}

void int_fuel_pin_enable(void)
{
  if(!chrg_in_en)
  {
    nrf_drv_gpiote_in_event_enable(FUEL_IN, true);
    chrg_in_en = true;
    chrg_in_dis = false;
  }
}

void int_fuel_pin_disable(void)
{
  if(!chrg_in_dis)
  {
    nrf_drv_gpiote_in_event_disable(FUEL_IN);
    chrg_in_en = false;
    chrg_in_dis = true;
  }
}

void gpio_init(void)
{
    ret_code_t err_code;

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_NOPULL;

    err_code = nrf_drv_gpiote_in_init(FUEL_IN, &in_config, in_fuel_pin_handler);
    APP_ERROR_CHECK(err_code);
    int_fuel_pin_enable();
    //nrf_drv_gpiote_in_event_enable(FUEL_IN, true);
}


void TW_Dashpod_init(void)
{
    log_init();
    ret_code_t err_code;
    #ifdef DEBUG
    printf("Debug firmware\n\n");
    #else
    printf("Release firmware\n\n");
    #endif
    FW_version_send_ble();
    printf("\nInitalization started\n");
    
    uint32_t bootloader_addr = BOOTLOADER_ADDRESS;
        
    if (bootloader_addr != 0xFFFFFFFF)
    {
      err_code = ble_dfu_buttonless_async_svci_init();
      APP_ERROR_CHECK(err_code);
    }

    bool erase_bonds = false;
    
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    power_management_init();
    //gpio_init();
    ble_stack_init();
    peer_manager_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    printf("BLE Initialized\n\n");

    bq2505_battery_init();
    bq2505_set_current_limit_mode(3);// BGATE : Force enable
    nrf_delay_ms(100);
    bq2505_set_current_limit_mode(11);//935mA
    nrf_delay_ms(50);
    bq2505_set_current_limit_mode(12);//temp: 0 to 60 degrees

    
    IO_buzz_init(&m_twi,0x21);
    nrf_delay_ms(100);
    led_ws2812b_init(LED_WS2812B);

    LED_LS(false);
    #ifdef DEBUG
    printf("Setting up battery\n\n");
    #endif

    setup_fuel_guage(&m_twi,0x36,1500);
   #ifndef DEBUG
    printf("Fuel gauge initalized\n\n");
    #endif
    #ifdef DEBUG
    printf("Battery percentage = %d%%\n",read_percentage_fuel_guage(&m_twi,0x36));
    printf("Battery capacity = %.2f\n",read_capacity_fuel_guage(&m_twi,0x36));
    printf("Battery percentage = %d%%\n",read_percentage_fuel_guage(&m_twi,0x36));
    printf("Battery capacity = %.2f\n",read_capacity_fuel_guage(&m_twi,0x36));
    #endif
    #ifndef DEBUG
    printf("Battery percentage = %d%%\n\n",read_percentage_fuel_guage(&m_twi,0x36));
    #endif
    nrf_delay_ms(300);

    setup_gyro_accel(&m_twi,0x6b);

    LED_LS(true);
    for(int i=0;i<NR_OF_PIXELS;i++)
    {
       led_ws2812b_set_pixel(i, &led_green);
    }
    led_ws2812b_show();

    //gpio_init();

    nrf_delay_ms(500);
    LED_LS(false);

      
    //nrf_temp_init();

    //ble_a111_status    = false;
    advertising_start(erase_bonds);
    
    chg_led_lock = false;
    gpio_init();
    //in_fuel_pin_handler(0,0);
    APP_ERROR_CHECK(app_timer_create(&charg_cycle, APP_TIMER_MODE_REPEATED, charg_cycle_timeout_handler1));

    in_fuel_pin_handler(0,0);

   printf("\nDashpod initalization completed\n\n");
    #ifdef DEBUG
    printf("Checking for A111 radar sensor\n");
    #endif

}

void TW_dashpod_start_loop(void)
{

  while(Mode)
  {
    switch(Mode)//Mode
    {
      case 1:
        int_fuel_pin_disable();
        nrf_drv_gpiote_in_uninit(FUEL_IN);
        tap_detect_exit();
        gpio_init();
        //Charge_timer_start();
        Mode = 4;
        break;
      case 2:
        int_fuel_pin_disable();
        nrf_drv_gpiote_in_uninit(FUEL_IN);
        acc_seq5_wave_tap_to_exit(0,0);
        gpio_init();
        //Charge_timer_start();
        Mode = 4;
        break;
      case 3:
        int_fuel_pin_disable();
        nrf_drv_gpiote_in_uninit(FUEL_IN);
        acc_seq6_wave_to_exit(0,0);
        gpio_init();
        //Charge_timer_start();
        Mode = 4;
        break;
      default :
        idle_state_handle();
        //hal_test_spi_read_chipid();
        //nrf_delay_ms(100);
    }
    //printf("\nin main Mode = %d\n\n",Mode);
  }
}

void TW_dashpod_wait_loop(void)
{
    for(;wait_loop;)
    {
        hal_test_spi_read_chipid();        //for testing the A111 module responding or not for given input
    }
 
}

void Charge_timer_start(void)
{
  if(!charge_conn)
  {
    APP_ERROR_CHECK(app_timer_start(charg_cycle, CHRG_CHK_TIME, NULL));
  }
}

void Charge_timer_stop(void)
{
  if(!charge_conn)
  {
    APP_ERROR_CHECK(app_timer_stop(charg_cycle));
  }
}

void Set_Mode()
{
      Mode=4;
}

int Get_Mode()
{
     return Mode;
}