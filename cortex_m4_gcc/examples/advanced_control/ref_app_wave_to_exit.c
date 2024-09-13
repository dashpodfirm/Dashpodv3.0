// Copyright (c) Acconeer AB, 2022
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "acc_detector_presence.h"
#include "acc_hal_definitions_a121.h"
#include "acc_hal_integration_nrf52833.h"
#include "acc_integration.h"
#include "acc_rss_a121.h"
//#include "acc_service.h"
#include "acc_version.h"
#include "acc_definitions_a121.h"
#include "led_ws2812b.h"
#include "nrf_delay.h"
#include "ble_comm.h"
#include "IO_expa_buzzer.h"
#include "LSM6DSR.h"
#include "main_include.h"


// Default values for this reference application
// ---------------------------------------------

// Detector configuration settings
#define SENSOR_ID         (1U)
#define SENSOR_TIMEOUT_MS (2000U)
//#define RANGE_START_M     (0.13f)
//#define RANGE_LENGTH_M    (0.3f)
#define UPDATE_RATE_HZ    (48U)//
#define SWEEPS_PER_FRAME  (16U)//16u
#define HWAAS             (80U)
#define PROFILE           (ACC_CONFIG_PROFILE_2)
#define POWER_SAVE_MODE   (ACC_CONFIG_IDLE_STATE_SLEEP)
float prev=0;


// Algorithm constants that could need fine tuning by the user.

// Detection threshold, level at which to trigger a "wave to exit".
#define DETECTION_THRESHOLD (1.3f)

// Cool down threshold, level at which to be able to trigger again.
#define COOL_DOWN_THRESHOLD (1.3f)

// Cool down time, minimal time between triggers in ms.
#define COOL_DOWN_TIME_MS (0)

// Cool down time in ticks, derived from cooldown time and update rate.
#define COOL_DOWN_TIME_TICKS ((COOL_DOWN_TIME_MS * UPDATE_RATE_HZ) / 1000)
typedef enum {
    STATE_IDLE,
    STATE_WAVE_DETECTED,
    STATE_READY_FOR_NEXT_WAVE
} detection_state_t;


volatile bool     hit_cmd              = true;
volatile bool     tap_fun              = true;
volatile bool     seq                  = true;
volatile int      comp_score           = 50;
volatile uint8_t  loop                 = 0;
volatile uint8_t  axes                 = 2; //0->X, 1->Y, 2->Z, 3->XYZ 

extern volatile uint8_t pod_action;
extern volatile uint8_t hit;

extern nrf_drv_twi_t m_twi;

float RANGE_LENGTH_M = 0.01f;

float RANGE_START_M  = 0.12f;

/**
 * Configure the detector to the specified configuration
 *
 * @param configuration The detector configuration to configure with application settings
 */
static void config_detector(acc_detector_presence_config_t *presence_config)
{
        acc_detector_presence_config_sensor_set(presence_config, SENSOR_ID);
        acc_detector_presence_config_start_set(presence_config, RANGE_START_M);
        acc_detector_presence_config_end_set(presence_config,RANGE_LENGTH_M);
        acc_detector_presence_config_profile_set(presence_config,PROFILE);
        acc_detector_presence_config_hwaas_set(presence_config,HWAAS);
        acc_detector_presence_config_automatic_subsweeps_set(presence_config, true);
        acc_detector_presence_config_signal_quality_set(presence_config, 30.0f);//30.0f
        acc_detector_presence_config_inter_frame_idle_state_set(presence_config, POWER_SAVE_MODE);
        acc_detector_presence_config_sweeps_per_frame_set(presence_config, SWEEPS_PER_FRAME);//16
        acc_detector_presence_config_frame_rate_set(presence_config, UPDATE_RATE_HZ);//10.0f
        acc_detector_presence_config_frame_rate_app_driven_set(presence_config, false);
        acc_detector_presence_config_reset_filters_on_prepare_set(presence_config, true);
        acc_detector_presence_config_intra_detection_set(presence_config, true);
        //acc_detector_presence_config_intra_detection_threshold_set(presence_config, DETECTION_THRESHOLD);//1.4
        acc_detector_presence_config_intra_frame_time_const_set(presence_config, 0.05f);//0.15f
        acc_detector_presence_config_intra_output_time_const_set(presence_config, 0.03f);//0.3f
        acc_detector_presence_config_inter_detection_set(presence_config, false);
        acc_detector_presence_config_inter_detection_threshold_set(presence_config, 1.0f);//1.0f
        acc_detector_presence_config_inter_frame_deviation_time_const_set(presence_config, 0.03f);//0.5f
        acc_detector_presence_config_inter_frame_fast_cutoff_set(presence_config, 6.0f);//50.f
        acc_detector_presence_config_inter_frame_slow_cutoff_set(presence_config, 0.20f);//0.20f
        acc_detector_presence_config_inter_output_time_const_set(presence_config, 1.0f);//2.0f
        acc_detector_presence_config_inter_frame_presence_timeout_set(presence_config,0);//3
        acc_detector_presence_config_inter_phase_boost_set(presence_config, false);
}


static void cleanup(acc_detector_presence_handle_t *presence_handle,
                    acc_detector_presence_config_t *presence_config,
                    acc_sensor_t                   *sensor,
                    void                           *buffer)
{
	acc_hal_integration_sensor_disable(SENSOR_ID);
	acc_hal_integration_sensor_supply_off(SENSOR_ID);

	if (presence_config != NULL)
	{
		acc_detector_presence_config_destroy(presence_config);
	}

	if (presence_handle != NULL)
	{
		acc_detector_presence_destroy(presence_handle);
	}

	if (sensor != NULL)
	{
		acc_sensor_destroy(sensor);
	}

	if (buffer != NULL)
	{
		acc_integration_mem_free(buffer);
	}
}


static bool do_sensor_calibration(acc_sensor_t *sensor, acc_cal_result_t *cal_result, void *buffer, uint32_t buffer_size)
{
	bool           status              = false;
	bool           cal_complete        = false;
	const uint16_t calibration_retries = 1U;

	// Random disturbances may cause the calibration to fail. At failure, retry at least once.
	for (uint16_t i = 0; !status && (i <= calibration_retries); i++)
	{
		// Reset sensor before calibration by disabling/enabling it
		acc_hal_integration_sensor_disable(SENSOR_ID);
		acc_hal_integration_sensor_enable(SENSOR_ID);

		do
		{
			status = acc_sensor_calibrate(sensor, &cal_complete, cal_result, buffer, buffer_size);

			if (status && !cal_complete)
			{
				status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
			}
		} while (status && !cal_complete);
	}

	if (status)
	{
		/* Reset sensor after calibration by disabling/enabling it */
		acc_hal_integration_sensor_disable(SENSOR_ID);
		acc_hal_integration_sensor_enable(SENSOR_ID);
	}

	return status;
}



/*------------------------------------------------------------------------------
--------------------------------------------------------------------------------
                      App command working function
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------*/


int acc_seq5_wave_tap_to_exit(int argc, char *argv[])
{
        #ifdef DEBUG
        printf("In Mode 2\n");
        #endif

        RANGE_START_M = 0.05f;

        RANGE_LENGTH_M = 0.07f;//1.5
       
        (void)argc;
	(void)argv;
        void          *buffer     = NULL;
        uint32_t      buffer_size = 0U;
        acc_sensor_t  *sensor     = NULL;
        acc_detector_presence_metadata_t metadata;
        acc_detector_presence_config_t *config = NULL;
        acc_detector_presence_handle_t *handle = NULL;

	#ifdef DEBUG
	printf("Acconeer software version %s\n", acc_version_get());
        #endif

	const acc_hal_a121_t *hal = acc_hal_rss_integration_get_implementation();

	if (!acc_rss_hal_register(hal))
	{
		printf("Failed to activate RSS\n");
		return EXIT_FAILURE;
	}

	config = acc_detector_presence_config_create();

	if (config == NULL)
	{
		printf("Failed to create detector config\n");
		cleanup(0,config,sensor,buffer);
		return EXIT_FAILURE;
	}

	config_detector(config);

        //Print the configuration
        acc_detector_presence_config_log(config);
	handle = acc_detector_presence_create(config,&metadata);

	if (handle == NULL)
	{
		printf("Failed to create detector\n");
                cleanup(handle,config,sensor,buffer);
		return EXIT_FAILURE;
	}

        if (!acc_detector_presence_get_buffer_size(handle, &buffer_size))
	{
		printf("acc_detector_presence_get_buffer_size() failed\n");
		cleanup(handle,config, sensor, buffer);
		return EXIT_FAILURE;
	}

	buffer = acc_integration_mem_alloc(buffer_size);
	if (buffer == NULL)
	{
		printf("buffer allocation failed\n");
		cleanup(handle,config, sensor, buffer);
		return EXIT_FAILURE;
	}

        acc_hal_integration_sensor_supply_on(SENSOR_ID);
        acc_hal_integration_sensor_enable(SENSOR_ID);

        sensor=acc_sensor_create(SENSOR_ID);

        acc_cal_result_t cal_result;

        if (!do_sensor_calibration(sensor, &cal_result, buffer, buffer_size))
	{
		printf("do_sensor_calibration() failed\n");
		cleanup(handle,config, sensor, buffer);
		return EXIT_FAILURE;
	}

	acc_detector_presence_result_t result;

	bool     status         = true;
	bool     cool_trig      = true;
	bool     cool_time      = true;
	uint32_t cool_counter   = 0;
        int      count          = 0;
        uint8_t  i              = 100;

        setup_Tap_INT(&m_twi,0x6B);


       status = acc_detector_presence_prepare(handle,config,sensor,&cal_result,buffer,buffer_size);

       #ifdef DEBUG
       printf("Dashpod not continous wave and tap function Mode 2\n");
       #endif

       float pscore = 1000.000;
       float diff_score = 0;


	while (status && (Get_Mode() == 2))
	{

                acc_detector_presence_result_t result;

		//acc_integration_sleep_until_periodic_wakeup();
                if (!acc_sensor_measure(sensor))
		{
			printf("acc_sensor_measure failed\n");
			cleanup(handle, config, sensor, buffer);
			return EXIT_FAILURE;
		}

		if (!acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS))
		{
			printf("Sensor interrupt timeout\n");
			cleanup(handle, config, sensor, buffer);
			return EXIT_FAILURE;
		}

		if (!acc_sensor_read(sensor, buffer, buffer_size))
		{
			printf("acc_sensor_read failed\n");
			cleanup(handle, config, sensor, buffer);
			return EXIT_FAILURE;
		}

		if (!acc_detector_presence_process(handle, buffer, &result))
		{
			printf("acc_detector_presence_process failed\n");
			cleanup(handle, config, sensor, buffer);
			return EXIT_FAILURE;
		}

                //printf("intra score= %f\t inter score = %f\t",result.intra_presence_score,result.inter_presence_score);


		if (result.processing_result.data_saturated)
		{
			printf("Data saturated. The detector result is not reliable.\n");
		}

		if (result.processing_result.frame_delayed)
		{
			//printf("Frame delayed. Could not read data fast enough.\n");
			//printf("Try lowering the frame rate or call 'acc_sensor_read' more frequently.\n");
		}

		/* If "calibration_needed" is indicated, the sensor needs to be recalibrated. */
		if (result.processing_result.calibration_needed)
		{
			printf("Sensor recalibration needed ... \n");

			if (!do_sensor_calibration(sensor, &cal_result, buffer, buffer_size))
			{
				printf("do_sensor_calibration() failed\n");
				cleanup(handle, config, sensor, buffer);
				return EXIT_FAILURE;
			}

			printf("Sensor recalibration done!\n");

			/* Before measuring again, the sensor needs to be prepared through the detector. */
			if (!acc_detector_presence_prepare(handle, config, sensor, &cal_result,
			                                   buffer, buffer_size))
			{
				printf("acc_detector_presence_prepare() failed\n");
				cleanup(handle, config, sensor, buffer);
				return EXIT_FAILURE;
			}
                        
		}
                if(!seq || !cool_trig || !cool_time)
                {
                        if (status)
                        {
                              if(hit)
                              {
                                    printf("\nWaiting for Wave\n");
                                    hit = 0;
                              }
                              acc_sensor_read(sensor, buffer, buffer_size);

                              bool wave_to_exit = false;
                              bool comp_wave_tap = false;

                              #ifdef DEBUG
                              printf("Presence score: %f\t previous score : %f\t",result.intra_presence_score*100,pscore*100);
                              #endif

                               diff_score = ((result.intra_presence_score*100) - (pscore*100));

                               #ifdef DEBUG
                               printf("diff score is %f\n",diff_score);
                               #endif


                               comp_wave_tap = diff_score > comp_score;

                              //printf("presence detected is %d\n",result.presence_detected);

                               if(result.presence_detected && cool_trig && cool_time)
                               {
                                        wave_to_exit = true;
                                        cool_trig    = false;
                                        cool_time    = false;
                               }

                               // The first cool-down criteria is that the presence score must fall below a threshold
                               // before a new "Wave to exit" is triggered

                              if (!cool_trig && (result.intra_presence_score < COOL_DOWN_THRESHOLD))
                              {

                                  #ifdef DEBUG
                                  printf("In cool down time threshold\t");
                                  #endif
                                  cool_trig    = true;
                                  cool_counter = COOL_DOWN_TIME_TICKS;
                                  #ifdef DEBUG
                                  printf("cool counter = %d\n",cool_counter);
                                  #endif

                              }
                              else if(!seq && !cool_trig && (comp_wave_tap) )
                              {
                                    #ifdef DEBUG
                                    printf("Second wave\n");
                                    #endif
                                    wave_to_exit = true;
                              }

                              // The second cool-down criteria is that a certain time, if set, must elapse before
                              // a new "Wave to exit" is triggered
                              if (!cool_time && cool_trig)
                              {
                                  #ifdef DEBUG
                                  printf("In cool time\n\n------------------------------------------------------\n\n\n\n");
                                  #endif
                                  if (cool_counter > 0)
                                  {
                                          cool_counter -= 1;
                                  }

                                  if (cool_counter == 0)
                                  {
                                          cool_time = true;
                                  }
                               }
                               if (wave_to_exit)
                               {
                                  
                                  printf("\nWave detected\n");
                                  printf("Waiting for Tap...\n");

                                  float data[3];
                                  short int data_main[3];

                                    while(i--)
                                    { 
                                         if(is_tap_detected(&m_twi))//x = 0; Y = 1; Z = 2;
                                         { 
                                                read_accel(&m_twi,0x6b,(uint8_t *)data_main);
                                                process_raw_accel(data_main, data);
                                                A111_tap_send_ble(data);                                
                                                seq = true;
                                                //pscore = 0;
                                                LED_LS(false);
                                                printf("Tap_triggerd\n");
                                                Beep();
                                                i=0;  
                                          }    
                                    } 
                                }//end of if(wave to exit)
                        }//end of if(status)
                }//end of if(!seq)
        }//end of while(status)
        clear_Tap_INT(&m_twi,0x6B);
        #ifdef DEBUG
        printf("A121 Function is stoped\n");
        #endif
        cleanup(handle,config,sensor,buffer);
	return status ? EXIT_SUCCESS : EXIT_FAILURE;
}



int acc_seq6_wave_to_exit(int argc, char *argv[])
{
        #ifdef DEBUG
        printf("In Mode 3\n");
        #endif
        
        RANGE_LENGTH_M = 1.2f;//1.5//1.0
        RANGE_START_M = 0.13f;


        
	bool     status         = true;
	bool     cool_trig      = true;
	bool     cool_time      = true;
	uint32_t cool_counter   = 0;


        (void)argc;
	(void)argv;
        void          *buffer     = NULL;
        uint32_t      buffer_size = 0U;
        acc_sensor_t  *sensor     = NULL;
        acc_detector_presence_config_t *config = NULL;
        acc_detector_presence_handle_t *handle = NULL;

        acc_detector_presence_metadata_t metadata;

	#ifdef DEBUG
        printf("Acconeer software version %s\n", acc_version_get());
        #endif

	const acc_hal_a121_t *hal = acc_hal_rss_integration_get_implementation();

	if (!acc_rss_hal_register(hal))
	{
		printf("Failed to activate RSS\n");
		return EXIT_FAILURE;
	}

	config = acc_detector_presence_config_create();

	if (config == NULL)
	{
		printf("Failed to create detector config\n");
		cleanup(handle,config,sensor,buffer);
		return EXIT_FAILURE;
	} 

	config_detector(config);
       
        //Print the configuration
        acc_detector_presence_config_log(config);

	handle = acc_detector_presence_create(config, &metadata);

	if (handle == NULL)
	{
		printf("Failed to create detector\n");
                cleanup(handle,config,sensor,buffer);
		return EXIT_FAILURE;
	}

        if (!acc_detector_presence_get_buffer_size(handle, &buffer_size))
	{
		printf("acc_detector_presence_get_buffer_size() failed\n");
		cleanup(handle,config, sensor, buffer);
		return EXIT_FAILURE;
	}

	buffer = acc_integration_mem_alloc(buffer_size);

	if (buffer == NULL)
	{
		printf("buffer allocation failed\n");
		cleanup(handle,config, sensor, buffer);
		return EXIT_FAILURE;
	}

        acc_hal_integration_sensor_supply_on(SENSOR_ID);
        acc_hal_integration_sensor_enable(SENSOR_ID);

        sensor = acc_sensor_create(SENSOR_ID);

        if (sensor == NULL)
	{
		printf("acc_sensor_create() failed\n");
		cleanup(handle,config, sensor, buffer);
		return EXIT_FAILURE;
	}

        acc_cal_result_t cal_result;

        if (!do_sensor_calibration(sensor, &cal_result, buffer, buffer_size))
	{
		printf("do_sensor_calibration() failed\n");
		cleanup(handle,config, sensor, buffer);
		return EXIT_FAILURE;
	}

       status = acc_detector_presence_prepare(handle,config,sensor,&cal_result,buffer,buffer_size);

       #ifdef DEBUG
        printf("Dashpod not continous wave function Mode 3\n");
        #endif

       float pscore = 1000.000;
       float diff_score = 0;

	while (status && (Get_Mode() == 3))
	{

                acc_detector_presence_result_t result;

		//acc_integration_sleep_until_periodic_wakeup();
                if (!acc_sensor_measure(sensor))
		{
			printf("acc_sensor_measure failed\n");
			cleanup(handle, config, sensor, buffer);
			return EXIT_FAILURE;
		}

		if (!acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS))
		{
			printf("Sensor interrupt timeout\n");
			cleanup(handle, config, sensor, buffer);
			return EXIT_FAILURE;
		}

		if (!acc_sensor_read(sensor, buffer, buffer_size))
		{
			printf("acc_sensor_read failed\n");
			cleanup(handle, config, sensor, buffer);
			return EXIT_FAILURE;
		}

		if (!acc_detector_presence_process(handle, buffer, &result))
		{
			printf("acc_detector_presence_process failed\n");
			cleanup(handle, config, sensor, buffer);
			return EXIT_FAILURE;
		}
              
                //printf("intra score= %f\t inter score = %f\t",result.intra_presence_score,result.inter_presence_score);


		if (result.processing_result.data_saturated)
		{
			printf("Data saturated. The detector result is not reliable.\n");
		}

		if (result.processing_result.frame_delayed)
		{
			//printf("Frame delayed. Could not read data fast enough.\n");
			//printf("Try lowering the frame rate or call 'acc_sensor_read' more frequently.\n");
		}

		/* If "calibration_needed" is indicated, the sensor needs to be recalibrated. */
		if (result.processing_result.calibration_needed)
		{
			printf("Sensor recalibration needed ... \n");

			if (!do_sensor_calibration(sensor, &cal_result, buffer, buffer_size))
			{
				printf("do_sensor_calibration() failed\n");
				cleanup(handle, config, sensor, buffer);
				return EXIT_FAILURE;
			}

			printf("Sensor recalibration done!\n");

			/* Before measuring again, the sensor needs to be prepared through the detector. */
			if (!acc_detector_presence_prepare(handle, config, sensor, &cal_result,
			                                   buffer, buffer_size))
			{
				printf("acc_detector_presence_prepare() failed\n");
				cleanup(handle, config, sensor, buffer);
				return EXIT_FAILURE;
			}
                        
		}

                if(!seq || !cool_time || !cool_trig)
                {

                      if (status)
                      {
                              if(hit)
                              {
                                  printf("\nWaiting for Wave\n");
                                  hit = 0;
                              }

                              (pod_action == 1 || pod_action == 2)?sound1(0.5):0;
                              acc_sensor_read(sensor, buffer, buffer_size);


                              bool wave_to_exit = false;
                              bool comp_wave_tap = false;

                              #ifdef DEBUG
                              printf("Presence score: %f\t previous score : %f\t",result.intra_presence_score*100,pscore*100);
                              #endif

                              diff_score = ((result.intra_presence_score*100) - (pscore*100));

                              #ifdef DEBUG
                              printf("diff score is %f\n",diff_score);
                              #endif


                              comp_wave_tap = diff_score > comp_score;

                              //printf("presence detected is %d\n",result.presence_detected);

                              if(result.presence_detected && cool_trig && cool_time)
                              {
                                        wave_to_exit = true;
                                        cool_trig    = false;
                                        cool_time    = false;
                              }


                              if (!cool_trig && (result.intra_presence_score < COOL_DOWN_THRESHOLD))
                              {

                                  #ifdef DEBUG
                                  printf("In cool down time threshold\t");
                                  #endif
                                  cool_trig    = true;
                                  cool_counter = COOL_DOWN_TIME_TICKS;
                                  #ifdef DEBUG
                                  printf("cool counter = %d\n",cool_counter);
                                  #endif

                              }
                              else if(!seq && !cool_trig && (comp_wave_tap) )
                              {
                                    #ifdef DEBUG
                                    printf("Second wave\n");
                                    #endif
                                    wave_to_exit = true;
                              }


                              // The second cool-down criteria is that a certain time, if set, must elapse before
                              // a new "Wave to exit" is triggered
                              if (!cool_time && cool_trig)
                              {
                                  #ifdef DEBUG
                                  printf("In cool time\n\n------------------------------------------------------\n\n\n\n");
                                  #endif
                                  if (cool_counter > 0)
                                  {
                                          cool_counter -= 1;
                                  }

                                  if (cool_counter == 0)
                                  {
                                          cool_time = true;
                                  }
                               }
                              
                                                  
                              if (wave_to_exit)
                              {
                                      //printf("\nDistance is = %f\t",result.presence_distance*100);
                                      //print_distance_result(&dis_result);
                                      printf("\nWave detected\n");
                                      seq = true;
                                      LED_LS(false);
                                      Beep();
                                      A111_wave_send_ble();
                                                            
                              }
                              pscore = result.intra_presence_score;
                             
                              
                      }
               }
        }
        #ifdef DEBUG
        printf("A121 Function is stoped\n");
        #endif
        acc_detector_presence_destroy(handle);
        acc_detector_presence_config_destroy(config);
        cleanup(handle,config,sensor,buffer);
	return status ? EXIT_SUCCESS : EXIT_FAILURE;
}



int tap_detect_exit(void)
{
      #ifdef DEBUG
      printf("Tap detection enabled\n");
      #endif

      setup_Tap_INT(&m_twi,0x6B);

      tap_enable(&m_twi);

      printf("\nWaiting for tap\n");
      
	while (Get_Mode() == 1)
	{
          if(!seq)
          {
              if(hit)
              {
                    printf("\nWaiting for tap\n");
                    hit = 0;
              }
              float data[3];
              short int data_main[3];

              //printf("hello world\n");

              if(is_tap_detected(&m_twi))//X = 0; Y = 1; Z = 2;
              { 
                  read_accel(&m_twi,0x6b,(uint8_t *)data_main);
                  process_raw_accel(data_main, data);
                  A111_tap_send_ble(data);
                  seq = true;
                  LED_LS(false);
                  printf("Tap_triggerd\n");
                  Beep();
              }
          } //end of if(!seq)//
	}//end of while(status) 
	// We will never exit
        clear_Tap_INT(&m_twi,0x6B);
        #ifdef DEBUG
        printf("Tap detection Function is stoped\n");
        #endif
	return 0;
}
