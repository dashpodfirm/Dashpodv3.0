// Copyright (c) Acconeer AB, 2022
// All rights reserved


#ifndef REF_APP_WAVE_TO_EXIT_H_
#define REF_APP_WAVE_TO_EXIT_H_

#include <stdbool.h>

/**
 * @brief Wave to exit reference application
 *
 * @return Returns EXIT_SUCCESS if successful, otherwise EXIT_FAILURE
 */


int acc_seq4_wave_to_exit(int argc, char *argv[]);

int acc_seq5_wave_tap_to_exit(int argc, char *argv[]);

int acc_seq6_wave_to_exit(int argc, char *argv[]);

int tap_detect_exit(void);
//static int (*fun[5])(int argc, char *argv[]) = {&acc_seq1_wave_to_exit,&acc_seq2_wave_to_exit,&acc_seq3_wave_to_exit, &acc_seq4_wave_to_exit, &acc_seq5_wave_to_exit};

//fun[0] = &acc_seq1_wave_to_exit;
//fun[1] = &acc_seq2_wave_to_exit;
//fun[3] = &acc_seq3_wave_to_exit;
//fun[4] = &acc_seq4_wave_to_exit;
#endif
