/** @file JrkDefinitions.h
 *  @brief Linear Motor Definitions.
 *
 *  This file contains definitions used by Linear Motor Interface.
 *
 *  @author Vasilis Bosdelekidis.
 *  @bug No known bugs.
 */

#ifndef JRK_INTERFACE_JRKDEFINITIONS_H
#define JRK_INTERFACE_JRKDEFINITIONS_H

#define INPUT_VARIABLE 0xA1
#define TARGET_VARIABLE 0xA3
#define FEEDBACK_VARIABLE 0xA5
#define SCALED_FEEDBACK_VARIABLE 0xA7
#define ERROR_SUM_VARIABLE 0xA9
#define DUTY_CYCLE_TARGET_VARIABLE 0xAB
#define DUTY_CYCLE_VARIABLE 0xAD
#define CURRENT_VARIABLE 0x8F
#define PID_PERIOD_COUNT_VARIABLE 0xB1
#define ERRORS_FLAG_VARIABLE 0xB5
#define ERRORS_HALTING_VARIABLE 0xB3

#endif  // JRK_INTERFACE_JRKDEFINITIONS_H
