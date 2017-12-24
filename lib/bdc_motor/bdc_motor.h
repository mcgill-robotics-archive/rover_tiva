/*
 * bdc_motor.h
 *
 *  Created on: Mar 29, 2017
 *      Author: Steven Dahdah
 *
 * Spin motor, read faults, read current, actuate brake
 * Datasheet link: http://www.ti.com/lit/ds/symlink/drv8842.pdf
 *
 */

// TODO Clean up these includes
//#include <stdint.h>
//#include <stdlib.h>
//#include <stdbool.h>
//#include <math.h>
//#include <stdio.h>
//#include <inttypes.h>
//#include "inc/hw_memmap.h"
//#include "inc/hw_types.h"
//#include "inc/hw_gpio.h"
//#include "inc/hw_qei.h"
//#include "inc/hw_ssi.h"
//#include "driverlib/sysctl.h"
//#include "driverlib/interrupt.h"
//#include "driverlib/timer.h"
//#include "driverlib/fpu.h"
//#include "driverlib/gpio.h"
//#include "driverlib/debug.h"
//#include "driverlib/pwm.h"
//#include "driverlib/pin_map.h"
//#include "driverlib/qei.h"
//#include "driverlib/ssi.h"
//#include "driverlib/rom.h"
//#include "driverlib/systick.h"
//#include "driverlib/uart.h"
//#include "utils/uartstdio.h"
//#include "inc/tm4c123gh6pm.h"
//#include "driverlib/adc.h"

#include "../tiva_lib_includes.h"

#ifndef BDC_MOTOR_H_
#define BDC_MOTOR_H_

typedef struct BDCs {
  // IN1 - Speed Output
  uint32_t SYSCTL_PERIPH_PWM_IN1;
  uint32_t SYSCTL_PERIPH_GPIO_IN1;
  uint32_t GPIO_PWM_IN1;
  uint32_t GPIO_PORT_BASE_IN1;
  uint32_t GPIO_PIN_IN1;
  uint32_t PWM_BASE_IN1;
  uint32_t PWM_GEN_IN1;
  uint32_t PWM_OUT_BIT_IN1;
  uint32_t PWM_OUT_IN1;
  // IN2 - Direction Output
  uint32_t SYSCTL_PERIPH_GPIO_IN2;
  uint32_t GPIO_PORT_BASE_IN2;
  uint32_t GPIO_PIN_IN2;
  // nFAULT - Fault Status Input
  uint32_t SYSCTL_PERIPH_GPIO_nFAULT;
  uint32_t GPIO_PORT_BASE_nFAULT;
  uint32_t GPIO_PIN_nFAULT;
  // nRESET - Reset Output
  uint32_t SYSCTL_PERIPH_GPIO_nRESET;
  uint32_t GPIO_PORT_BASE_nRESET;
  uint32_t GPIO_PIN_nRESET;
  // BRAKE - Brake Output
  uint32_t SYSCTL_PERIPH_GPIO_BRAKE;
  uint32_t GPIO_PORT_BASE_BRAKE;
  uint32_t GPIO_PIN_BRAKE;
  // CS - Current Sense Input
  uint32_t SYSCTL_PERIPH_ADC_CS;
  uint32_t SYSCTL_PERIPH_GPIO_CS;
  uint32_t GPIO_PORT_BASE_CS;
  uint32_t GPIO_PIN_CS;
  uint32_t ADC_BASE_CS;
  uint32_t ADC_CTL_CH_CS;

  bool last_brake;

} BDC;

// Initialize all peripherals needed to interact with motor
void bdc_init(BDC bdc);

// Set motor velocity (+ve is CCW, -ve is CW, 0 is stop)
void bdc_set_velocity(BDC bdc, int32_t velocity);

// Read motor current
uint32_t bdc_get_current(BDC bdc);

// Engage or disengage brake
bool bdc_set_brake(BDC bdc, bool disengaged);

// Read fault status of motor driver
uint8_t bdc_get_fault(BDC bdc);

// Enable or disable motor driver. Toggle to reset a fault
void bdc_set_enabled(BDC bdc, uint8_t enabled);

#endif /* BDC_MOTOR_H_ */
