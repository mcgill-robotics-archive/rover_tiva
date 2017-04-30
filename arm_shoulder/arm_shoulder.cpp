// Standard includes
#include <stdbool.h>
#include <stdint.h>

// MR Lib includes
#include "../lib/bdc_motor/bdc_motor.h"

// ROS includes
#include <ros.h>
#include <std_msgs/Int32.h>

// TivaC specific includes
extern "C"
{
  #include <driverlib/sysctl.h>
  #include <driverlib/gpio.h>
  #include <driverlib/pwm.h>
  #include "inc/hw_memmap.h"
}


// ROS nodehandle
ros::NodeHandle nh;

// Motor speeds
volatile int32_t vel_a = 0;
volatile int32_t vel_b = 0;

void vel_a_cb(const std_msgs::Int32& msg) {
  vel_a = msg.data;
}

ros::Subscriber<std_msgs::Int32> sub_a("motor_shoulder_a", &vel_a_cb);

void vel_b_cb(const std_msgs::Int32& msg) {
  vel_b = msg.data;
}

ros::Subscriber<std_msgs::Int32> sub_b("motor_shoulder_b", &vel_b_cb);

int main(void) {
  // Tiva boilerplate
  MAP_FPUEnable();
  MAP_FPULazyStackingEnable();
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
  // Rosserial boilerplate
  nh.initNode();
  nh.subscribe(sub_a);
  nh.subscribe(sub_b);

  // Motor initialization
  BDC motor_a; 
  // IN1 - Speed Output PB4
  motor_a.SYSCTL_PERIPH_PWM_IN1 = SYSCTL_PERIPH_PWM0;
  motor_a.SYSCTL_PERIPH_GPIO_IN1 = SYSCTL_PERIPH_GPIOB;
  motor_a.GPIO_PWM_IN1 = GPIO_PB4_M0PWM2;
  motor_a.GPIO_PORT_BASE_IN1 = GPIO_PORTB_BASE;
  motor_a.GPIO_PIN_IN1 = GPIO_PIN_4;
  motor_a.PWM_BASE_IN1 = PWM0_BASE;
  motor_a.PWM_GEN_IN1 = PWM_GEN_1;
  motor_a.PWM_OUT_BIT_IN1 = PWM_OUT_2_BIT;
  motor_a.PWM_OUT_IN1 = PWM_OUT_2;
  // IN2 - Direction Output PB5
  motor_a.SYSCTL_PERIPH_GPIO_IN2 = SYSCTL_PERIPH_GPIOB;
  motor_a.GPIO_PORT_BASE_IN2 = GPIO_PORTB_BASE;
  motor_a.GPIO_PIN_IN2 = GPIO_PIN_5;
  // nFAULT - Fault Status Input PE2
  motor_a.SYSCTL_PERIPH_GPIO_nFAULT = SYSCTL_PERIPH_GPIOE;
  motor_a.GPIO_PORT_BASE_nFAULT = GPIO_PORTE_BASE;
  motor_a.GPIO_PIN_nFAULT = GPIO_PIN_2;
  // nRESET - Reset Output PA0
  motor_a.SYSCTL_PERIPH_GPIO_nRESET = SYSCTL_PERIPH_GPIOA;
  motor_a.GPIO_PORT_BASE_nRESET = GPIO_PORTA_BASE;
  motor_a.GPIO_PIN_nRESET = GPIO_PIN_0;
  // BRAKE - Brake Output PF2
  motor_a.SYSCTL_PERIPH_GPIO_BRAKE = SYSCTL_PERIPH_GPIOF;
  motor_a.GPIO_PORT_BASE_BRAKE = GPIO_PORTF_BASE;
  motor_a.GPIO_PIN_BRAKE = GPIO_PIN_2;
  // CS - Current Sense Input PE0
  motor_a.SYSCTL_PERIPH_ADC_CS = SYSCTL_PERIPH_ADC0;
  motor_a.SYSCTL_PERIPH_GPIO_CS = SYSCTL_PERIPH_GPIOE;
  motor_a.GPIO_PORT_BASE_CS = GPIO_PORTE_BASE;
  motor_a.GPIO_PIN_CS = GPIO_PIN_1;
  motor_a.ADC_BASE_CS = ADC0_BASE;
  motor_a.ADC_CTL_CH_CS = ADC_CTL_CH3;

  // Motor initialization
  BDC motor_b; 
  // IN1 - Speed Output PB6
  motor_b.SYSCTL_PERIPH_PWM_IN1 = SYSCTL_PERIPH_PWM0;
  motor_b.SYSCTL_PERIPH_GPIO_IN1 = SYSCTL_PERIPH_GPIOB;
  motor_b.GPIO_PWM_IN1 = GPIO_PB6_M0PWM0;
  motor_b.GPIO_PORT_BASE_IN1 = GPIO_PORTB_BASE;
  motor_b.GPIO_PIN_IN1 = GPIO_PIN_6;
  motor_b.PWM_BASE_IN1 = PWM0_BASE;
  motor_b.PWM_GEN_IN1 = PWM_GEN_0;
  motor_b.PWM_OUT_BIT_IN1 = PWM_OUT_0_BIT;
  motor_b.PWM_OUT_IN1 = PWM_OUT_0;
  // IN2 - Direction Output PB7
  motor_b.SYSCTL_PERIPH_GPIO_IN2 = SYSCTL_PERIPH_GPIOB;
  motor_b.GPIO_PORT_BASE_IN2 = GPIO_PORTB_BASE;
  motor_b.GPIO_PIN_IN2 = GPIO_PIN_7;
  // nFAULT - Fault Status Input PE3
  motor_b.SYSCTL_PERIPH_GPIO_nFAULT = SYSCTL_PERIPH_GPIOE;
  motor_b.GPIO_PORT_BASE_nFAULT = GPIO_PORTE_BASE;
  motor_b.GPIO_PIN_nFAULT = GPIO_PIN_3;
  // nRESET - Reset Output PA1
  motor_b.SYSCTL_PERIPH_GPIO_nRESET = SYSCTL_PERIPH_GPIOA;
  motor_b.GPIO_PORT_BASE_nRESET = GPIO_PORTA_BASE;
  motor_b.GPIO_PIN_nRESET = GPIO_PIN_1;
  // BRAKE - Brake Output PF3
  motor_b.SYSCTL_PERIPH_GPIO_BRAKE = SYSCTL_PERIPH_GPIOF;
  motor_b.GPIO_PORT_BASE_BRAKE = GPIO_PORTF_BASE;
  motor_b.GPIO_PIN_BRAKE = GPIO_PIN_3;
  // CS - Current Sense Input PE1
  motor_b.SYSCTL_PERIPH_ADC_CS = SYSCTL_PERIPH_ADC0;
  motor_b.SYSCTL_PERIPH_GPIO_CS = SYSCTL_PERIPH_GPIOE;
  motor_b.GPIO_PORT_BASE_CS = GPIO_PORTE_BASE;
  motor_b.GPIO_PIN_CS = GPIO_PIN_1;
  motor_b.ADC_BASE_CS = ADC0_BASE;
  motor_b.ADC_CTL_CH_CS = ADC_CTL_CH2;

  bdc_init(motor_a);
  bdc_set_enabled(motor_a, 1);

  while (1)
  {
    bdc_set_velocity(motor_a, vel_a);
    nh.spinOnce();
    //nh.getHardware()->delay(100);
  }
}
