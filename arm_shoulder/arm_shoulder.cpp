// Includes
#include <stdbool.h>
#include <stdint.h>

// TivaC specific includes
extern "C"
{
  #include <driverlib/sysctl.h>
  #include <driverlib/gpio.h>
  #include <driverlib/pwm.h>
  #include "inc/hw_memmap.h"
}

#include <bdc_motor.h>

// ROS includes
#include <ros.h>
#include <std_msgs/Int32.h>

// ROS nodehandle
ros::NodeHandle nh;

// Motor speed
volatile int32_t vel1 = 0;

void messageCb(const std_msgs::Int32& msg) {
  vel1 = msg.data;
}

ros::Subscriber<std_msgs::Int32> sub("motor_test_1", &messageCb);

int main(void) {
  MAP_FPUEnable();
  MAP_FPULazyStackingEnable();
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

  nh.initNode();
  nh.subscribe(sub);

  // Motor initialization
  BDC motor1; 
  // IN1 - Speed Output PB6
  motor1.SYSCTL_PERIPH_PWM_IN1 = SYSCTL_PERIPH_PWM0;
  motor1.SYSCTL_PERIPH_GPIO_IN1 = SYSCTL_PERIPH_GPIOB;
  motor1.GPIO_PWM_IN1 = GPIO_PB6_M0PWM0;
  motor1.GPIO_PORT_BASE_IN1 = GPIO_PORTB_BASE;
  motor1.GPIO_PIN_IN1 = GPIO_PIN_6;
  motor1.PWM_BASE_IN1 = PWM0_BASE;
  motor1.PWM_GEN_IN1 = PWM_GEN_0;
  motor1.PWM_OUT_BIT_IN1 = PWM_OUT_0_BIT;
  motor1.PWM_OUT_IN1 = PWM_OUT_0;
  // IN2 - Direction Output PB7
  motor1.SYSCTL_PERIPH_GPIO_IN2 = SYSCTL_PERIPH_GPIOB;
  motor1.GPIO_PORT_BASE_IN2 = GPIO_PORTB_BASE;
  motor1.GPIO_PIN_IN2 = GPIO_PIN_7;
  // nFAULT - Fault Status Input PE3
  motor1.SYSCTL_PERIPH_GPIO_nFAULT = SYSCTL_PERIPH_GPIOE;
  motor1.GPIO_PORT_BASE_nFAULT = GPIO_PORTE_BASE;
  motor1.GPIO_PIN_nFAULT = GPIO_PIN_3;
  // nRESET - Reset Output PA1
  motor1.SYSCTL_PERIPH_GPIO_nRESET = SYSCTL_PERIPH_GPIOA;
  motor1.GPIO_PORT_BASE_nRESET = GPIO_PORTA_BASE;
  motor1.GPIO_PIN_nRESET = GPIO_PIN_1;
  // BRAKE - Brake Output PF3
  motor1.SYSCTL_PERIPH_GPIO_BRAKE = SYSCTL_PERIPH_GPIOF;
  motor1.GPIO_PORT_BASE_BRAKE = GPIO_PORTF_BASE;
  motor1.GPIO_PIN_BRAKE = GPIO_PIN_3;
  // CS - Current Sense Input PE1
  motor1.SYSCTL_PERIPH_ADC_CS = SYSCTL_PERIPH_ADC0;
  motor1.SYSCTL_PERIPH_GPIO_CS = SYSCTL_PERIPH_GPIOE;
  motor1.GPIO_PORT_BASE_CS = GPIO_PORTE_BASE;
  motor1.GPIO_PIN_CS = GPIO_PIN_1;
  motor1.ADC_BASE_CS = ADC0_BASE;
  motor1.ADC_CTL_CH_CS = ADC_CTL_CH2;

  bdc_init(motor1);

  bdc_set_enabled(motor1, 1);

  while (1)
  {
    bdc_set_velocity(motor1, vel1);
    nh.spinOnce();
    nh.getHardware()->delay(100);
  }
}
