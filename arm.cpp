#ifdef ARM_SHOULDER
#define MOTOR_A "motor_shoulder_a"
#define MOTOR_B "motor_shoulder_b"
#define INC_ENCODER_A "inc_shoulder_a"
#define INC_ENCODER_B "inc_shoulder_b"
#endif

#ifdef ARM_ELBOW
#define MOTOR_A "motor_elbow_a"
#define MOTOR_B "motor_elbow_b"
#define INC_ENCODER_A "inc_elbow_a"
#define INC_ENCODER_B "inc_elbow_b"
#endif

#ifdef ARM_WRIST
#define MOTOR_A "motor_wrist_a"
#define MOTOR_B "motor_wrist_b"
#define INC_ENCODER_A "inc_wrist_a"
#define INC_ENCODER_B "inc_wrist_b"
#endif

// Standard includes
#include <stdbool.h>
#include <stdint.h>

// MR Lib includes
#include "../lib/bdc_motor/bdc_motor.h"
#include "../lib/inc_encoder/inc_encoder.h"

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
#ifdef ARM_WRIST
volatile int32_t vel_c = 0;
#endif
volatile uint32_t inc_pos_a = 0;
volatile uint32_t inc_vel_a = 0;
volatile int32_t inc_dir_a = 0;
volatile uint32_t inc_pos_b = 0;
volatile uint32_t inc_vel_b = 0;
volatile int32_t inc_dir_b = 0;

void vel_a_cb(const std_msgs::Int32& msg) {
  vel_a = msg.data;
}
ros::Subscriber<std_msgs::Int32> sub_a(MOTOR_A, &vel_a_cb);

void vel_b_cb(const std_msgs::Int32& msg) {
  vel_b = msg.data;
}
ros::Subscriber<std_msgs::Int32> sub_b(MOTOR_B, &vel_b_cb);

#ifdef ARM_WRIST
void vel_c_cb(const std_msgs::Int32& msg) {
  vel_c = msg.data;
}
ros::Subscriber<std_msgs::Int32> sub_c("motor_claw", &vel_c_cb);
#endif

std_msgs::Int32 pos_a_msg;
ros::Publisher inc_encoder_a(INC_ENCODER_A, &pos_a_msg);

std_msgs::Int32 pos_b_msg;
ros::Publisher inc_encoder_b(INC_ENCODER_B, &pos_b_msg);

int main(void) {
  // Tiva boilerplate
  MAP_FPUEnable();
  MAP_FPULazyStackingEnable();
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
  // Rosserial boilerplate
  nh.initNode();
  nh.subscribe(sub_a);
  nh.subscribe(sub_b);
  nh.advertise(inc_encoder_a);

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

  #ifdef ARM_WRIST
  // Motor initialization
  BDC motor_c; 
  // IN1 - Speed Output PA6
  motor_c.SYSCTL_PERIPH_PWM_IN1 = SYSCTL_PERIPH_PWM1;
  motor_c.SYSCTL_PERIPH_GPIO_IN1 = SYSCTL_PERIPH_GPIOA;
  motor_c.GPIO_PWM_IN1 = GPIO_PA6_M1PWM2;
  motor_c.GPIO_PORT_BASE_IN1 = GPIO_PORTA_BASE;
  motor_c.GPIO_PIN_IN1 = GPIO_PIN_6;
  motor_c.PWM_BASE_IN1 = PWM1_BASE;
  motor_c.PWM_GEN_IN1 = PWM_GEN_1;
  motor_c.PWM_OUT_BIT_IN1 = PWM_OUT_2_BIT;
  motor_c.PWM_OUT_IN1 = PWM_OUT_2;
  // IN2 - Direction Output PA7
  motor_c.SYSCTL_PERIPH_GPIO_IN2 = SYSCTL_PERIPH_GPIOA;
  motor_c.GPIO_PORT_BASE_IN2 = GPIO_PORTA_BASE;
  motor_c.GPIO_PIN_IN2 = GPIO_PIN_7;
  // nFAULT - Fault Status Input PD7
  motor_c.SYSCTL_PERIPH_GPIO_nFAULT = SYSCTL_PERIPH_GPIOD;
  motor_c.GPIO_PORT_BASE_nFAULT = GPIO_PORTD_BASE;
  motor_c.GPIO_PIN_nFAULT = GPIO_PIN_7;
  // nRESET - Reset Output PC7
  motor_c.SYSCTL_PERIPH_GPIO_nRESET = SYSCTL_PERIPH_GPIOC;
  motor_c.GPIO_PORT_BASE_nRESET = GPIO_PORTC_BASE;
  motor_c.GPIO_PIN_nRESET = GPIO_PIN_7;
  // BRAKE - Brake Output PE4
  motor_c.SYSCTL_PERIPH_GPIO_BRAKE = SYSCTL_PERIPH_GPIOE;
  motor_c.GPIO_PORT_BASE_BRAKE = GPIO_PORTE_BASE;
  motor_c.GPIO_PIN_BRAKE = GPIO_PIN_4;
  // CS - Current Sense Input PE5
  motor_c.SYSCTL_PERIPH_ADC_CS = SYSCTL_PERIPH_ADC0;
  motor_c.SYSCTL_PERIPH_GPIO_CS = SYSCTL_PERIPH_GPIOE;
  motor_c.GPIO_PORT_BASE_CS = GPIO_PORTE_BASE;
  motor_c.GPIO_PIN_CS = GPIO_PIN_5;
  motor_c.ADC_BASE_CS = ADC0_BASE;
  motor_c.ADC_CTL_CH_CS = ADC_CTL_CH8;
  #endif

  // Incremental Encoder A (QEI 0)
  INC inc_a;
  inc_a.PHA_GPIO_PIN = GPIO_PIN_0;
  inc_a.PHB_GPIO_PIN = GPIO_PIN_1;
  inc_a.IDX_GPIO_PIN = GPIO_PIN_4;
  inc_a.QEI_SYSCTL_PERIPH_GPIO = SYSCTL_PERIPH_GPIOF;
  inc_a.QEI_GPIO_P_PHA = GPIO_PF0_PHA0;
  inc_a.QEI_GPIO_P_PHB = GPIO_PF1_PHB0;
  inc_a.QEI_GPIO_P_IDX = GPIO_PF4_IDX0;
  inc_a.QEI_SYSCTL_PERIPH_QEI = SYSCTL_PERIPH_QEI0;
  inc_a.QEI_GPIO_PORT_BASE = GPIO_PORTF_BASE;
  inc_a.QEI_BASE = QEI0_BASE;
  inc_a.QEI_VELDIV = QEI_VELDIV_1;

  // Incremental Encoder B (QEI 1)
  INC inc_b;
  inc_b.PHA_GPIO_PIN = GPIO_PIN_5;
  inc_b.PHB_GPIO_PIN = GPIO_PIN_6;
  inc_b.IDX_GPIO_PIN = GPIO_PIN_4;
  inc_b.QEI_SYSCTL_PERIPH_GPIO = SYSCTL_PERIPH_GPIOC;
  inc_b.QEI_GPIO_P_PHA = GPIO_PC5_PHA1;
  inc_b.QEI_GPIO_P_PHB = GPIO_PC6_PHB1;
  inc_b.QEI_GPIO_P_IDX = GPIO_PC4_IDX1;
  inc_b.QEI_SYSCTL_PERIPH_QEI = SYSCTL_PERIPH_QEI1;
  inc_b.QEI_GPIO_PORT_BASE = GPIO_PORTC_BASE;
  inc_b.QEI_BASE = QEI1_BASE;
  inc_b.QEI_VELDIV = QEI_VELDIV_1;

  bdc_init(motor_a);
  bdc_set_enabled(motor_a, 1);
  bdc_init(motor_b);
  bdc_set_enabled(motor_b, 1);
  
  #ifdef ARM_WRIST
  bdc_init(motor_c);
  bdc_set_enabled(motor_c, 1);
  #endif

  inc_init(inc_a);
  inc_init(inc_b);

  while (1)
  {
    bdc_set_velocity(motor_a, vel_a);
    bdc_set_velocity(motor_b, vel_b);
    #ifdef ARM_WRIST
    bdc_set_velocity(motor_c, vel_c);
    #endif
    // inc_dir_a = inc_get_direction(inc_a);
    // inc_vel_a = inc_get_velocity(inc_a);
    // inc_pos_a = inc_get_position(inc_a);
    pos_a_msg.data = inc_get_position(inc_a);
    inc_encoder_a.publish(&pos_a_msg);
    pos_b_msg.data = inc_get_position(inc_b);
    inc_encoder_b.publish(&pos_b_msg);
    nh.spinOnce();
    nh.getHardware()->delay(100);
  }
}
