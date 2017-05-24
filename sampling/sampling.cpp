#ifdef SAMPLING
#define MOTOR_A "motor_carriage"
#define MOTOR_B "motor_drill"
#define MOTOR_RESET "motor_shoulder_reset"
#define INC_ENCODER_A "inc_carriage"
#define INC_ENCODER_B "inc_drill"
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
#include <std_msgs/Bool.h>




// TivaC specific includes
extern "C"
{
  #include "driverlib/pin_map.h"
  #include <driverlib/sysctl.c> 
  #include <driverlib/sysctl.h>
  #include <driverlib/gpio.c>
  #include <driverlib/gpio.h>
  #include <driverlib/pwm.h>
  #include "inc/hw_memmap.h"
}


#define BUTTON_PERIPH SYSCTL_PERIPH_GPIOA
#define BUTTON_BASE GPIO_PORTA_BASE
#define BUTTON GPIO_PIN_5
volatile uint32_t value=0;
  

#define LC_PERIPH SYSCTL_PERIPH_GPIOA
#define LC_BASE GPIO_PORTA_BASE
#define TLC GPIO_PIN_2
#define BLC GPIO_PIN_3



#define LED_PERIPH SYSCTL_PERIPH_GPIOD
#define LED_BASE GPIO_PORTD_BASE
#define RED_LED GPIO_PIN_0


// ROS nodehandle
ros::NodeHandle nh;

// Motor speeds
volatile int32_t vel_a = 0;
volatile int32_t vel_b = 0;
volatile uint32_t inc_pos_a = 0;
volatile uint32_t inc_vel_a = 0;
volatile int32_t inc_dir_a = 0;
volatile uint32_t inc_pos_b = 0;
volatile uint32_t inc_vel_b = 0;
volatile int32_t inc_dir_b = 0;


bool reset_flag = false;

void vel_a_cb(const std_msgs::Int32& msg) {
  vel_a = msg.data;
}
ros::Subscriber<std_msgs::Int32> sub_a(MOTOR_A, &vel_a_cb);

void vel_b_cb(const std_msgs::Int32& msg) {
  vel_b = msg.data;
}
ros::Subscriber<std_msgs::Int32> sub_b(MOTOR_B, &vel_b_cb);

void reset_cb(const std_msgs::Bool& status){
  reset_flag = status.data;
}

ros::Subscriber<std_msgs::Bool> sub_reset(MOTOR_RESET, &reset_cb);

std_msgs::Int32 inc_a_msg;
ros::Publisher inc_encoder_a(INC_ENCODER_A, &inc_a_msg);

std_msgs::Int32 inc_b_msg;
ros::Publisher inc_encoder_b(INC_ENCODER_B, &inc_b_msg);

int main(void) {
        //THIS UNLOCKING MECHANISM NEEDS LOOKING INTO. 
        //HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
        //HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
        //HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0; 
  //Limit Switch Variables
  volatile int32_t TLC_value=0;
  volatile int32_t BLC_value=0;

  // Tiva boilerplate
  MAP_FPUEnable();
  MAP_FPULazyStackingEnable();
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

  // Rosserial boilerplate
  nh.initNode();
  nh.subscribe(sub_a);
  nh.subscribe(sub_b);
  nh.advertise(inc_encoder_a);
  nh.advertise(inc_encoder_b);

  // Motor initialization-CARRIAGE 
  BDC motor_a;
  // IN2 - Speed Output PA7
  motor_a.SYSCTL_PERIPH_PWM_IN1 = SYSCTL_PERIPH_PWM1;
  motor_a.SYSCTL_PERIPH_GPIO_IN1 = SYSCTL_PERIPH_GPIOA;
  motor_a.GPIO_PWM_IN1 = GPIO_PA7_M1PWM3;
  motor_a.GPIO_PORT_BASE_IN1 = GPIO_PORTA_BASE;
  motor_a.GPIO_PIN_IN1 = GPIO_PIN_7;
  motor_a.PWM_BASE_IN1 = PWM1_BASE;
  motor_a.PWM_GEN_IN1 = PWM_GEN_1;
  motor_a.PWM_OUT_BIT_IN1 = PWM_OUT_3_BIT;
  motor_a.PWM_OUT_IN1 = PWM_OUT_3;
  // IN1 - Direction Output PA6
  motor_a.SYSCTL_PERIPH_GPIO_IN2 = SYSCTL_PERIPH_GPIOA;
  motor_a.GPIO_PORT_BASE_IN2 = GPIO_PORTA_BASE;
  motor_a.GPIO_PIN_IN2 = GPIO_PIN_6;
  // nFAULT - Fault Status Input PE0
  motor_a.SYSCTL_PERIPH_GPIO_nFAULT = SYSCTL_PERIPH_GPIOE;
  motor_a.GPIO_PORT_BASE_nFAULT = GPIO_PORTE_BASE;
  motor_a.GPIO_PIN_nFAULT = GPIO_PIN_0;
  // nRESET - Reset Output PD7 
  motor_a.SYSCTL_PERIPH_GPIO_nRESET = SYSCTL_PERIPH_GPIOD;
  motor_a.GPIO_PORT_BASE_nRESET = GPIO_PORTD_BASE;
  motor_a.GPIO_PIN_nRESET = GPIO_PIN_7;
  // BRAKE - Brake Output PA0
  motor_a.SYSCTL_PERIPH_GPIO_BRAKE = SYSCTL_PERIPH_GPIOA;
  motor_a.GPIO_PORT_BASE_BRAKE = GPIO_PORTA_BASE;
  motor_a.GPIO_PIN_BRAKE = GPIO_PIN_0;
  // CS - Current Sense Input PB4
  motor_a.SYSCTL_PERIPH_ADC_CS = SYSCTL_PERIPH_ADC1;
  motor_a.SYSCTL_PERIPH_GPIO_CS = SYSCTL_PERIPH_GPIOB;
  motor_a.GPIO_PORT_BASE_CS = GPIO_PORTB_BASE;
  motor_a.GPIO_PIN_CS = GPIO_PIN_4;
  motor_a.ADC_BASE_CS = ADC1_BASE;
  motor_a.ADC_CTL_CH_CS = ADC_CTL_CH10;
  
  //Limit switch initialization
  SysCtlPeripheralEnable(LC_PERIPH);
  while(!SysCtlPeripheralReady(LC_PERIPH)){};
SysCtlPeripheralEnable(BUTTON_PERIPH);
  while(!SysCtlPeripheralReady(BUTTON_PERIPH)){}
  SysCtlDelay(3);
  GPIOPinTypeGPIOInput(LC_BASE, TLC);
  GPIOPinTypeGPIOInput(LC_BASE, BLC);
  GPIOPadConfigSet(LC_BASE, TLC, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
  GPIOPadConfigSet(LC_BASE, BLC, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);  
  
  // LED 
  SysCtlPeripheralEnable(LED_PERIPH);
  while(!SysCtlPeripheralReady(LED_PERIPH)){}
  GPIOPinTypeGPIOOutput(LED_BASE, RED_LED);
  GPIOPinTypeGPIOInput(BUTTON_BASE, BUTTON);
  GPIOPadConfigSet(BUTTON_BASE, BUTTON, GPIO_STRENGTH_2MA,     GPIO_PIN_TYPE_STD_WPU);
  

  // Motor initialization-DRILL
  BDC motor_b;
  // IN2 - Speed Output-PF3
  motor_b.SYSCTL_PERIPH_PWM_IN1 = SYSCTL_PERIPH_PWM1;
  motor_b.SYSCTL_PERIPH_GPIO_IN1 = SYSCTL_PERIPH_GPIOF;
  motor_b.GPIO_PWM_IN1 = GPIO_PF3_M1PWM7;
  motor_b.GPIO_PORT_BASE_IN1 = GPIO_PORTF_BASE;
  motor_b.GPIO_PIN_IN1 = GPIO_PIN_3;
  motor_b.PWM_BASE_IN1 = PWM1_BASE;
  motor_b.PWM_GEN_IN1 = PWM_GEN_3;
  motor_b.PWM_OUT_BIT_IN1 = PWM_OUT_7_BIT;
  motor_b.PWM_OUT_IN1 = PWM_OUT_7;
  // IN1 - Direction Output-PF2
  motor_b.SYSCTL_PERIPH_GPIO_IN2 = SYSCTL_PERIPH_GPIOF;
  motor_b.GPIO_PORT_BASE_IN2 = GPIO_PORTF_BASE;
  motor_b.GPIO_PIN_IN2 = GPIO_PIN_2;
  // nFAULT - Fault Status Input PE2
  motor_b.SYSCTL_PERIPH_GPIO_nFAULT = SYSCTL_PERIPH_GPIOE;
  motor_b.GPIO_PORT_BASE_nFAULT = GPIO_PORTE_BASE;
  motor_b.GPIO_PIN_nFAULT = GPIO_PIN_2;
  // nRESET - Reset Output PE1
  motor_b.SYSCTL_PERIPH_GPIO_nRESET = SYSCTL_PERIPH_GPIOE;
  motor_b.GPIO_PORT_BASE_nRESET = GPIO_PORTE_BASE;
  motor_b.GPIO_PIN_nRESET = GPIO_PIN_1;
  // BRAKE - Brake Output PA1
  motor_b.SYSCTL_PERIPH_GPIO_BRAKE = SYSCTL_PERIPH_GPIOA;
  motor_b.GPIO_PORT_BASE_BRAKE = GPIO_PORTA_BASE;
  motor_b.GPIO_PIN_BRAKE = GPIO_PIN_1;
  // CS - Current Sense Input-PB5
  motor_b.SYSCTL_PERIPH_ADC_CS = SYSCTL_PERIPH_ADC1;
  motor_b.SYSCTL_PERIPH_GPIO_CS = SYSCTL_PERIPH_GPIOB;
  motor_b.GPIO_PORT_BASE_CS = GPIO_PORTB_BASE;
  motor_b.GPIO_PIN_CS = GPIO_PIN_5;
  motor_b.ADC_BASE_CS = ADC1_BASE;
  motor_b.ADC_CTL_CH_CS = ADC_CTL_CH11;

  // Incremental Encoder A (QEI 1)
  INC inc_a;
  inc_a.PHA_GPIO_PIN = GPIO_PIN_5;
  inc_a.PHB_GPIO_PIN = GPIO_PIN_6;
  inc_a.IDX_GPIO_PIN = GPIO_PIN_4;
  inc_a.QEI_SYSCTL_PERIPH_GPIO = SYSCTL_PERIPH_GPIOC;
  inc_a.QEI_GPIO_P_PHA = GPIO_PC5_PHA1;
  inc_a.QEI_GPIO_P_PHB = GPIO_PC6_PHB1;
  inc_a.QEI_GPIO_P_IDX = GPIO_PC4_IDX1;
  inc_a.QEI_SYSCTL_PERIPH_QEI = SYSCTL_PERIPH_QEI1;
  inc_a.QEI_GPIO_PORT_BASE = GPIO_PORTC_BASE;
  inc_a.QEI_BASE = QEI1_BASE;
  inc_a.QEI_VELDIV = QEI_VELDIV_1;

  // Incremental Encoder B (QEI 0)-Not used
  INC inc_b;
  inc_b.PHA_GPIO_PIN = GPIO_PIN_0;
  inc_b.PHB_GPIO_PIN = GPIO_PIN_1;
  inc_b.IDX_GPIO_PIN = GPIO_PIN_4;
  inc_b.QEI_SYSCTL_PERIPH_GPIO = SYSCTL_PERIPH_GPIOF;
  inc_b.QEI_GPIO_P_PHA = GPIO_PF0_PHA0;
  inc_b.QEI_GPIO_P_PHB = GPIO_PF1_PHB0;
  inc_b.QEI_GPIO_P_IDX = GPIO_PF4_IDX0;
  inc_b.QEI_SYSCTL_PERIPH_QEI = SYSCTL_PERIPH_QEI0;
  inc_b.QEI_GPIO_PORT_BASE = GPIO_PORTF_BASE;
  inc_b.QEI_BASE = QEI0_BASE;
  inc_b.QEI_VELDIV = QEI_VELDIV_1; 

  bdc_init(motor_a);
  bdc_set_enabled(motor_a, 1);
  bdc_init(motor_b);
  bdc_set_enabled(motor_b, 1);

  inc_init(inc_a);
  inc_init(inc_b);

  while (1)
  {
    TLC_value= GPIOPinRead(LC_BASE,TLC);
    BLC_value= GPIOPinRead(LC_BASE,BLC);
    
  //value = GPIOPinRead(BUTTON_BASE, BUTTON);
  //  if( BLC_value == BLC)
  //  {
  //    GPIOPinWrite(LED_BASE, RED_LED, RED_LED);
  // }
  //  else
  //  {
  //    GPIOPinWrite(LED_BASE, RED_LED, 0);
  // }

    if(TLC_value == TLC && vel_a > 0)
    {
    bdc_set_velocity(motor_a, vel_a);
    }
    //else if (BLC_value == BLC && vel_a < 0)
    //{
   //  bdc_set_velocity(motor_a, vel_a);
    //}
    else
    {
      bdc_set_velocity(motor_a, 0);
    }
 // bdc_set_velocity(motor_b, vel_b);

  

    // if(reset_flag){
    //   nh.loginfo("reset");
    //   bdc_set_enabled(motor_a, 0);
    //   bdc_set_enabled(motor_b, 0);

    //   nh.getHardware()->delay(500);
    //   bdc_set_enabled(motor_a, 1);
    //   bdc_set_enabled(motor_b, 1);

    //   reset_flag = false;
    // }
    


    // inc_dir_a = inc_get_direction(inc_a);
    // inc_vel_a = inc_get_velocity(inc_a);
    // inc_pos_a = inc_get_position(inc_a);
    //inc_a_msg.data = inc_get_position(inc_a);
    //inc_encoder_a.publish(&inc_a_msg);
    //inc_b_msg.data = inc_get_position(inc_b);
    //inc_encoder_b.publish(&inc_b_msg);
    nh.spinOnce();
    nh.getHardware()->delay(10);
  }
}
