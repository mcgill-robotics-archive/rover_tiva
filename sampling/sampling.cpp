#ifdef SAMPLING
#define MOTOR_A "motor_carriage"
#define MOTOR_B "motor_drill"
#define MOTOR_RESET "motor_shoulder_reset"
#define INC_ENCODER_A "inc_drill"
#endif

// Standard includes
#include <stdbool.h>
#include <stdint.h>

// MR Lib includes
#include "../lib/bdc_motor/bdc_motor.h"
#include "../lib/inc_encoder/inc_encoder.h"

// ROS includeiis
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

// TivaC specific includes
extern "C" {
#include <driverlib/gpio.h>
#include <driverlib/pwm.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.c>
#include <driverlib/sysctl.c>
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"
}

#define LC_PERIPH SYSCTL_PERIPH_GPIOA
#define LC_BASE GPIO_PORTA_BASE
#define TLC GPIO_PIN_2
#define BLC GPIO_PIN_3

#define P_PERIPH SYSCTL_PERIPH_GPIOB
#define P_BASE GPIO_PORTB_BASE
#define P GPIO_PIN_6

#define LP_PERIPH SYSCTL_PERIPH_GPIOA
#define LP_BASE GPIO_PORTA_BASE
#define TLP GPIO_PIN_4
#define BLP GPIO_PIN_5

// ROS nodehandle
ros::NodeHandle nh;

// Motor speeds
volatile int32_t vel_a = 0;
volatile int32_t vel_b = 0;
volatile uint32_t inc_pos_a = 0;
volatile uint32_t inc_vel_a = 0;
volatile int32_t inc_dir_a = 0;

void PWM_Pulse(uint32_t Speed);
void PWM_Stop(void);
void PWM_Config(void);
uint32_t Data[3];

// Probe variables
#ifdef SAMPLING
volatile int32_t vel_c = 0;
#endif

bool reset_flag = false;

void vel_a_cb(const std_msgs::Int32& msg) { vel_a = msg.data; }
ros::Subscriber<std_msgs::Int32> sub_a(MOTOR_A, &vel_a_cb);

void vel_b_cb(const std_msgs::Int32& msg) { vel_b = msg.data; }
ros::Subscriber<std_msgs::Int32> sub_b(MOTOR_B, &vel_b_cb);

void reset_cb(const std_msgs::Bool& status) { reset_flag = status.data; }

ros::Subscriber<std_msgs::Bool> sub_reset(MOTOR_RESET, &reset_cb);

std_msgs::Int32 inc_a_msg;
ros::Publisher inc_encoder_a(INC_ENCODER_A, &inc_a_msg);

std_msgs::Int32 probe_wind_msg;
ros::Publisher probe_wind("probe_wind", &probe_wind_msg);

std_msgs::Int32 probe_humidity_msg;
ros::Publisher probe_humidity("probe_humidity", &probe_humidity_msg);

std_msgs::Int32 probe_temperature_msg;
ros::Publisher probe_temperature("probe_temperature", &probe_temperature_msg);

#ifdef SAMPLING
void vel_c_cb(const std_msgs::Int32& msg) { vel_c = msg.data; }
ros::Subscriber<std_msgs::Int32> sub_c("probe", &vel_c_cb);
#endif

void PWM_Pulse(int32_t Speed) {
  if (Speed >= 50) {
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN);
  }

  else if (Speed < 50) {
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
  }
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,
                   (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 100) * Speed);
  PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
  PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}

void PWM_Stop(void) {
  // Enable the PWM peripheral
  PWMGenDisable(PWM0_BASE, PWM_GEN_0);
}

void PWM_Config(void) {
  SysCtlPWMClockSet(SYSCTL_PWMDIV_8);

  // Enable the PWM peripheral and wait for it to be ready.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)) {
  }

  // Enable the GPIO peripheral and wait for it to be ready.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {
  }

  // Configure the internal multiplexer to connect the PWM peripheral to PF3
  GPIOPinConfigure(GPIO_PB6_M0PWM0);

  // Set up the PWM module on pin PF3
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

  /* Configure PWM mode to count up/down without synchronization
   * This is just a detail, you probably don't have to worry about it.
   * See pages 1235 and 1237 in the data sheet if you want to learn more.
   */
  PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
                  PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 200000);
}

void ADC_Init(void) {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);  // Enable the ADC peripheral and
                                               // wait for it to be ready.
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {
  }

  ADCHardwareOversampleConfigure(ADC0_BASE, 64);

  SysCtlPeripheralEnable(
      SYSCTL_PERIPH_GPIOD);  // Enable GPIO port D and wait for it to be ready.
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)) {
  }
  SysCtlPeripheralEnable(
      SYSCTL_PERIPH_GPIOE);  // Enable GPIO port E and wait for it to be ready.
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) {
  }

  GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);
  GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3);
  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

  ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR,
                       0);  // Configure the ADC sample sequence.

  ADCSequenceStepConfigure(
      ADC0_BASE, 3, 0,
      ADC_CTL_CH5 | ADC_CTL_IE |
          ADC_CTL_END);  // Configure the ADC sample sequence steps-PD2-HUMIDITY
  ADCSequenceStepConfigure(
      ADC0_BASE, 3, 0,
      ADC_CTL_CH4 | ADC_CTL_IE | ADC_CTL_END);  // PD3-TEMPERATURE
  ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
                           ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);  // PE3-WIND

  ADCSequenceEnable(
      ADC0_BASE,
      3);  // Since sample sequence 3 is now configured, it must be enabled.

  ADCIntClear(ADC0_BASE, 3);
}

void Probe(void) {
  // wind

  ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
                           ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);  // PE3
  ADCIntClear(ADC0_BASE, 3);
  ADCProcessorTrigger(ADC0_BASE, 3);  // Trigger the ADC conversion.
  while (!ADCIntStatus(ADC0_BASE, 3, false)) {
  }                           // Wait for conversion to be completed.
  ADCIntClear(ADC0_BASE, 3);  // Clear the ADC interrupt flag.
  ADCSequenceDataGet(ADC0_BASE, 3, &Data[0]);  // Read ADC Value. ADC has 12-bit
                                               // precision so the output ranges
                                               // from 0 to 4095

  // temperature
  ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
                           ADC_CTL_CH4 | ADC_CTL_IE | ADC_CTL_END);  // PD3
  ADCIntClear(ADC0_BASE, 3);
  ADCProcessorTrigger(ADC0_BASE, 3);
  while (!ADCIntStatus(ADC0_BASE, 3, false)) {
  }
  ADCIntClear(ADC0_BASE, 3);
  ADCSequenceDataGet(ADC0_BASE, 3, &Data[1]);

  // humidity
  ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
                           ADC_CTL_CH5 | ADC_CTL_IE | ADC_CTL_END);  // PD2
  ADCIntClear(ADC0_BASE, 3);
  ADCProcessorTrigger(ADC0_BASE, 3);
  while (!ADCIntStatus(ADC0_BASE, 3, false)) {
  }
  ADCIntClear(ADC0_BASE, 3);
  ADCSequenceDataGet(ADC0_BASE, 3, &Data[2]);

  // wind-tuning
  if (Data[0] > (uint32_t)495 &
      Data[0] < (uint32_t)2483) {  // the voltage is above 0.4 and below 2v.
    Data[0] = (((Data[0] - 496) * 324) / 1986);
  }

  else {
    Data[0] = 0;
  }
  // temperature-tuning
  if (Data[1] > (uint32_t)1551) {
    Data[1] = (((Data[1] - 1552)) / 6);
  }

  else {
    Data[1] = 0;
  }
  // humidity-tuning
  if (Data[2] < 0) {
    Data[2] = 0;
  }
}

int main(void) {
  // THIS UNLOCKING MECHANISM NEEDS LOOKING INTO.
  // HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
  // HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
  // HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

  // Carriage Limit Switch Variables
  volatile int32_t TLC_value = 0;
  volatile int32_t BLC_value = 0;

  // Tiva boilerplate
  MAP_FPUEnable();
  MAP_FPULazyStackingEnable();
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                     SYSCTL_OSC_MAIN);

  // Rosserial boilerplate
  nh.initNode();
  nh.subscribe(sub_a);
  nh.subscribe(sub_b);
  nh.subscribe(sub_c);
  nh.advertise(inc_encoder_a);
  nh.advertise(probe_wind);
  nh.advertise(probe_humidity);
  nh.advertise(probe_temperature);

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

  // Limit switch initialization
  SysCtlPeripheralEnable(LC_PERIPH);
  while (!SysCtlPeripheralReady(LC_PERIPH)) {
  };
  GPIOPinTypeGPIOInput(LC_BASE, TLC);
  GPIOPinTypeGPIOInput(LC_BASE, BLC);
  GPIOPadConfigSet(LC_BASE, TLC, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
  GPIOPadConfigSet(LC_BASE, BLC, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

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

  // Incremental Encoder A (QEI 1)-Drill
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

  bdc_init(motor_a);
  bdc_set_enabled(motor_a, 1);
  bdc_init(motor_b);
  bdc_set_enabled(motor_b, 1);

  inc_init(inc_a);

  // PROBE initialization

  GPIOPinTypeGPIOInput(LP_BASE, TLP);
  GPIOPinTypeGPIOInput(LP_BASE, BLP);
  GPIOPadConfigSet(LP_BASE, TLP, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
  GPIOPadConfigSet(LP_BASE, BLP, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
  PWM_Config();
  PWM_Stop();
  ADC_Init();

  while (1) {
    TLC_value = GPIOPinRead(LC_BASE, TLC);
    BLC_value = GPIOPinRead(LC_BASE, BLC);

    if (TLC_value == TLC && vel_a > 0) {
      bdc_set_velocity(motor_a, vel_a);
    } else if (BLC_value == BLC && vel_a < 0) {
      bdc_set_velocity(motor_a, vel_a);
    } else {
      bdc_set_velocity(motor_a, 0);
    }

    bdc_set_velocity(motor_b, vel_b);

    if (GPIOPinRead(LP_BASE, TLP) == TLP && vel_c >= 50) {
      PWM_Pulse(vel_c);
    }

    else if (GPIOPinRead(LP_BASE, BLP) == BLP && vel_c < 50) {
      PWM_Pulse(vel_c);
    }

    else {
      PWM_Stop();
    }

    Probe();
    probe_wind_msg.data = Data[0];
    probe_temperature_msg.data = Data[1];
    probe_humidity_msg.data = Data[2];
    probe_wind.publish(&probe_wind_msg);
    probe_temperature.publish(&probe_temperature_msg);
    probe_humidity.publish(&probe_humidity_msg);

    inc_dir_a = inc_get_direction(inc_a);
    inc_vel_a = inc_get_velocity(inc_a);
    inc_pos_a = inc_get_position(inc_a);
    inc_a_msg.data = inc_get_position(inc_a);
    inc_encoder_a.publish(&inc_a_msg);

    nh.spinOnce();
    nh.getHardware()->delay(10);
  }
}
