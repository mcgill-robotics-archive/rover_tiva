/*
 * bdc_motor.cpp
 *
 *  Created on: Mar 29, 2017
 *      Author: steven
 *
 * Spin motor, read faults, read current, actuate brake
 * Datasheet link: http://www.ti.com/lit/ds/symlink/drv8842.pdf
 *
 */

#include "bdc_motor.h"

// Initialize all peripherals needed to interact with motor
void bdc_init(BDC bdc) {
	//TODO Do we need to check if a peripheral is already enabled before
	//     enabling it?
	// PWM Config
	//TODO Change resolution if needed
	SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
	SysCtlPeripheralEnable(bdc.SYSCTL_PERIPH_PWM_IN1);
	while(!SysCtlPeripheralReady(bdc.SYSCTL_PERIPH_PWM_IN1)){}
	SysCtlPeripheralEnable(bdc.SYSCTL_PERIPH_GPIO_IN1);
	while(!SysCtlPeripheralReady(bdc.SYSCTL_PERIPH_GPIO_IN1)){}
	GPIOPinConfigure(bdc.GPIO_PWM_IN1);
	GPIOPinTypePWM(bdc.GPIO_PORT_BASE_IN1, bdc.GPIO_PIN_IN1);
	PWMGenConfigure(bdc.PWM_BASE_IN1, bdc.PWM_GEN_IN1, PWM_GEN_MODE_UP_DOWN |
	                    PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(bdc.PWM_BASE_IN1, bdc.PWM_GEN_IN1, 4000);
	PWMOutputState(bdc.PWM_BASE_IN1, bdc.PWM_OUT_BIT_IN1, true);
	PWMGenEnable(bdc.PWM_BASE_IN1, bdc.PWM_GEN_IN1);

	// Direction Config
	SysCtlPeripheralEnable(bdc.SYSCTL_PERIPH_GPIO_IN2);
	while(!SysCtlPeripheralReady(bdc.SYSCTL_PERIPH_GPIO_IN2)){}
	GPIOPinTypeGPIOOutput(bdc.GPIO_PORT_BASE_IN2, bdc.GPIO_PIN_IN2);

	// Fault Config
	SysCtlPeripheralEnable(bdc.SYSCTL_PERIPH_GPIO_nFAULT);
	while(!SysCtlPeripheralReady(bdc.SYSCTL_PERIPH_GPIO_nFAULT)){}
	GPIOPinTypeGPIOInput(bdc.GPIO_PORT_BASE_nFAULT, bdc.GPIO_PIN_nFAULT);

	// Reset Config
	SysCtlPeripheralEnable(bdc.SYSCTL_PERIPH_GPIO_nRESET);
	while(!SysCtlPeripheralReady(bdc.SYSCTL_PERIPH_GPIO_nRESET)){}
	GPIOPinTypeGPIOOutput(bdc.GPIO_PORT_BASE_nRESET, bdc.GPIO_PIN_nRESET);

	// Brake Config
	SysCtlPeripheralEnable(bdc.SYSCTL_PERIPH_GPIO_BRAKE);
	while(!SysCtlPeripheralReady(bdc.SYSCTL_PERIPH_GPIO_BRAKE)){}
	GPIOPinTypeGPIOOutput(bdc.GPIO_PORT_BASE_BRAKE, bdc.GPIO_PIN_BRAKE);

	// ADC Config
	//TODO Change sample depth if needed
	//uint32_t pui32ADC0Value[1];
	SysCtlPeripheralEnable(bdc.SYSCTL_PERIPH_ADC_CS);
	while(!SysCtlPeripheralReady(bdc.SYSCTL_PERIPH_ADC_CS)){}
	SysCtlPeripheralEnable(bdc.SYSCTL_PERIPH_GPIO_CS);
	while(!SysCtlPeripheralReady(bdc.SYSCTL_PERIPH_GPIO_CS)){}
	GPIOPinTypeADC(bdc.GPIO_PORT_BASE_CS, bdc.GPIO_PIN_CS);
	ADCSequenceConfigure(bdc.ADC_BASE_CS, 3, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(bdc.ADC_BASE_CS, 3, 0, bdc.ADC_CTL_CH_CS
			| ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(bdc.ADC_BASE_CS, 3);
	ADCIntClear(bdc.ADC_BASE_CS, 3);
}

// Set motor velocity (+ve is CCW, -ve is CW, 0 is stop)
void bdc_set_velocity(BDC bdc, int32_t velocity) {
	// Ensure that input stays in [-3999, -2] U [2, 3999] (PWM period is 4000)
	// PWM generation hardware does not work at 0% or 100% duty cycle
	int8_t sign = 1;
    if (velocity >= 0) {
        sign = 1;
    } else {
        sign = -1;
    }
    if(abs(velocity) < 2) {
        velocity = 0;
    } else if(abs(velocity >= 4000)) {
        velocity = sign * 3999;
    }
	// If velocity is 0, disable PWM gen
	// Otherwise enable it and set direction pin
	if (velocity == 0) {
		PWMOutputState(bdc.PWM_BASE_IN1, bdc.PWM_OUT_BIT_IN1, 0);
	} else if (sign == 1) {
        PWMOutputState(bdc.PWM_BASE_IN1, bdc.PWM_OUT_BIT_IN1, bdc.PWM_OUT_BIT_IN1);
		GPIOPinWrite(bdc.GPIO_PORT_BASE_IN2, bdc.GPIO_PIN_IN2, bdc.GPIO_PIN_IN2);
        // No duty cycle scaling is needed, since input is directly the duty cycle
        PWMPulseWidthSet(bdc.PWM_BASE_IN1, bdc.PWM_OUT_IN1, abs(velocity));
	} else {
		PWMOutputState(bdc.PWM_BASE_IN1, bdc.PWM_OUT_BIT_IN1, bdc.PWM_OUT_BIT_IN1);
		GPIOPinWrite(bdc.GPIO_PORT_BASE_IN2, bdc.GPIO_PIN_IN2, 0);
        // No duty cycle scaling is needed, since input is directly the duty cycle
        // However, need to invert the duty cycle when reversing direction
        PWMPulseWidthSet(bdc.PWM_BASE_IN1, bdc.PWM_OUT_IN1, 4000 - abs(velocity));
	}
}

// Read motor current
uint32_t bdc_get_current(BDC bdc) {
	//TODO write current sense
	return 0;
}

// Engage or disengage brake
void bdc_set_brake(BDC bdc, uint8_t engaged) {
	if(engaged) {
		GPIOPinWrite(bdc.GPIO_PORT_BASE_BRAKE, bdc.GPIO_PIN_BRAKE,
				bdc.GPIO_PIN_BRAKE);
	} else {
		GPIOPinWrite(bdc.GPIO_PORT_BASE_BRAKE, bdc.GPIO_PIN_BRAKE, 0);
	}
}

// Read fault status of motor driver
uint8_t bdc_get_fault(BDC bdc) {
	if(GPIOPinRead(bdc.GPIO_PORT_BASE_nFAULT, bdc.GPIO_PIN_nFAULT)
			== bdc.GPIO_PIN_nFAULT) {
		return 0;
	} else {
		return 1;
	}
}

// Enable or disable motor driver. Toggle to reset a fault
void bdc_set_enabled(BDC bdc, uint8_t enabled) {
	if(enabled) {
		GPIOPinWrite(bdc.GPIO_PORT_BASE_nRESET, bdc.GPIO_PIN_nRESET,
				bdc.GPIO_PIN_nRESET);
	} else {
		GPIOPinWrite(bdc.GPIO_PORT_BASE_nRESET, bdc.GPIO_PIN_nRESET, 0);
	}
}
