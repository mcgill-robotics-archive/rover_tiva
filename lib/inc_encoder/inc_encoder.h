#ifndef __INC_ENCODER_H_
#define __INC_ENCODER_H_
#include "../tiva_lib_includes.h"

typedef struct INCs {
    uint32_t PHA_GPIO_PIN;//ex: GPIO_PIN_3
    uint32_t PHB_GPIO_PIN;//ex: GPIO_PIN_3
    uint32_t IDX_GPIO_PIN;//ex: GPIO_PIN_3
    
    uint32_t QEI_SYSCTL_PERIPH_GPIO;//ex: SYSCTL_PERIPH_GPIOD

    uint32_t QEI_GPIO_P_PHA;//ex: GPIO_PD6_PHA0
    uint32_t QEI_GPIO_P_PHB;//externx: GPIO_PD7_PHB0
    uint32_t QEI_GPIO_P_IDX;//ex: GPIO_PD3_IDX0

    uint32_t QEI_SYSCTL_PERIPH_QEI; //ex :SYSCTL_PERIPH_QEI0
    uint32_t QEI_GPIO_PORT_BASE;//ex: GPIO_PORTF_BASE
    uint32_t QEI_BASE; //externxx: QEI0_BASE
    uint32_t QEI_VELDIV;//ex: QEI_VELDIV_1
} INC;

// Initialize everything needed for the encoder to run
void inc_init(INC cui);
//Read encoder values
int32_t inc_get_direction(INC cui);
uint32_t inc_get_velocity(INC cui);
uint32_t inc_get_position(INC cui);

#endif //__INC_ENCODER_H_

