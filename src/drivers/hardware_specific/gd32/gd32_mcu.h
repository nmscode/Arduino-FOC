#ifndef GD32_DRIVER_MCU_DEF
#define GD32_DRIVER_MCU_DEF
#include "../../hardware_api.h"

#if defined(_GD32_DEF_)   // Declared in ArduinoCore-GD32/cores/arduino/gd32/gd32_def.h

// default pwm parameters
#define _PWM_RESOLUTION 12 // 12bit
#define _PWM_FREQUENCY 16000 // 16khz
#define _PWM_FREQUENCY_MAX 50000 // 50khz

// 6pwm parameters
#define _HARDWARE_6PWM 1
#define _SOFTWARE_6PWM 0
#define _ERROR_6PWM -1

#define RCU_TIMER_BLDC RCU_TIMER0
#define TIMER_BLDC TIMER0
#define DEAD_TIME 60

// Channel G
#define TIMER_BLDC_CHANNEL_G TIMER_CH_2
#define TIMER_BLDC_GH_PIN GPIO_PIN_10
#define TIMER_BLDC_GH_PORT GPIOA
#define TIMER_BLDC_GL_PIN GPIO_PIN_15
#define TIMER_BLDC_GL_PORT GPIOB
// Channel B
#define TIMER_BLDC_CHANNEL_B TIMER_CH_1
#define TIMER_BLDC_BH_PIN GPIO_PIN_9
#define TIMER_BLDC_BH_PORT GPIOA
#define TIMER_BLDC_BL_PIN GPIO_PIN_14
#define TIMER_BLDC_BL_PORT GPIOB
// Channel Y
#define TIMER_BLDC_CHANNEL_Y TIMER_CH_0
#define TIMER_BLDC_YH_PIN GPIO_PIN_8
#define TIMER_BLDC_YH_PORT GPIOA
#define TIMER_BLDC_YL_PIN GPIO_PIN_13
#define TIMER_BLDC_YL_PORT GPIOB

typedef struct GD32DriverParams {
  uint32_t timers[6] = {NULL};
  uint32_t channels[6];
  long pwm_frequency;
  long unsigned int range;
  float dead_zone;
  uint8_t interface_type;
} GD32DriverParams;


#endif
#endif