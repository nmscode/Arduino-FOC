
#ifndef GD32_CURRENTSENSE_MCU_DEF
#define GD32_CURRENTSENSE_MCU_DEF
#include "../../hardware_api.h"
#include "../../../common/foc_utils.h"

#if defined(_GD32_DEF_) 

// generic implementation of the hardware specific structure
// containing all the necessary current sense parameters
// will be returned as a void pointer from the _configureADCx functions
// will be provided to the _readADCVoltageX() as a void pointer
typedef struct GD32CurrentSenseParams {
  int pins[3] = {(int)NOT_SET};
  float adc_voltage_conv;
  //ADC_HandleTypeDef* adc_handle = NP;
  //HardwareTimer* timer_handle = NP;
} GD32CurrentSenseParams;

//#define ADC_USE_INTERRUPT         // If defined, will use the End of conversion interrupt of inserted ADC
#define PHASE_CURRENT_FILTER 0.10 // Used with the interrupt. If 1.00, no filter 

#endif
#endif