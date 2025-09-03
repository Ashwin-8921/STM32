

#ifndef INC_STM32L4XX_ADC_DRIVER_H_
#define INC_STM32L4XX_ADC_DRIVER_H_

#include <stdint.h>

//Initialize ADC peripheral (clock, GPIO, calibration, setup)
void adc_init(void);

// Trigger ADC conversion (continuous mode supported)
void start_conversion(void);

// Read latest ADC conversion result (blocking read)
uint16_t adc_read(void);


#endif /* INC_STM32L4XX_ADC_DRIVER_H_ */
