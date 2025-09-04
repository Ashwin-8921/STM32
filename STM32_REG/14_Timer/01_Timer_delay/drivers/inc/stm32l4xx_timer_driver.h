

#ifndef INC_STM32L4XX_TIMER_DRIVER_H_
#define INC_STM32L4XX_TIMER_DRIVER_H_


void timer2_1hz_init(void);           // Init Timer2 for 1Hz tick


#define SR_UIF    (1U<<0)   // Update interrupt flag
#define SR_CC1IF  (1U<<1)   // Capture/Compare 1 interrupt flag




#endif /* INC_STM32L4XX_TIMER_DRIVER_H_ */
