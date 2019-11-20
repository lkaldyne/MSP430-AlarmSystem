#ifndef MAIN_H_
#define MAIN_H_

#include "driverlib/driverlib.h"

#define TIMER_A_PERIOD  1000 //T = 1/f = (TIMER_A_PERIOD * 1 us)
#define HIGH_COUNT      500  //Number of cycles signal is high (Duty Cycle = HIGH_COUNT / TIMER_A_PERIOD)

//LaunchPad LED1 - note unavailable if UART is used
//#define LED1_PORT       GPIO_PORT_P1
//#define LED1_PIN        GPIO_PIN0
//LaunchPad LED2
//#define LED2_PORT       GPIO_PORT_P4
//#define LED2_PIN        GPIO_PIN0
//LaunchPad Pushbutton Switch 1
#define SW1_PORT        GPIO_PORT_P1
#define SW1_PIN         GPIO_PIN2
//LaunchPad Pushbutton Switch 2
#define SW2_PORT        GPIO_PORT_P2
#define SW2_PIN         GPIO_PIN6
//Input to ADC - in this case input A9 maps to pin P8.1
// This is our MIC in
#define MIC_IN_PORT     GPIO_PORT_P8
#define MIC_IN_PIN      GPIO_PIN1
#define MIC_IN_CHANNEL  ADC_INPUT_A9
//Launchpad pins
// Buzzer
#define BUZZER_PORT        GPIO_PORT_P1
#define BUZZER_PIN         GPIO_PIN7
// LED 1
#define LED1_PORT        GPIO_PORT_P1
#define LED1_PIN         GPIO_PIN5
// LED 2
#define LED2_PORT        GPIO_PORT_P1
#define LED2_PIN         GPIO_PIN4
// LED 3
#define LED3_PORT        GPIO_PORT_P2
#define LED3_PIN         GPIO_PIN7
// LED 4
#define LED4_PORT        GPIO_PORT_P2
#define LED4_PIN         GPIO_PIN5
// MUX
#define MUX_CONTROL_0_PORT        GPIO_PORT_P1
#define MUX_CONTROL_0_PIN         GPIO_PIN6
#define MUX_CONTROL_1_PORT        GPIO_PORT_P5
#define MUX_CONTROL_1_PIN         GPIO_PIN0
#define DOOR_SENSOR_IN_PORT     GPIO_PORT_P1
#define DOOR_SENSOR_IN_PIN      GPIO_PIN3

void Init_GPIO(void);
void Init_Clock(void);
void Init_UART(void);
void Init_PWM(void);
void Init_ADC(void);

Timer_A_outputPWMParam param; //Timer configuration data structure for PWM

#endif /* MAIN_H_ */
