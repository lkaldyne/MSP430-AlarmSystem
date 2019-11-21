#include "main.h"
#include "driverlib/driverlib.h"
#include "hal_LCD.h"

/*
 * This project contains some code samples that may be useful.
 *
 */

char ADCState = 0; //Busy state of the ADC
int16_t ADCResult = 0; //Storage for the ADC conversion result
typedef struct
{
    int hours;
    int minutes;
    int seconds;
    char cSeconds[2];
    char cMinutes[2];
    char cHours[2];
} timer;

typedef struct
{
    timer armTime;
    timer disarmTime;
    int armed;
    int led_port;
    int led_pin;
    int s0;
    int s1;
    int flash_delay;
    int flash_switch;


} room;

timer alarmTimer;

room rooms[4];

int alarmActive = 0;

int decrement_timer(timer *alarmTimer);
int compare_times(timer timer1, timer timer2);

void main(void)
{
    int magnetState = 0; //Current button press state (to allow edge detection)
    int led_state = 0;
    /*alarmTimer.cHours[0] = '1';
    alarmTimer.cHours[1] = '1';
    alarmTimer.cMinutes[0] = '0';
    alarmTimer.cMinutes[1] = '0';
    alarmTimer.cSeconds[0] = '0';
    alarmTimer.cSeconds[1] = '0';*/
    int counter = 0;

    rooms[0].led_port = GPIO_PORT_P2;
    rooms[0].led_pin = GPIO_PIN5;
    rooms[1].led_port = GPIO_PORT_P2;
    rooms[1].led_pin = GPIO_PIN7;
    rooms[2].led_port = GPIO_PORT_P1;
    rooms[2].led_pin = GPIO_PIN4;
    rooms[3].led_port = GPIO_PORT_P1;
    rooms[3].led_pin = GPIO_PIN5;

    rooms[0].s0 = 0;
    rooms[0].s1 = 1;
    rooms[1].s0 = 1;
    rooms[1].s1 = 0;
    rooms[2].s0 = 0;
    rooms[2].s1 = 0;
    rooms[3].s0 = 1;
    rooms[3].s1 = 1;


    /*
     * Functions with two underscores in front are called compiler intrinsics.
     * They are documented in the compiler user guide, not the IDE or MCU guides.
     * They are a shortcut to insert some assembly code that is not really
     * expressible in plain C/C++. Google "MSP430 Optimizing C/C++ Compiler
     * v18.12.0.LTS" and search for the word "intrinsic" if you want to know
     * more.
     * */


    //Turn off interrupts during initialization
    __disable_interrupt();

    //Stop watchdog timer unless you plan on using it
    WDT_A_hold(WDT_A_BASE);

    // Initializations - see functions for more detail
    Init_GPIO();    //Sets all pins to output low as a default
    Init_PWM();     //Sets up a PWM output
    Init_ADC();     //Sets up the ADC to sample
    Init_Clock();   //Sets up the necessary system clocks
    Init_UART();    //Sets up an echo over a COM port
    Init_LCD();     //Sets up the LaunchPad LCD display

     /*
     * The MSP430 MCUs have a variety of low power modes. They can be almost
     * completely off and turn back on only when an interrupt occurs. You can
     * look up the power modes in the Family User Guide under the Power Management
     * Module (PMM) section. You can see the available API calls in the DriverLib
     * user guide, or see "pmm.h" in the driverlib directory. Unless you
     * purposefully want to play with the power modes, just leave this command in.
     */
    PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    //All done initializations - turn interrupts back on.
    __enable_interrupt();

    displayScrollText("GRP 11 DEMO");

    // Initialize RTC
    // Source = 32kHz crystal, divided by 1024
    RTCCTL = RTCSS__XT1CLK | RTCSR | RTCPS__1024 | RTCIE;
    RTCMOD = 32-1;

    __bis_SR_register(GIE);     // Enter LPM3, enable interrupt

    while(1) //Do this when you want an infinite loop of code
    {
        /*if (counter++ >= 550 && alarmActive) {
            alarmActive = decrement_timer(&alarmTimer);
            counter = 0;
        }*/
        showChar(alarmTimer.cHours[0],pos1);
        showChar(alarmTimer.cHours[1],pos2);
        showChar(alarmTimer.cMinutes[0],pos3);
        showChar(alarmTimer.cMinutes[1],pos4);
        showChar(alarmTimer.cSeconds[0],pos5);
        showChar(alarmTimer.cSeconds[1],pos6);

        int j;
        for (j = 0; j < 4; ++j) {
            if (rooms[j].armed != 2) {
                if (compare_times(alarmTimer, rooms[j].armTime)) {
                    rooms[j].armed = 1;

                }
                if (compare_times(alarmTimer, rooms[j].disarmTime)) {
                    rooms[j].armed = 0;
                }

                if (rooms[j].armed == 1) { //Read sensors if room is armed
                    if (rooms[j].s0)
                        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
                    else
                        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
                    if (rooms[j].s0)
                        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0);
                    else
                        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0);

                    __delay_cycles(100);

                    if ((GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN3) == 1)) //If reed switch is on for this room
                    {
                        rooms[j].armed = 2;
                    }

                    if (j == 0) { //dealing with mic input from room 0
                        if (ADCState == 0) {
                            if (ADCResult > 530) {
                                rooms[j].armed = 2;
                            }
                            ADCState = 1; //Set flag to indicate ADC is busy - ADC ISR (interrupt) will clear it
                            ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);
                        }
                    }
                }
            }

            if (rooms[j].armed == 0) {
                GPIO_setOutputLowOnPin (rooms[j].led_port, rooms[j].led_pin);
            }

            else if (rooms[j].armed == 1) {
                GPIO_setOutputHighOnPin (rooms[j].led_port, rooms[j].led_pin);
            }

            else if (rooms[j].armed == 2) {
                if (rooms[j].flash_delay++ >= 150) {
                    if (rooms[j].flash_switch) {
                        GPIO_setOutputHighOnPin (rooms[j].led_port, rooms[j].led_pin);
                        rooms[j].flash_switch = 0;
                    }

                    else {
                        GPIO_setOutputLowOnPin (rooms[j].led_port, rooms[j].led_pin);
                        rooms[j].flash_switch = 1;
                    }
                    rooms[j].flash_delay = 0;
                }
                if (magnetState == 0) {
                    Timer_A_outputPWM(TIMER_A0_BASE, &param);   //Turn on PWM
                    magnetState = 1;
                }
            }
        }
    }
}
//        //Buttons SW1 and SW2 are active low (1 until pressed, then 0)
//        if ((GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN3) == 1) & (magnetState == 0)) //Look for rising edge
//        {
//            Timer_A_outputPWM(TIMER_A0_BASE, &param);   //Turn on PWM
//            magnetState = 1;                //Capture new button state
//        }
//        if ((GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN3) == 0) & (magnetState == 1)) //Look for falling edge
//        {
//            Timer_A_stop(TIMER_A0_BASE);                //Shut off PWM signal
//            magnetState = 0;                            //Capture new button state
//        }
//
//        //Start an ADC conversion (if it's not busy) in Single-Channel, Single Conversion Mode
//        if (ADCState == 0)
//        {
//            if (ADCResult > 900) {
//                led_state = 1;
//            }
//            else {
//                led_state = 0;
//            }
//            ADCState = 1; //Set flag to indicate ADC is busy - ADC ISR (interrupt) will clear it
//            ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);
//        }
//        if (led_state) {
//            GPIO_setOutputHighOnPin (GPIO_PORT_P2, GPIO_PIN7);
//        }
//        else {
//            GPIO_setOutputLowOnPin (GPIO_PORT_P2, GPIO_PIN7);
//        }
//    }

    /*
     * You can use the following code if you plan on only using interrupts
     * to handle all your system events since you don't need any infinite loop of code.
     *
     * //Enter LPM0 - interrupts only
     * __bis_SR_register(LPM0_bits);
     * //For debugger to let it know that you meant for there to be no more code
     * __no_operation();
    */

int compare_times(timer timer1, timer timer2) {
    if (timer1.cHours[0] == timer2.cHours[0] &&
            timer1.cHours[1] == timer2.cHours[1] &&
            timer1.cMinutes[0] == timer2.cMinutes[0] &&
            timer1.cMinutes[1] == timer2.cMinutes[1] &&
            timer1.cSeconds[1] == timer2.cSeconds[1] &&
            timer1.cSeconds[0] == timer2.cSeconds[0])
        return 1;
    else
        return 0;
}

int decrement_timer(timer *alarmTimer) {
//    sprintf(alarmTimer->cSeconds, "%ld", alarmTimer->seconds);
//    sprintf(alarmTimer->cMinutes, "%ld", alarmTimer->minutes);

    alarmTimer->cSeconds[1]++;

    if (alarmTimer->cSeconds[1] == ':') {
        alarmTimer->cSeconds[1] = '0';
        alarmTimer->cSeconds[0]++;
    }
    if (alarmTimer->cSeconds[0] == '6') {
        alarmTimer->cSeconds[0] = '0';
        alarmTimer->cMinutes[1]++;
    }
    if (alarmTimer->cMinutes[1] == ':') {
        alarmTimer->cMinutes[1] = '0';
        alarmTimer->cMinutes[0]++;
    }
    if (alarmTimer->cMinutes[0] == '6') {
        alarmTimer->cMinutes[0] = '0';
        alarmTimer->cHours[1]++;
    }
    if (alarmTimer->cHours[1] == ':') {
        alarmTimer->cHours[1] = '0';
        alarmTimer->cHours[0]++;
    }
    if (alarmTimer->cHours[0] == '2' && alarmTimer->cHours[1] == '4') {
        alarmTimer->cHours[0] = '0';
        alarmTimer->cHours[1] = '0';
        alarmTimer->cMinutes[0] = '0';
        alarmTimer->cMinutes[1] = '0';
        alarmTimer->cSeconds[0] = '0';
        alarmTimer->cSeconds[1] = '0';
    }
    return 1;
}

void Init_GPIO(void)
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);
    //GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN5);
    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN3);

    //Set LED1 and LED2 as outputs
    //GPIO_setAsOutputPin(LED1_PORT, LED1_PIN); //Comment if using UART
    GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);
}

/* Clock System Initialization */
void Init_Clock(void)
{
    /*
     * The MSP430 has a number of different on-chip clocks. You can read about it in
     * the section of the Family User Guide regarding the Clock System ('cs.h' in the
     * driverlib).
     */

    /*
     * On the LaunchPad, there is a 32.768 kHz crystal oscillator used as a
     * Real Time Clock (RTC). It is a quartz crystal connected to a circuit that
     * resonates it. Since the frequency is a power of two, you can use the signal
     * to drive a counter, and you know that the bits represent binary fractions
     * of one second. You can then have the RTC module throw an interrupt based
     * on a 'real time'. E.g., you could have your system sleep until every
     * 100 ms when it wakes up and checks the status of a sensor. Or, you could
     * sample the ADC once per second.
     */
    //Set P4.1 and P4.2 as Primary Module Function Input, XT_LF
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768);
    // Set ACLK = XT1
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Initializes the XT1 crystal oscillator
    CS_turnOnXT1LF(CS_XT1_DRIVE_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
}

unsigned int output = 0;
unsigned int timerCounter = 0;



/* UART Initialization */
void Init_UART(void)
{
    /* UART: It configures P1.0 and P1.1 to be connected internally to the
     * eSCSI module, which is a serial communications module, and places it
     * in UART mode. This let's you communicate with the PC via a software
     * COM port over the USB cable. You can use a console program, like PuTTY,
     * to type to your LaunchPad. The code in this sample just echos back
     * whatever character was received.
     */

    //Configure UART pins, which maps them to a COM port over the USB cable
    //Set P1.0 and P1.1 as Secondary Module Function Input.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

    /*
     * UART Configuration Parameter. These are the configuration parameters to
     * make the eUSCI A UART module to operate with a 9600 baud rate. These
     * values were calculated using the online calculator that TI provides at:
     * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
     */

    //SMCLK = 1MHz, Baudrate = 9600
    //UCBRx = 6, UCBRFx = 8, UCBRSx = 17, UCOS16 = 1
    EUSCI_A_UART_initParam param = {0};
        param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
        param.clockPrescalar    = 6;
        param.firstModReg       = 8;
        param.secondModReg      = 17;
        param.parity            = EUSCI_A_UART_NO_PARITY;
        param.msborLsbFirst     = EUSCI_A_UART_LSB_FIRST;
        param.numberofStopBits  = EUSCI_A_UART_ONE_STOP_BIT;
        param.uartMode          = EUSCI_A_UART_MODE;
        param.overSampling      = 1;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable EUSCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    if(!output){
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\n');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\r');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'E');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'n');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 't');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'r');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'c');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'u');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'r');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'r');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'n');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 't');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 't');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'i');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'm');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\n');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\r');
        output = 1;
    }

}


/* EUSCI A0 UART ISR - Echoes data back to PC host */
#pragma vector=USCI_A0_VECTOR
__interrupt
void EUSCIA0_ISR(void)
{
    uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);

    if (RxStatus)
    {

        /*alarmTimer.cHours[0] = '1';
        alarmTimer.cHours[1] = '1';
        alarmTimer.cMinutes[0] = '0';
        alarmTimer.cMinutes[1] = '0';
        alarmTimer.cSeconds[0] = '0';
        alarmTimer.cSeconds[1] = '0';*/

        uint8_t value = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);

        // Arm zone 1
        if(((char)value >= '0' && (char)value <= '9') && timerCounter < 2){
            alarmTimer.cHours[timerCounter] = (char)value;
            timerCounter++;
        }
        if((char)value == 'h' && timerCounter == 2){
            timerCounter++;
        }

        if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 2 && timerCounter < 5)){
            alarmTimer.cMinutes[timerCounter - 3] = (char)value;
            timerCounter++;
        }
        if((char)value == 'm' && timerCounter == 5){
            timerCounter++;
        }

        if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 5 && timerCounter < 8)){
            alarmTimer.cSeconds[timerCounter-6] = (char)value;
            timerCounter++;
        }
        if((char)value == 's' && timerCounter == 8){
            timerCounter++;
        }

        // Arm zone 1
        if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 8 && timerCounter < 11)){
            rooms[0].armTime.cHours[timerCounter-9] = (char)value;
            timerCounter++;
        }
        if((char)value == 'h' && timerCounter == 11){
            timerCounter++;
        }

        if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 11 && timerCounter < 14)){
            rooms[0].armTime.cMinutes[timerCounter-12] = (char)value;
            timerCounter++;
        }
        if((char)value == 'm' && timerCounter == 14){
            timerCounter++;
        }

        if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 14 && timerCounter < 17)){
            rooms[0].armTime.cSeconds[timerCounter-15] = (char)value;
            timerCounter++;
        }
        if((char)value == 's' && timerCounter == 17){
            timerCounter++;
        }

        // Disarm zone 1
        if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 17 && timerCounter < 20)){
            rooms[0].disarmTime.cHours[timerCounter-18] = (char)value;
            timerCounter++;
        }
        if((char)value == 'h' && timerCounter == 20){
            timerCounter++;
        }

        if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 20 && timerCounter < 23)){
            rooms[0].disarmTime.cMinutes[timerCounter-21] = (char)value;
            timerCounter++;
        }
        if((char)value == 'm' && timerCounter == 23){
            timerCounter++;
        }

        if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 23 && timerCounter < 26)){
            rooms[0].disarmTime.cSeconds[timerCounter-24] = (char)value;
            timerCounter++;
        }
        if((char)value == 's' && timerCounter == 26){
            timerCounter++;
        }
        // Arm zone 2
        if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 26 && timerCounter < 29)){
            rooms[1].armTime.cHours[timerCounter-27] = (char)value;
            timerCounter++;
        }
        if((char)value == 'h' && timerCounter == 29){
            timerCounter++;
        }

        if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 29 && timerCounter < 32)){
            rooms[1].armTime.cMinutes[timerCounter-30] = (char)value;
            timerCounter++;
        }
        if((char)value == 'm' && timerCounter == 32){
            timerCounter++;
        }

        if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 32 && timerCounter < 35)){
            rooms[1].armTime.cSeconds[timerCounter-33] = (char)value;
            timerCounter++;
        }
        if((char)value == 's' && timerCounter == 35){
            timerCounter++;
        }

        // Disarm zone 2
        if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 35 && timerCounter < 38)){
               rooms[1].disarmTime.cHours[timerCounter-36] = (char)value;
               timerCounter++;
           }
           if((char)value == 'h' && timerCounter == 38){
               timerCounter++;
           }

           if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 38 && timerCounter < 41)){
               rooms[1].disarmTime.cMinutes[timerCounter-39] = (char)value;
               timerCounter++;
           }
           if((char)value == 'm' && timerCounter == 41){
               timerCounter++;
           }

           if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 41 && timerCounter < 44)){
               rooms[1].disarmTime.cSeconds[timerCounter-42] = (char)value;
               timerCounter++;
           }
           if((char)value == 's' && timerCounter == 44){
               timerCounter++;
           }
          // Arm zone 3
          if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 44 && timerCounter < 47)){
              rooms[2].armTime.cHours[timerCounter-45] = (char)value;
              timerCounter++;
          }
          if((char)value == 'h' && timerCounter == 47){
              timerCounter++;
          }

          if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 47 && timerCounter < 50)){
              rooms[2].armTime.cMinutes[timerCounter-48] = (char)value;
              timerCounter++;
          }
          if((char)value == 'm' && timerCounter == 50){
              timerCounter++;
          }

          if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 50 && timerCounter < 53)){
              rooms[2].armTime.cSeconds[timerCounter-51] = (char)value;
              timerCounter++;
          }
          if((char)value == 's' && timerCounter == 53){
              timerCounter++;
          }
          // Disarm zone 3
          if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 53 && timerCounter < 56)){
             rooms[2].disarmTime.cHours[timerCounter-54] = (char)value;
             timerCounter++;
          }
          if((char)value == 'h' && timerCounter == 56){
              timerCounter++;
          }

          if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 56 && timerCounter < 59)){
              rooms[2].disarmTime.cMinutes[timerCounter-57] = (char)value;
              timerCounter++;
          }
          if((char)value == 'm' && timerCounter == 59){
              timerCounter++;
          }

          if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 59 && timerCounter < 62)){
              rooms[2].disarmTime.cSeconds[timerCounter-60] = (char)value;
              timerCounter++;
          }
          if((char)value == 's' && timerCounter == 62){
              timerCounter++;
          }
          // Arm zone 4
          if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 62 && timerCounter < 65)){
              rooms[3].armTime.cHours[timerCounter-63] = (char)value;
              timerCounter++;
          }
          if((char)value == 'h' && timerCounter == 65){
              timerCounter++;
          }

          if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 65 && timerCounter < 68)){
              rooms[3].armTime.cMinutes[timerCounter-66] = (char)value;
              timerCounter++;
          }
          if((char)value == 'm' && timerCounter == 68){
              timerCounter++;
          }

          if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 68 && timerCounter < 71)){
              rooms[3].armTime.cSeconds[timerCounter-69] = (char)value;
              timerCounter++;
          }
          if((char)value == 's' && timerCounter == 71){
              timerCounter++;
          }
          // Disarm zone 4
          if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 71 && timerCounter < 74)){
              rooms[3].disarmTime.cHours[timerCounter-72] = (char)value;
              timerCounter++;
          }
          if((char)value == 'h' && timerCounter == 74){
              timerCounter++;
          }

          if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 74 && timerCounter < 77)){
              rooms[3].disarmTime.cMinutes[timerCounter-75] = (char)value;
              timerCounter++;
          }
          if((char)value == 'm' && timerCounter == 77){
              timerCounter++;
          }

          if(((char)value >= '0' && (char)value <= '9') && (timerCounter > 77 && timerCounter < 80)){
              rooms[3].disarmTime.cSeconds[timerCounter-78] = (char)value;
              timerCounter++;
          }
          if((char)value == 's' && timerCounter == 80){
              timerCounter++;
              alarmActive = 1;
          }



        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, value);



        if((char)value == 's' && timerCounter == 9){
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'E');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 't');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'a');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'm');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 't');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'i');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'm');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'z');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'o');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '1');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\r');
        }
        if((char)value == 's' && timerCounter == 18){
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'E');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 't');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'd');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'i');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 's');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'a');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'm');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 't');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'i');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'm');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'z');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'o');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '1');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\r');
        }
        if((char)value == 's' && timerCounter == 27){
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'E');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 't');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'a');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'm');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 't');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'i');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'm');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'z');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'o');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '2');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\r');
        }
        if((char)value == 's' && timerCounter == 36){
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'E');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 't');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'd');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'i');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 's');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'a');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'm');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 't');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'i');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'm');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'z');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'o');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '2');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\r');
        }
        if((char)value == 's' && timerCounter == 45){
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'E');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 't');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'a');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'm');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 't');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'i');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'm');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'z');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'o');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '3');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\r');
        }
        if((char)value == 's' && timerCounter == 54){
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'E');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 't');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'd');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'i');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 's');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'a');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'm');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 't');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'i');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'm');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'z');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'o');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '3');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\r');
        }
        if((char)value == 's' && timerCounter == 63){
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'E');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 't');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'a');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'm');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 't');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'i');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'm');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'z');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'o');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '4');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\r');
        }
        if((char)value == 's' && timerCounter == 72){
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'E');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 't');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'd');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'i');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 's');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'a');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'm');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 't');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'i');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'm');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'z');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'o');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '4');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\r');
        }
        if((char)value == 's' && timerCounter == 81){
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\r');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'S');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 't');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'u');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'p');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, ' ');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'c');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'o');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'm');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'p');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'l');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 't');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'e');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\n');
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '\r');
        }
    }
}

/* PWM Initialization */
void Init_PWM(void)
{
    /*
     * The internal timers (TIMER_A) can auto-generate a PWM signal without needing to
     * flip an output bit every cycle in software. The catch is that it limits which
     * pins you can use to output the signal, whereas manually flipping an output bit
     * means it can be on any GPIO. This function populates a data structure that tells
     * the API to use the timer as a hardware-generated PWM source.
     *
     */
    //Generate PWM - Timer runs in Up-Down mode
    param.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod           = TIMER_A_PERIOD; //Defined in main.h
    param.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle             = HIGH_COUNT; //Defined in main.h

    //PWM_PORT PWM_PIN (defined in main.h) as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, PWM_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
}

void Init_ADC(void)
{
    /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */

    //Set ADC_IN to input direction
    GPIO_setAsPeripheralModuleFunctionInputPin(ADC_IN_PORT, ADC_IN_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    //Initialize the ADC Module
    /*
     * Base Address for the ADC Module
     * Use internal ADC bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC_init(ADC_BASE,
             ADC_SAMPLEHOLDSOURCE_SC,
             ADC_CLOCKSOURCE_ADCOSC,
             ADC_CLOCKDIVIDER_1);

    ADC_enable(ADC_BASE);

    /*
     * Base Address for the ADC Module
     * Sample/hold for 16 clock cycles
     * Do not enable Multiple Sampling
     */
    ADC_setupSamplingTimer(ADC_BASE,
                           ADC_CYCLEHOLD_16_CYCLES,
                           ADC_MULTIPLESAMPLESDISABLE);

    //Configure Memory Buffer
    /*
     * Base Address for the ADC Module
     * Use input ADC_IN_CHANNEL
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */
    ADC_configureMemory(ADC_BASE,
                        ADC_IN_CHANNEL,
                        ADC_VREFPOS_AVCC,
                        ADC_VREFNEG_AVSS);

    ADC_clearInterrupt(ADC_BASE,
                       ADC_COMPLETED_INTERRUPT);

    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE,
                        ADC_COMPLETED_INTERRUPT);
}

//ADC interrupt service routine
#pragma vector=ADC_VECTOR
__interrupt
void ADC_ISR(void)
{
    uint8_t ADCStatus = ADC_getInterruptStatus(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);

    ADC_clearInterrupt(ADC_BASE, ADCStatus);

    if (ADCStatus)
    {
        ADCState = 0; //Not busy anymore
        ADCResult = ADC_getResults(ADC_BASE);
    }
}

// RTC interrupt service routine
#pragma vector=RTC_VECTOR
__interrupt void RTC_ISR(void)
{
         if(RTCIV & RTCIV_RTCIF)    {                  // RTC Overflow
             if(alarmActive)
                 decrement_timer(&alarmTimer);
         }
}
