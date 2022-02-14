/****************************************
* ME 6705 - Lab 11
* Motor Control
* Description: Line Following Robot
  MSP432
* Author: Cody Houff
****************************************/

/* DriverLib Includes */
#include "driverlib.h"
#include "stdio.h"

#define TIMER_A_PERIOD 10000

int i = 0;
int j = 0;
int m = 0;
int n = 0;

int run_program = 0;
int button1 = 0;
int s1,s2,s3,s4,s5,s6,s7,s8 = 0;


float left_motor_speed = 0;
float right_motor_speed = 0;
float base_speed = 5; // 5% speed
float max_speed = 10; // 10% speed

float val = 0;
float lastval = 0;
float sum = 0;
float weighted_val = 0;
float kp_error = 0;
float kd_error = 0;


// PID control
float Kp = .007;
float Ki = 0.00008;
float Kd = .06;
float u = 0;

float error = 0;
float error_dot = 0;
float error_int = 0;
float error_prev = 0;



const Timer_A_UpModeConfig upConfig_0 =  // Configure counter in Up mode
{   TIMER_A_CLOCKSOURCE_SMCLK,        // Tie Timer A to SMCLK
    TIMER_A_CLOCKSOURCE_DIVIDER_48,   // Increment counter every 48 clock cycles
    1250,                            //3MHz /64 = 46875     3MHz/48 = 62,500 Hz       1250/62,500 = .02 s or 50Hz
    TIMER_A_TAIE_INTERRUPT_DISABLE,         // Enable Timer interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,    // Enable CCR0 interrupt
    TIMER_A_DO_CLEAR                     // Clear counter upon initialization
};


//2.4 and 2.5
const Timer_A_PWMConfig pwmConfig1=
{
TIMER_A_CLOCKSOURCE_SMCLK,
TIMER_A_CLOCKSOURCE_DIVIDER_1,
TIMER_A_PERIOD,
TIMER_A_CAPTURECOMPARE_REGISTER_1,
TIMER_A_OUTPUTMODE_RESET_SET,
1000
};

//2.6 and 2.7
const Timer_A_PWMConfig pwmConfig2=
{
TIMER_A_CLOCKSOURCE_SMCLK,
TIMER_A_CLOCKSOURCE_DIVIDER_1,
TIMER_A_PERIOD,
TIMER_A_CAPTURECOMPARE_REGISTER_3,
TIMER_A_OUTPUTMODE_RESET_SET,
1000
};


void main(void)
{
    // Halting WDT
    MAP_WDT_A_holdTimer();

    // Set LEDs at output pins and set their initial state to off
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);     // Set LED1 as an output pin
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0); // Set LED1 initial state to off
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0); // Set LED2.0 as an output pin
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0); // Set LED2.0 initial state to off
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1); // Set LED2.1 as an output pin
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1); // Set LED2.1 initial state to off
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2); // Set LED2.2 as an output pin
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2); // Set LED2.2 initial state to off


    // Set switch 1 and switch 2 as input button (connected to P1.1 and P1.4)
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1); // use S1 as an input with a pull up resistor
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4); // use S2 as an input with a pull up resistor

    // Set up clock signals
    unsigned int dcoFrequency = 3E+6;
    MAP_CS_setDCOFrequency(dcoFrequency);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    //Set GPIO Output pin  P2.4 and P2.5
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION); // Set GPIO pin  P2.4
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION); // Set GPIO pin P2.5
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION); // Set GPIO pin  P2.6
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION); // Set GPIO pin P2.7
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);

    //PWM signal
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig1);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig2);


    /*** Timer1_A Set-Up ***/
    TA0CCR1 = 0; //duty cycle 2.4
    TA0CCR2 = 0; //duty cycle 2.5
    TA0CCR3 = 0; //duty cycle 2.6
    TA0CCR4 = 0; //duty cycle 2.7

    TA0CCTL1 = OUTMOD_7;
    TA0CCTL2 = OUTMOD_7;
    TA0CCTL3 = OUTMOD_7;
    TA0CCTL4 = OUTMOD_7;
    //TA0CTL = TASSEL__SMCLK | MC__UP | TACLR;


    // Infinite loop
    while (1)
    {


        if (run_program == 1)
        {
            //printf("run_program = 1 \n");

            //MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0); // Set LED1 state to on
            TA0CCR1 = 0;
            //TA0CCR2 = 10000;
            //TA0CCR3 = 10000;
            TA0CCR4 = 0;

            //TA0.0 interrupt
            MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig_0);  // Configure Timer A using above struct
            MAP_Interrupt_enableInterrupt(INT_TA0_0);
            MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);  // Start Timer A0

            /* Enabling interrupts and going to sleep */
            MAP_Interrupt_enableSleepOnIsrExit();

            MAP_Interrupt_enableMaster();
        }
        else if (run_program == 0)
        {
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0); // Set LED1 state to off
            TA0CCR1 = 0;
            TA0CCR2 = 0;
            TA0CCR3 = 0;
            TA0CCR4 = 0;
        }


        if (MAP_GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) == GPIO_INPUT_PIN_LOW) //if switch 1 is pressed
        {
            printf("button 1: pressed if \n");
            run_program = 1;
            for(i=1; i<=30000;i++){}
            //printf("run_program = 1 \n");
        }
        else if (MAP_GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4) == GPIO_INPUT_PIN_LOW) //if switch 2 is pressed
        {
            printf("button 2: pressed if \n");
            run_program = 0;
            for(i=1; i<=30000;i++){}
            //printf("run_program = 0 \n");
        }
    }
}



void TA0_0_IRQHandler(void)
{
    //printf("TA0_0_IRQHandler \n");
    MAP_GPIO_toggleOutputOnPin( GPIO_PORT_P1, GPIO_PIN0);
    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    //Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
    Timer_A_clearInterruptFlag(TIMER_A0_BASE);

    //MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P6, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION); //P6.7 sensor 1

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P6,GPIO_PIN7);      //sensor 8, P6.7
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN7);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2,GPIO_PIN3);      //sensor 7, P2.3
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN3);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5,GPIO_PIN1);      //sensor 6, P5.1
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN1);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN5);      //sensor 5, P3.5
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5);

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN6);      //sensor 4, P1.6
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN7);      //sensor 3, P1.7
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5,GPIO_PIN0);      //sensor 2, P5.0
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5,GPIO_PIN2);      //sensor 1, P5.2
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);

    for(i=1; i<=3;i++){};             // 3/3,000,000  1us wide
    MAP_GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN7); //sensor 8, P6.7
    MAP_GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN3); //sensor 7, P2.3
    MAP_GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN1); //sensor 6, P5.1
    MAP_GPIO_setAsInputPin(GPIO_PORT_P3, GPIO_PIN5); //sensor 5, P3.5
    MAP_GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN6); //sensor 4, P1.6
    MAP_GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN7); //sensor 3, P1.7
    MAP_GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN0); //sensor 2, P5.0
    MAP_GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN2); //sensor 1, P5.2
    for(i=1; i<=300;i++){};         // 300/3,000,000  .1ms wide

    s8 = MAP_GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN7);
    s7 = MAP_GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN3);
    s6 = MAP_GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN1);
    s5 = MAP_GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN5);
    s4 = MAP_GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6);
    s3 = MAP_GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7);
    s2 = MAP_GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN0);
    s1 = MAP_GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN2);


    //from left to right. Sensor output is high when white: 0, black = 1

    weighted_val = (s1*0)+(s2*1000)+(s3*2000)+(s4*3000)+(s5*4000)+(s6*5000)+(s7*6000)+(s8*7000);
    sum = (s1+s2+s3+s4+s5+s6+s7+s8);

    if(sum>0)
    {
        val = weighted_val/sum;
    }
    else if(sum == 0)
    {
        val = lastval;
    }
    lastval = val;

    //PID Control
    error = 3500-val; //Kp error
    error_int = error_int + error; //Ki error
    error_dot = error-error_prev; //Kd error
    error_prev = error;

    kp_error = Kp*error;
    kd_error = Kd*error_dot;

    u = Kp*error + Ki*error_int + Kd*error_dot;


    left_motor_speed = base_speed-u;
    if(left_motor_speed >max_speed)
    {
        left_motor_speed = max_speed;
    }
    else if(left_motor_speed <0)
    {
        left_motor_speed = 0;
    }

    right_motor_speed = base_speed + u;
    if(right_motor_speed >max_speed)
    {
        right_motor_speed = max_speed;
    }
    else if(right_motor_speed <0)
    {
        right_motor_speed = 0;
    }


    else if (sum == 8) //all black
    {
        left_motor_speed = 0;
        right_motor_speed = 0;
    }


    TA0CCR2 = left_motor_speed*100; //duty cycle from 0 to max_speed or 10000
    TA0CCR3 = right_motor_speed*100;

    //printf("[%u] [%u] [%u] [%u] [%u] [%u] [%u] [%u]\n",s1,s2,s3,s4,s5,s6,s7,s8);
    //printf("[%f] [%f] [%f] [%f] [%f]\n",u,kp_error,kd_error,left_motor_speed,right_motor_speed);

    m++;
}

