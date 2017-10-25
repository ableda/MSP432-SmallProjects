/*  Created by Alex Bleda Vilalta
    EC450 2/25/2017

    In order to play a song through the MSP432 buzzer an array of notes and tempo is
    required. An example is the BellaCiao and moonlight songs played here.
*/

/* DriverLib Includes */
#include </Users/alexbleda/ti/devices/msp432p4xx/driverlib/driverlib.h>
/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// PORTS -- as needed by driverlib calls
#define LED_PORT GPIO_PORT_P2
#define LED_PIN  GPIO_PIN4

// Define half periods for each note
// Based in the formula: half_period = 1000000/(2*frequency)
#define A5  568
#define A4  1136
#define A3  2272
#define Ab3 2403
#define B5  506
#define B3  2040
#define Bb  2145
#define C4  1915
#define C3  3816
#define Cs  3596
#define D4  1700
#define D3  3400
#define Ds  3214
#define F4  1428
#define F3  2856
#define E4  1515
#define Es  1607
#define Eb  3400
#define E3  3030
#define G4  1275
#define Gs  1205
#define G3  2550
#define pause 0

//Lengths of song arrays
#define bc_length 44
#define Kanye_Length 62
#define moon    79

// Global Arrays to store musical score
unsigned const int Bella_Ciao[bc_length] = {E3,A3,B3,C4,A3,pause,E3,A3,B3,C4,A3,pause,pause,A3,B3,C4,B3,A3,C4,B3,A3,E4,E4,E4,E4,
                                            D4,E4,F4,F4,pause,F4,E4,D4,F4,E4,pause,E4,D4,C4,B3,E4,B3,C4,A4};

unsigned const char bella_length[bc_length] =  {1,1,1,1,4,1,1,1,1,1,4,1,1,1,1,2,1,1,2,1,1,2,2,1,1,1,1,1,4,1,1,1,1,1,4,1,1,1,1,2,2,2,2,14};


unsigned const int moonlight[moon] = {A3, D3, F3, A3, D3, F3,A3, D3, F3,A3, D3, F3,A3, D3, F3,A3, D3, F3,A3, D3, F3,A3, D3, F3, Bb, D3,F3,Bb,D3,F3, Bb,Eb,G3, Bb,Eb,G3, A3,Cs,G3, A3,
                                        D3, F3, A3, Cs, G3, A3, D3,F3, A3, D3,E3, G3,Cs,E3, F3,A3,D3, A3,D3,F3, A3,D3,F3, A3,D3,F3, A3,E3,G3, A3,E3,G3, A3,E3,G3, A3,E3,G3, pause};
unsigned const int moon_length[moon] = {1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1, 1,1,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,4};

// half period for a 0 (don't output anything until song starts)
#define initialHalfPeriod 0

// declare functions defined below
void mapports();         // connect TACCR0 to P2.7 using PMAP
void init_timer(void);   // routine to setup the timer
void init_buttons(void);  // routine to setup the button


// Global Variables
volatile unsigned int state = 0;
volatile unsigned int counter = 0;
volatile unsigned int note_counter = 0;
volatile unsigned int song_counter = 0;
volatile unsigned int speed = 1;

int main(void)
{
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();

    // Initialize timer and buttons
    mapports();
    init_timer();
    init_buttons();

    // Initalize ACLK to VLO/32 around 294Hz
    MAP_CS_initClockSignal(CS_ACLK, CS_VLOCLK_SELECT, CS_CLOCK_DIVIDER_32);
    //Initialize Watchdog timer to interval mode, to stop every 64/294 or around 0.21 seconds, the closest value to 1/16th of a note given that the
    // bpm for an 1/8th note was 60/144 = 0.42
    MAP_WDT_A_initIntervalTimer(WDT_A_CLOCKSOURCE_ACLK, WDT_A_CLOCKITERATIONS_64);

    // Setup the LED output pin
    MAP_GPIO_setAsOutputPin(LED_PORT, LED_PIN);
    MAP_GPIO_setOutputLowOnPin(LED_PORT, LED_PIN);

    // setup NVIC and Interrupts
    MAP_Interrupt_disableSleepOnIsrExit();   // Specify that after an interrupt, the CPU wakes up

    MAP_Interrupt_enableMaster();// unmask IRQ interrupts to allow the CPU to respond.
    MAP_Interrupt_enableInterrupt(INT_PORT5); // for the button, using static interrupt
    MAP_Interrupt_enableInterrupt(INT_PORT3);
    MAP_Interrupt_enableInterrupt(INT_PORT4);
    MAP_Interrupt_enableInterrupt(INT_WDT_A); // For watchdog timer

    // Start the watchdog interval timer
    MAP_WDT_A_startTimer();

    while(1)
    {
        /* Go to LPM0 mode (Low power mode with CPU powered off */
        MAP_PCM_gotoLPM0();       //
        __no_operation(); //  For debugger
    }
}

/*
 * PORT MAPPING
 * We want the output from TA0.0 to go to P2.7
 * This can be done through the portmapper since P2.7 is reconfigurable
 *
 * In general, the portmap unit PMAP has to be unlocked to be configured
 * By default it can be unlocked only once after a hardware reset.
 *
 */

/* Port mapper configuration register */
const uint8_t port_mapping[] =
{
        //Port P2:
        PM_NONE, PM_NONE, PM_NONE, PM_NONE, PM_NONE, PM_NONE, PM_NONE,
        PM_TA0CCR0A
};

void mapports(){  // With driver lib, this is a single call (given the data structure above)
    MAP_PMAP_configurePorts(port_mapping, PMAP_P2MAP, 1,
            PMAP_DISABLE_RECONFIGURATION);
}

/*
 * Sound Production System
 * In this example, we simply run TA0 in up mode
 * (TA0R goes from 0 to TA0CCR0 and then back to 0)
 * And when we want the sound on, we activate output mode 4 (toggle),
 * which makes the CCR0 output signal toggle each time TA0R hits TA0CCR0's value.
 * Thus, the half-period of the tone is TACCR0+1 timer ticks.
 *
 * The timer clock source is SMCLK (3Mhz), prescaled by 1, then divided by 3.
 * Thus it is about 1MHz.
 * Note: possible prescale factors are 1,2,4,8.
 *       possible additional EX0 dividers are 1,2,3,4,5,6,7,8
 *
 */

/* Timer_A UpMode Configuration Parameter */
const Timer_A_UpModeConfig upConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_3,          // SMCLK/3 = 1MHz
        initialHalfPeriod,                      // 500 tick period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE ,   // Disable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};

const Timer_A_CompareModeConfig compareConfig ={
        TIMER_A_CAPTURECOMPARE_REGISTER_0,
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
        TIMER_A_OUTPUTMODE_TOGGLE,
        initialHalfPeriod
};

void init_timer(){              // initialization and start of timer
    /* Configuring Timer_A1 for Up Mode */
    MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig);
    MAP_Timer_A_initCompare(TIMER_A0_BASE, &compareConfig);

    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN7,GPIO_PRIMARY_MODULE_FUNCTION);
    P2->SEL0|=BIT7; // connect timer output to pin (select alternate function for pin)
    P2->DIR |=BIT7; // output mode on P2.7 (direction output completes setting the function)
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE); // start TA0 in up mode
}


/* WDT ISR - This ISR interrupts every 1/8th of a note*/
void WDT_A_IRQHandler(void)
{
    // IF state is initial song play Bella Ciao
    if (state == 0){
        // variable to change tone speed
        volatile int tone_speed = 0;

        if (speed == 0){
            if (bella_length[song_counter] == 1)
                tone_speed = 0;
            else
                tone_speed = (bella_length[song_counter])/2;
        }
        else
            tone_speed = bella_length[song_counter] * speed;

        // This calculates the amount of 1/16ths for every note
        if( note_counter >= tone_speed ){
            note_counter = 0;
            song_counter++;
            TIMER_A0->CCR[0] = Bella_Ciao[song_counter];
        }
        else
            note_counter++;

        // If the song is over restart
        if (song_counter >= bc_length)
            song_counter = 0;
    }

    // Else play personalized song
    else if (state == 1){
        volatile int tone_speed = 0;

        if (speed == 0){
            if (moon_length[song_counter] == 1)
                tone_speed = 0;
            else
                tone_speed = (moon_length[song_counter])/2;
        }
        else
            tone_speed = moon_length[song_counter] * speed;

        if( note_counter >= tone_speed){
            note_counter = 0;
            song_counter++;
            TIMER_A0->CCR[0] = moonlight[song_counter];
        }
        else
            note_counter++;

        if (song_counter >= moon)
            song_counter = 0;
    }
}

/*
 * Button input System
 *
 * The Button toggles the state of the sound (on or off)
 * Action will be interrupt driven on a down going signal on the pin
 * no debouncing in this code, but MSP432P401R has glitch suppression which may help.
 */

void PORT5_IRQHandler(){
// Good practice to check which of the 8 pins caused the interrupt.
// In general, we would take different actions for interrupts on different
// input pins, and this handler would be called for any of them.
// In this program, there should be only the one interrupt.
    if (MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5) & GPIO_PIN1){ // check that it is the button interrupt flag
        MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5,GPIO_PIN1); // clear the flag to allow for another interrupt later.
        // This handler changes the state of the timer CCTL0 control register!
        // Toggle OUTMOD between
        //    mode 0: output = constant value determined by CCTL0 bit 2, and
        //    mode 4: toggle, which when repeated produces a square wave.
        TIMER_A0->CCTL[0]^=TIMER_A_CCTLN_OUTMOD_4; // done using register style ==> just toggle OUTMOD bits
        MAP_GPIO_setOutputLowOnPin(LED_PORT,LED_PIN);
    }
}

void PORT3_IRQHandler(){
    // This button changes song
    if (MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P3) & GPIO_PIN5){ // check that it is the button interrupt flag
        MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3,GPIO_PIN5); // clear the flag to allow for another interrupt later.
        if (state == 0){
            state = 1;
        }
        else{
            state = 0;
        }
    }
    note_counter = 0;
    song_counter = 0;
    speed = 1;
}

void PORT4_IRQHandler(){
    // This button changes song speed
    if (MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P4) & GPIO_PIN1){ // check that it is the button interrupt flag
        MAP_GPIO_toggleOutputOnPin(LED_PORT, LED_PIN);

        if (speed == 1){
            speed = 0;
        }
        else{
            speed = 1;
        }
    }
}

void init_buttons(){
// All GPIO's are already inputs if we are coming in after a reset
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5,GPIO_PIN1);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P5,GPIO_PIN1,GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5,GPIO_PIN1);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P5,GPIO_PIN1);

    //initialize second button to change song
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P3,GPIO_PIN5);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P3,GPIO_PIN5,GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3,GPIO_PIN5);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P3,GPIO_PIN5);

    //Initialize button for speed change
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4,GPIO_PIN1);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4,GPIO_PIN5,GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4,GPIO_PIN1);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4,GPIO_PIN1);
}
