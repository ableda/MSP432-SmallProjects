/* Created by Alex Bleda
 * Homework 4 Board-to-Board Communication
 * See attached PDF for design description and assessment of the results.
 *
 * Parameters and Details
 * ======================
 * Connections (from the EDU MKII):
 * ------------------------------
 * Joystick X analog input: P6.0 (A15) [A15 is the default tertiary function of P6.0]
 * Joystick Y analog input: P4.4 (A9)
 * Joystick Button input: P4.1 (not used here, yet)
 *
 * The Crystalfont128x128 display is connected to an SPI communication interface
 * and several gpio pins:
 * LCD SPI CLK:  P1.5 (UCB0CLK)
 * LCD RST:      P5.7
 * LCD SPI MOSI: P1.6 (UCB0SIMO)
 * LCD SPI CS:   P5.0
 * LCD RS PIN:   P3.7
 *
 * The display is managed by a Graphics Library (grlib.h) and
 * an LCD driver for the Crystalfont128x128 display.
 * See the graphics driver library documentation in the MSP432 SDK
 * and the LCD driver code that is part of this project.

 **************************************************************************************/

// Full set of include files including graphics and driver libraries
// and also the LCD driver which is part of the project itself
#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"

/* Standard Includes */
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

/********************************************
 * Global Variables shared between handlers
 ********************************************/
/* ADC results buffer */
uint16_t resultsBuffer[2];           // latest readings from the analog inputs
uint8_t  receive_byte[4];

uint16_t buttonPressed;              // 1 if joystick button is pressed
uint16_t print_flag;                 // flag to signal main to redisplay - set by ADC14
uint16_t firsttime = 1;             // this parameter is initialized and will keep track of whether the x or y coordinate is being received by the system.
uint16_t transmit_order;            //Will help initialize the system, will run once to fill the uart_control
uint8_t uart_control[2];            //will contain incoming data, will be used to designate x, y and the parameter
uint8_t firstint, secondint;        //firstint and secondint denote the 8bit numbers that will be sent to the other msp432

/***************************************************************
 * WDT system
 * 512K cycles / 12MHz = 42.6ms
 * Set the WDT to this time interval to make sure the system has time
 * to transmit each data point before new data is acquired
 ***************************************************************/
void WDT_A_IRQHandler(void) {
    MAP_ADC14_toggleConversionTrigger(); // trigger the next conversion
}

void init_WDT(){//WDT initialization
    MAP_WDT_A_initIntervalTimer(WDT_A_CLOCKSOURCE_SMCLK,WDT_A_CLOCKITERATIONS_512K);  //64, 512, 8192, 32k, 512k, 8129k, 128M cycle options
    MAP_WDT_A_startTimer(); // start the timer
}

/*************************************************
 * ADC14 Subsystem
 *************************************************/
/*
 * ADC Interrupt handler
 * This interrupt is fired whenever a conversion is completed and placed in
 * ADC_MEM1. This signals the end of conversion and the results array is
 * grabbed and placed in resultsBuffer
 */
void ADC14_IRQHandler(void)
{
    uint64_t status;
    status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);

    /* ADC_MEM1 conversion completed */
    if(status & ADC_INT1)
    {
        /* Store ADC14 conversion results */
        resultsBuffer[0] = MAP_ADC14_getResult(ADC_MEM0);
        resultsBuffer[1] = MAP_ADC14_getResult(ADC_MEM1);

        /* Determine if JoyStick button is pressed */
        buttonPressed = (P4IN & GPIO_PIN1)?0:1;

        //Enable UART transmit interrupt after you update x and y values
        MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);

        print_flag=1;  // signal main to refresh the display
    }
}

/* * ADC Setup */
void init_ADC(){
    /* Configure Pin 6.0 (A15) and 4.4 (A9) to be analog inputs ('tertiary function') */
    /* see the port 4 and port 6 pinout details of the MSP432p401r */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Initializing ADC (ADCOSC/64/8)
     * drive from the internal ASD oscillator
     * with predivider 64 and divider 8, no routing to internal pins
     */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8, ADC_NOROUTE);

    /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM1 (A15, A9)  with repeat)
     * Basic operation is sequence mode with
     *   ADC-MEM0 -- A15
     *   ADC_MEM1 -- A9
     *
     *   NO automatic repeats
     */
    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, false); // use MEM...MEM1 channels
    // configure each memory channel:

    MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A15, ADC_NONDIFFERENTIAL_INPUTS);
    MAP_ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A9, ADC_NONDIFFERENTIAL_INPUTS);

    /* Enabling the interrupt when a conversion on channel 1 (end of sequence)
     * is complete and enabling conversions*/
    MAP_ADC14_enableInterrupt(ADC_INT1);

    /* Setting up the sample timer to automatically step through the sequence
     * convert.*/
    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    /* Enable conversions (must be triggered using ASC14_toggle_conversion()) */
    MAP_ADC14_enableConversion();
    //MAP_ADC14_toggleConversionTrigger();
}

/********************************************************
 * DISPLAY Section
 ********************************************************/
// Color parameters for drawing on the screen - see grlib.h
#define TEXTCOL GRAPHICS_COLOR_YELLOW
#define BACKCOL GRAPHICS_COLOR_BLACK
#define DOTCOL GRAPHICS_COLOR_LIGHT_GREEN
#define DOTCOL_PRESSED GRAPHICS_COLOR_RED
#define SECOND_DOT GRAPHICS_COLOR_BLUE
#define RADIUS 2

// Graphics Globals (used by put_dot and ADC14 handler)
Graphics_Context g_sContext;    // graphics context for grlib
uint16_t xscreen, yscreen;      // current screen location coordinates
uint16_t xscreen2, yscreen2;

// Draw a dot (small circle) on the screen at position x,y of color dotcolor
// also ERASE previous dot (remembered in globals xscreen, yscreen)
// adapted for two dots
void put_dot(uint16_t x,uint16_t y, uint16_t x2,uint16_t y2){
    // erase previous dot
    Graphics_setForegroundColor(&g_sContext, BACKCOL);
    Graphics_fillCircle(&g_sContext,xscreen,yscreen,RADIUS);

    // draw the requested circle
    Graphics_setForegroundColor(&g_sContext, DOTCOL);
    Graphics_fillCircle(&g_sContext,x,y,RADIUS);

    Graphics_setForegroundColor(&g_sContext, BACKCOL);
    Graphics_fillCircle(&g_sContext,xscreen2,yscreen2,RADIUS);

    // draw the requested circle
    Graphics_setForegroundColor(&g_sContext, SECOND_DOT);
    Graphics_fillCircle(&g_sContext,x2,y2,RADIUS);

    xscreen=x;
    yscreen=y;
    xscreen2=x2;
    yscreen2=y2;
}

// text printout of joystick readings on the screen
void print_current_results(uint16_t *results){
    char string[8];

    Graphics_setForegroundColor(&g_sContext, TEXTCOL);

    sprintf(string, "X: %5d", results[0]);
    Graphics_drawString(&g_sContext, (int8_t *)string, 8, 20, 116, OPAQUE_TEXT);

    sprintf(string, "Y: %5d", results[1]);
    Graphics_drawString(&g_sContext, (int8_t *)string, 8, 76, 116, OPAQUE_TEXT);
}

void init_display(){
    /* All init code for the display*/
    /* Initializes display */
    Crystalfontz128x128_Init();

    /* Set default screen orientation */
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

    /* Initializes graphics context */
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128,&g_sCrystalfontz128x128_funcs);
    Graphics_setForegroundColor(&g_sContext, TEXTCOL);
    Graphics_setBackgroundColor(&g_sContext, BACKCOL);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);
    Graphics_drawString(&g_sContext, "J:", AUTO_STRING_LENGTH, 0, 116, OPAQUE_TEXT);

    xscreen=0;
    yscreen=0;  // just use origin, first write is a background
    xscreen2=0;
    yscreen2=0;
}

/********************************************************
 * UART Communication Section
 ********************************************************/
/* UART Configuration Parameter. These are the configuration parameters to
 * make the eUSCI A UART module to operate with a 9600 baud rate. These
 * values were calculated using the online calculator that TI provides
 * at:
 *http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */
const eUSCI_UART_Config uartConfig =            //Configuration at 9600 baud rate
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        78,                                      // BRDIV = 78
        2,                                       // UCxBRF = 2
        0,                                       // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};

/* EUSCI A2 UART ISR */
void EUSCIA2_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_BASE);
    MAP_UART_clearInterruptFlag(EUSCI_A2_BASE, status);

    // while the transmit buffer is full dont transmit
    if (status & EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG)//Once the transmit interrupt flag is set allowing the Transmission to occur, go transmit
    {
        if (firsttime == 1){                                    //firsttime = 1 denotes the byte to send it the protocol 255 byte,
            uint8_t protocol = 255;                             //this will indicate that the next byte send will be the x plot point.
            MAP_UART_transmitData(EUSCI_A2_BASE, protocol);     //transmit
            firsttime = 2;                                      //Set firsttime = 2 to denote next send will be of the x coordinate
        }
        else if(firsttime == 2){                                //Send the x coordinate as a 0-127 number
            firstint = resultsBuffer[0]/128;                    //divide by 128 to get in range of 0-127
            MAP_UART_transmitData(EUSCI_A2_BASE, firstint);     //transmit
            firsttime = 3;                                      //set firsttime = 3 to denote next send will be of the y coordinate
        }
        else {                                                  //Send the y coordinate as a 0-127 number
            secondint = resultsBuffer[1]/128;                   //divide by 128 to get in range of 0-127
            MAP_UART_transmitData(EUSCI_A2_BASE, secondint);    //transmit
            MAP_UART_disableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);//disable interrupt until the ADC interrupt is called again to prevent to many sending
            firsttime = 1;                                      //Set firsttime = 1 so that next send is the 255 protocol
        }
    }

    // This flag is set when the UART module receives a START byte.
    else if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)//receiving section
    {
        if (transmit_order == 1) {                                  //Will run first time to fill uart_control
            uart_control[0] = MAP_UART_receiveData(EUSCI_A2_BASE);  //receive
            transmit_order = 2;                                     //set so this section will not run again
        }
        else {
            uart_control[1] = MAP_UART_receiveData(EUSCI_A2_BASE);  //Place new number in second spot

            if (uart_control[0] == 255){                            //if last one was 255 then this one is x coordinate so we place in receive_byte[0]
                receive_byte[0] = uart_control[1];
                uart_control[0] = uart_control[1];
            }
            else if (uart_control[0] == receive_byte[0]){           //else if the last one was x then we place in y coordinate location receive_byte[1]
                receive_byte[1] = uart_control[1];
                uart_control[0] = uart_control[1];
            }
            else if (uart_control[1] == 255){                       //if this one is the protocol then we just shift it to first location
                uart_control[0] = uart_control[1];
            }
        }
    }
    print_flag = 1;  // signal main to refresh the display
}

void init_uart(){
    transmit_order = 1;

    /* Selecting P3.2 and P3.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A2_BASE, &uartConfig);

    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A2_BASE);

    /* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);
}

/**********************************
 * Main function
 **********************************/
void main(void)
{
    // Variables for refresh of display
    //unsigned dotcolor;          // color of the dot to display
    uint16_t xdisplay,ydisplay; // screen coordinates to disolay
    uint16_t xdisplay2,ydisplay2;

    /* Setting DCO to 12MHz */
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);  // 3, 6, 12, 24, 48 MHz options

    /* Initialize the modules */
    init_WDT();
    init_display(); // setup the display
    init_ADC();
    init_uart();

    print_flag=0;   //clear print flag until there is a result

    /* Enable Interrupts at the NVIC level */
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableInterrupt(INT_WDT_A);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA2);  //UART

    MAP_Interrupt_disableSleepOnIsrExit();   // Specify that after an interrupt, the CPU wakes up
    MAP_Interrupt_enableMaster();

    while(1)
    {
        MAP_PCM_gotoLPM0();
        __no_operation(); //  For debugger
        if (print_flag)
        {
            print_flag=0;
            xdisplay2 = receive_byte[0];
            ydisplay2 = 127 - receive_byte[1];

            xdisplay = resultsBuffer[0]/128;
            ydisplay=127-resultsBuffer[1]/128;

            put_dot(xdisplay, ydisplay, xdisplay2, ydisplay2);
            print_current_results(resultsBuffer);
        }
    }
}
