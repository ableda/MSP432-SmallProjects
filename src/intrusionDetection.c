/*****************************************************************************
 * Home intrusion detection system using a GSM module and a PIR sensor
 * Created by Alex Bleda and Nick Dargi
 *
 *****************************************************************************/
/* DriverLib Includes */
#include </Users/alexbleda/ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

// What to send to GSM
unsigned char setBaud[13] = "AT+IPR=9600";
unsigned char initializer[10] = "AT+CMGF=1";
unsigned char phoneNumba[27] = "AT+CMGS=\"+18029173275\"\r";
unsigned char message[49] = "Intruder Detected, Reply RESET to re-arm system.";
unsigned char receiveing[18] = "AT+CNMI=2,2,0,0,0";
unsigned char ctlZ = 26;
unsigned char rrr = 13;
unsigned char newline = 10;

//Global variables
uint16_t state = 1; //where are we
uint16_t index = 0; //which one are we
uint16_t line = 0;

// P3.2 connects to 7   RX-->TX
// P3.3 connects to 8   TX-->RX
int see = 1;
uint16_t receiveindex = 0;
unsigned char receiver[5];
unsigned char buffer[150];
unsigned char reply[5] = "RESET";
int locatin = 0;

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
    int i;
    int k;

    // while the transmit buffer is full dont transmit
    if (status & EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG)//Once the transmit interrupt flag is set allowing the Transmission to occur, go transmit
    {
        if (state == 0){
            if (index <= 10)
                MAP_UART_transmitData(EUSCI_A2_BASE, setBaud[index]);    //transmit

            else{
                if (line == 0){
                    MAP_UART_transmitData(EUSCI_A2_BASE, newline);
                    line = 1;
                }
                else if (line == 1){
                    MAP_UART_transmitData(EUSCI_A2_BASE, rrr);
                    line = 0;
                    state = 1;
                    index = 0;
                    for (i = 0; i < 1000000; i++);
                    return;
                }
            }
            index++;
        }
        if (state == 1){
            if (index <= 8)
                MAP_UART_transmitData(EUSCI_A2_BASE, initializer[index]);    //transmit

            else{
                if (line == 0){
                    MAP_UART_transmitData(EUSCI_A2_BASE, rrr);
                    line = 1;
                }
                else if (line == 1){
                    MAP_UART_transmitData(EUSCI_A2_BASE, newline);
                    line = 0;
                    state = 2;
                    index = 0;
                    for (i = 0; i < 1000000; i++);
                    return;
                }
            }
            index++;
        }
        else if (state == 2){
            if (index <= 22)
                MAP_UART_transmitData(EUSCI_A2_BASE, phoneNumba[index]);

            else{
                if (line == 0){
                    MAP_UART_transmitData(EUSCI_A2_BASE, rrr);
                    line = 1;
                }
                else if (line == 1){
                    MAP_UART_transmitData(EUSCI_A2_BASE, newline);
                    line = 0;
                    state = 3;
                    index = 0;

                    for (i = 0; i < 1000000; i++);
                    return;
                }
            }
            index++;
        }
        else if (state == 3){
            if(index <= 47)
                MAP_UART_transmitData(EUSCI_A2_BASE, message[index]);

            else{
                if (line == 0){
                    MAP_UART_transmitData(EUSCI_A2_BASE, rrr);
                    line = 1;
                }
                else if (line == 1){
                    MAP_UART_transmitData(EUSCI_A2_BASE, newline);
                    line = 0;
                    state = 4;
                    index = 0;
                    for (i = 0; i < 1000000; i++);
                    return;
                }
            }
            index++;
        }
        else if (state == 4){
            if (index == 0){
                MAP_UART_transmitData(EUSCI_A2_BASE, ctlZ);
                index++;
            }
            else{
                if (line == 0){
                    MAP_UART_transmitData(EUSCI_A2_BASE, rrr);
                    line = 1;
                }
                else if (line == 1){
                    MAP_UART_transmitData(EUSCI_A2_BASE, newline);
                    line = 0;
                    state = 5;
                    index = 0;
                    for (i = 0; i < 1000000; i++);
                    return;
                }
            }
        }
        else if (state == 5){
                if (index <= 16){
                    MAP_UART_transmitData(EUSCI_A2_BASE, receiveing[index]);
                    index++;
                }
                else{
                    if (line == 0){
                        MAP_UART_transmitData(EUSCI_A2_BASE, rrr);
                        line = 1;
                    }
                    else if (line == 1){
                        MAP_UART_transmitData(EUSCI_A2_BASE, newline);
                        line = 0;
                        state = 6;
                        index = 0;

                        MAP_UART_disableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);

                        for (i = 0; i < 1000000; i++);
                        return;
                    }
                }
        }
    }

    // This flag is set when the UART module receives a START byte.
    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)//receiving section
    {

        for (k = 0; k < 5; k++){
            receiver[k]=receiver[k+1];
        }

        receiver[4] = MAP_UART_receiveData(EUSCI_A2_BASE);
        buffer[locatin] = receiver[4];
        locatin++;
        int same = 1;

        same  = memcmp(receiver, reply, 5);
         // They must be the same
        if (same == 0)
            see = 1;
    }
}

void PORT2_IRQHandler(){

    uint32_t status;
    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P2);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P2, status);

    /* Toggling the output on the LED */
    if( see && status && GPIO_PIN7)
    {
        state = 1;
        see = 0;
        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);
    }

}
void init_uart(){
    /* Selecting P3.2 and P3.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A2_BASE, &uartConfig);

    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A2_BASE);

    /* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

}


void main(void)
{
    /* Halting the Watchdog */
    MAP_WDT_A_holdTimer();

    /* Setting DCO to 12MHz */
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);  // 3, 6, 12, 24, 48 MHz options

    /* Initialize the modules */
    init_uart();

    /* Configuring P2.7 as input pin for PIR, interrupt only from low to high */
    MAP_GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P2, GPIO_PIN7);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P2,GPIO_PIN7,GPIO_LOW_TO_HIGH_TRANSITION);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P2,GPIO_PIN7);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN7);

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    /* Enable Interrupts at the NVIC level */
    MAP_Interrupt_enableInterrupt(INT_EUSCIA2);  //UART
    MAP_Interrupt_enableInterrupt(INT_PORT2);

    MAP_Interrupt_disableSleepOnIsrExit();   // Specify that after an interrupt, the CPU wakes up
    MAP_Interrupt_enableMaster();

    while(1)
    {
        MAP_PCM_gotoLPM0();
        __no_operation(); //  For debugger
    }

}
