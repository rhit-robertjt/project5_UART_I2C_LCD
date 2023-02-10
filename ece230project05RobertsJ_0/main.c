/*! \file */
/******************************************************************************
 *
 * Description: This project uses a terminal in UART mode to communicate with a
 * microcontroller. The microcontroller will interface with a MPU6050 in I2C
 * configuration and an LCD. Based on the chosen parameter in the terminal, the
 * MPU will be in 2g, 4g, 8g, or 16g mode, and the LCD will display the X-axis,
 * Y-axis, or Z-axis of the accelerometer in terms of gravity to the nearest mg.
 * The terminal is in 38400 8E1 and the DCO, MCLK, and SMCLK are at 12MHz.
 *
 *
 * Author: Justin Roberts
 * Last-modified: 1/30/23
 *
 * An external LF crystal between LFXIN & LFXOUT is required for ACLK
 *                                  ___  ___
 *                                   |    |
 *               MSP432P411x        10k  10k     MPU6050
 *             ------------------    |    |    -----------
 *         /|\|     P1.6/UCB0SDA |<--|----|-->| SDA
 *          | |                  |   |        |
 *          --|RST               |   |        |
 *            |     P1.7/UCB0SCL |<--|------->| SCL
 *            |              Vcc |<---------->| ADD
 *            |                  |             -----------
 *            |                  |
 *            |                  |                LCD
 *            |                  |             --------
 *            |             P5.7 |----------->| RS
 *            |                  |            |
 *            |             P5.6 |----------->| En
 *            |                  |            |
 *            |                  |     8      |
 *            |              P4  |-----\----->| DB
 *            |                  |            |
 *            |                  |             --------
 *            |                  |
 *            |             PJ.0 |------
 *            |                  |     |
 *            |                  |    LFXT @ 32kHz
 *            |                  |     |
 *            |             PJ.1 |------
 *            |                  |
 *            |     P1.3/UCA0TXD |---------->  PC (echo)
 *            |     P1.2/UCA0RXD |<----------  PC
 *
*******************************************************************************/

#include "msp.h"
#include <stdint.h>
#include <stdbool.h>
#include "csLFXT.h"
#include "lcd.h"

#define CLK_FREQUENCY       12000000    // MCLK using CS
#define NUM_OF_REC_BYTES        6       // number of bytes to receive from sensor read
#define GY521_ADDRESS           0x69    // I2C address of GY-521 sensor, ADD pin = Vcc
#define ACCEL_BASE_ADDR         0x3B    // base address of accelerometer data registers
#define PWR_MGMT_ADDR           0x6B    // address of power management register
#define ACCEL_MGMT_ADDR         0x1C    // address of accelerometer management register

uint8_t RXData[NUM_OF_REC_BYTES] = {0, 0, 0, 0, 0, 0};
uint8_t RXDataPointer, TXDataPointer;
int16_t accel_x, accel_y, accel_z;
uint32_t i;

/* Used to talk to the LCD */
const char accel[] = {'A', 'c', 'c', 'e', 'l', 'e', 'r', 'o', 'm', 'e', 't', 'e', 'r'}; // gets rid of the null end-of-string char
const char mode2[] = " 2g";
const char mode4[] = " 4g";
const char mode8[] = " 8g";
const char mode16[] = "16g";
const char modeX[] = {'X', ':'};
const char modeY[] = {'Y', ':'};
const char modeZ[] = {'Z', ':'};

uint8_t mode_spec = 2; // changes the sensor mode based on input, will be 0, 1, 2, 3 only
uint8_t mode_gyro = 255; // changes the axis that is being read, 0=X, 1=Y, 2=Z, 3=P

/* Used to talk to the terminal */
//const char prompt[] = "ac"; // verification code to test parity
const char prompt[] = "\n\rOptions: \n\r(P)rint to terminal, display (X)-axis, "
                      "display (Y)-axis, display (Z)-axis, \n\r(2)g sensor range, "
                      "(4)g sensor range, (8)g sensor range, 1(6)g sensor range "
                      "\n\rSelection:";
const char AccelX[] = "\n\rAccel_X:";
const char AccelY[] = "    Accel_Y:";
const char AccelZ[] = "    Accel_Z:";

_Bool newValues = false;
_Bool newChoice = false;
_Bool newSpec = false;

void GY_521_init(void){
    /* Configure UART pins */
    P1->SEL0 |= BIT6 | BIT7;                // set I2C pins as secondary function
    P1->SEL1 &= ~(BIT6 | BIT7);

    // Initialize data variable
    RXDataPointer = 0;
    TXDataPointer = 0;

    /* Configure eUSCI_B0 for I2C mode
     *  I2C master mode, synchronous, 7-bit address, SMCLK clock source,
     *  transmit mode, with automatic STOP condition generation
     */
    EUSCI_B0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Software reset enabled
    EUSCI_B0->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI in reset mode
            EUSCI_B_CTLW0_MODE_3 |          // I2C mode
            EUSCI_B_CTLW0_MST |             // Master mode
            EUSCI_B_CTLW0_SYNC |            // Sync mode
            EUSCI_B_CTLW0_TR |              // Transmitter mode
            EUSCI_B_CTLW0_SSEL__SMCLK;      // SMCLK

    /* I2C clock calculation */
    // configure eUSCI_B0 bit rate control for 100 kbps
    EUSCI_B0->BRW = 120;
    /* Configure I2C to communicate with GY-521 */
    EUSCI_B0->I2CSA = GY521_ADDRESS;            // I2C peripheral address
    EUSCI_B0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;    // Release eUSCI from reset

    /* Initialize GY-521 by writing to Power Management Register */
    // Ensure stop condition not pending
    while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);
    do {
        // Send I2C start condition and address frame with W
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
        // wait for TX buffer to be ready
        while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
        // load 1st data byte into TX buffer
        EUSCI_B0->TXBUF = PWR_MGMT_ADDR;            // send register address
        // wait for ACK/NACK after address frame
        while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT);
    } while(EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG);   // resend address frame if ACK not received
    // wait for TX buffer to be ready
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
    // load 2nd data byte into TX buffer
    EUSCI_B0->TXBUF = 0;                // write value to register
    // wait for 2nd data byte to begin transmit
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
    // Send I2C stop condition
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

    // Ensure stop condition got sent
    while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);
    // ensure flags are cleared before enabling interrupts
    EUSCI_B0->IFG &= ~(EUSCI_B_IFG_TXIFG0 | EUSCI_B_IFG_RXIFG0 | EUSCI_B_IFG_NACKIFG);

    EUSCI_B0->IE |= EUSCI_A_IE_RXIE |       // Enable receive interrupt
            EUSCI_A_IE_TXIE |               // Enable transmit interrupt
            EUSCI_B_IE_NACKIE;              // Enable NACK interrupt

    // Enable eUSCIB0 interrupt in NVIC module
    NVIC->ISER[0] |= (1 << EUSCIB0_IRQn);

    // Enable global interrupt
    __enable_irq();
}

void changeAccelMode(uint8_t in){
    while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);
    do {
        // Send I2C start condition and address frame with W
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
        // wait for TX buffer to be ready
        while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
        // load 1st data byte into TX buffer
        EUSCI_B0->TXBUF = ACCEL_MGMT_ADDR;            // send register address
        // wait for ACK/NACK after address frame
        while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT);
    } while(EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG);   // resend address frame if ACK not received
    // wait for TX buffer to be ready
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
    // load 2nd data byte into TX buffer
    EUSCI_B0->TXBUF = in << 3;                // write value to register
    // wait for 2nd data byte to begin transmit
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
    // Send I2C stop condition
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

    // Ensure stop condition got sent
    while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);
    newSpec = false;
}

void terminal_init(void){

    /* Configure UART pins */
    P1->SEL0 |= BIT2 | BIT3;                // set 2-UART pins as secondary function
    P1->SEL1 &= ~(BIT2 | BIT3);

    /* Configure UART
     *  Asynchronous UART mode, 8E1 (8-bit data, even parity, 1 stop bit),
     *  LSB first, SMCLK clock source
     */
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST |  // Remain eUSCI in reset
    // complete configuration of UART in eUSCI_A0 control register
            EUSCI_A_CTLW0_PEN | EUSCI_A_CTLW0_MODE_0 | EUSCI_A_CTLW0_SSEL__SMCLK | EUSCI_A_CTLW0_PAR;
    /* Baud Rate calculation */
    // set clock prescaler in eUSCI_A0 baud rate control register
    EUSCI_A0->BRW = 19;
    // configure baud clock modulation in eUSCI_A0 modulation control register
    //                   BRS             BRF         Oversampling Enable
    EUSCI_A0->MCTLW = (0x65 << 0x8) + (8 << 4) + EUSCI_A_MCTLW_OS16;

    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;    // Initialize eUSCI
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;        // Clear eUSCI RX interrupt flag
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;            // Enable USCI_A0 RX interrupt

    // Enable global interrupt
    __enable_irq();

    // Enable eUSCIA0 interrupt in NVIC module
    NVIC->ISER[0] |= (1 << EUSCIA0_IRQn );

}

void printMessage(const char* message, int msgLength) {
    int i;
    for (i = 0; i < msgLength; i++) {
        // Check if the TX buffer is empty first
        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));

        // Send next character of message
        //  Note that writing to TX buffer clears the flag
        EUSCI_A0->TXBUF = message[i];
    }
}

void printArrLCD(const char* arr, int size){
    int i;
    for(i = 0; i < size;i++){
        printChar(arr[i]);
    }

}

void timer_init(void){
    /* Configure Timer_A0 and CCRs */
    TIMER_A0->CCR[0] = 31999; // timer 0 is used for a 1 Hz GY-521 polling cycle
    TIMER_A0->CTL   |= TIMER_A_CTL_MC__UP | TIMER_A_CTL_CLR | TIMER_A_CTL_IE |
                       TIMER_A_CTL_SSEL__ACLK;
    NVIC->ISER[0] |= BIT(TA0_N_IRQn);
}

void printSpec(void){
    switch(mode_spec){
    case 0:
        printArrLCD(mode2, sizeof(mode2)/sizeof(mode2[0]));
        break;
    case 1:
        printArrLCD(mode4, sizeof(mode4)/sizeof(mode4[0]));
        break;
    case 3:
        printArrLCD(mode16, sizeof(mode16)/sizeof(mode16[0]));
        break;
    default:
        printArrLCD(mode8, sizeof(mode8)/sizeof(mode8[0]));
        break;
    }
}

void printDigits(int16_t in, _Bool mode){
    int single, i = 0;
    float g_s = (in / 16384.0);
    char arr[] = {0, 0, '.', 0, 0, 0};
    if(in < 0){
        arr[0] = '-';
        g_s *= -1;
    } else {
        arr[0] = ' ';
    }

    arr[1] = (int) g_s + 0x30;
    g_s -= (arr[1]-0x30);
    g_s *= 1000;

    in = (int) g_s;

    for(i = 0; i < 3; i++){
        single = in % 10;
        arr[5-i] = single + 0x30;
        in /= 10;
    }
    if(mode){
        printArrLCD(arr, sizeof(arr)/sizeof(arr[0]));
        printChar('g');
    } else {
        printMessage(arr, sizeof(arr)/sizeof(arr[0]));
    }
}

void terminalPrinter(void){
    switch(mode_gyro){
    case 0:
        // x-axis
        // clear display
        clearDisplay();
        printArrLCD(accel, sizeof(accel)/sizeof(accel[0]));
        printSpec();
        // move to second row
        secondRow();
        printArrLCD(modeX, sizeof(modeX)/sizeof(modeX[0]));
        printDigits(accel_x, 1);
        break;
    case 1:
        // y-axis
        // clear display
        clearDisplay();
        printArrLCD(accel, sizeof(accel)/sizeof(accel[0]));
        printSpec();
        // move to second row
        secondRow();
        printArrLCD(modeY, sizeof(modeY)/sizeof(modeY[0]));
        printDigits(accel_y, 1);
        break;
    case 2:
        // z-axis
        // clear display
        clearDisplay();
        printArrLCD(accel, sizeof(accel)/sizeof(accel[0]));
        printSpec();
        // move to second row
        secondRow();
        printArrLCD(modeZ, sizeof(modeZ)/sizeof(modeZ[0]));
        printDigits(accel_z, 1);
        break;
    case 3:
        // print to terminal
        printMessage(AccelX, sizeof(AccelX)/sizeof(AccelX[0]));
        printDigits(accel_x, 0);
        printMessage(AccelY, sizeof(AccelY)/sizeof(AccelY[0]));
        printDigits(accel_y, 0);
        printMessage(AccelZ, sizeof(AccelZ)/sizeof(AccelZ[0]));
        printDigits(accel_z, 0);
        break;
    default:
        break;
    }
    // print prompt to terminal over UART
    printMessage(prompt, (sizeof(prompt)/sizeof(prompt[0])));
    // Ensure stop condition got sent
    while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);
}

/**
 * main.c
 */
void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

    /* Configure MCLK/SMCLK source to DCO, with DCO = 12MHz */
    CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 8-16MHz range)
    CS->CTL1 = CS_CTL1_SELS_3 |             // SMCLK = DCO
            CS_CTL1_SELM_3;                 // MCLK = DCO
    CS->KEY = 0;                            // Lock CS module from unintended accesses

    // Sets ACLK to 32kHz
    configLFXT();
    // configures pins and delay library
    configLCD(CLK_FREQUENCY);
    // sends initialization sequence and configuration to LCD
    initLCD();
    timer_init();
    GY_521_init();
    terminal_init(); // SMCLK/MCLK 12MHz, ACLK 32 kHz
    changeAccelMode(2); // GY-521 begins at +/- 8g config

    // change to transmitter mode (Write)
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR;
    // begin I2C transmission
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    // print prompt to terminal over UART
    printMessage(prompt, (sizeof(prompt)/sizeof(prompt[0])));

    while(1){
        if(newValues){
            /* combine bytes to form 16-bit accel_ values  */
            accel_x = (RXData[0] << 0x8) + RXData[1];
            accel_y = (RXData[2] << 0x8) + RXData[3];
            accel_z = (RXData[4] << 0x8) + RXData[5];
            RXDataPointer = 0;
            newValues = false;
        }

        if(newChoice){
            terminalPrinter();
            newChoice = false;
        }

        if(newSpec){
            changeAccelMode(mode_spec);
            // print prompt to terminal over UART
            printMessage(prompt, (sizeof(prompt)/sizeof(prompt[0])));
            // Ensure stop condition got sent
            while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);
            clearDisplay();
            printArrLCD(accel, sizeof(accel)/sizeof(accel[0]));
            printSpec();
        }
    }
}

// I2C interrupt service routine
void EUSCIB0_IRQHandler(void)
{
    // Handle if ACK not received for address frame
    if (EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG) {
        EUSCI_B0->IFG &= ~ EUSCI_B_IFG_NACKIFG;

        // resend I2C start condition and address frame
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
        TXDataPointer = 0;
        RXDataPointer = 0;
    }
    // When TX buffer is ready, load next byte or Restart for Read
    if (EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) {
        if (TXDataPointer == 0) {
            // load 1st data byte into TX buffer (writing to buffer clears the flag)
            EUSCI_B0->TXBUF = ACCEL_BASE_ADDR;      // send register address
            TXDataPointer = 1;
        } else {
            // change to receiver mode (Read)
            EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_TR;
            // send Restart and address frame with R bit
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
            TXDataPointer = 0;
            RXDataPointer = 0;
            // need to clear flag since not writing to buffer
            EUSCI_B0->IFG &= ~ EUSCI_B_IFG_TXIFG0;
        }
    }
    // When new byte is received, read value from RX buffer
    if (EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0) {
        // Get RX data
        if (RXDataPointer < NUM_OF_REC_BYTES) {
            // reading the buffer clears the flag
            RXData[RXDataPointer++] = EUSCI_B0->RXBUF;
        }
        else {  // in case of glitch, avoid array out-of-bounds error
            EUSCI_B0->IFG &= ~ EUSCI_B_IFG_RXIFG0;
        }

        // check if last byte being received - if so, initiate STOP (and NACK)
        if (RXDataPointer == (NUM_OF_REC_BYTES-1)) {
            // Send I2C stop condition
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
            newValues = true;
        }
    }
}

// UART interrupt service routine
void EUSCIA0_IRQHandler(void)
{
    // Check if receive flag is set (value ready in RX buffer)
    if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG)
    {
        // Note that reading RX buffer clears the flag
        uint8_t digit = EUSCI_A0->RXBUF;

        // Echo character back to screen, otherwise user will not be able to
        //  verify what was typed
        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG)); // Wait for TX buffer ready
        EUSCI_A0->TXBUF = digit;                 // Echo character to terminal

        switch (digit) {
        // deal with the 2, 4, 8, 16 here
        case '2':
            mode_spec = 0;
            newSpec = true;
            break;
        case '4':
            mode_spec = 1;
            newSpec = true;
            break;
        case '6':
            mode_spec = 3;
            newSpec = true;
            break;
        case '8':
            mode_spec = 2;
            newSpec = true;
            break;
        case 'X':
        case 'x':
            // deal with x-accel here
            mode_gyro = 0;
            newChoice = true;
            break;
        case 'Y':
        case 'y':
            // deal with y-accel here
            mode_gyro = 1;
            newChoice = true;
            break;
        case 'Z':
        case 'z':
            // deal with z-accel here
            mode_gyro = 2;
            newChoice = true;
            break;
        case 'P':
        case 'p':
            // deal with printing to terminal here
            mode_gyro = 3;
            newChoice = true;
            break;
        default:
            // invalid character(s) received, ignore it
            break;
        }
    }
}

void TA0_N_IRQHandler(void) {
    if(TIMER_A0->CTL & TIMER_A_CTL_IFG){
        /* Read register values from sensor by sending register address and restart
         *  Initiated with start condition - completion handled in other ISR */
        // change to transmitter mode (Write)
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR;
        // send I2C start condition with address frame and W bit
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
        /* Clear Flag */
        TIMER_A0->CTL &= ~TIMER_A_CTL_IFG;
    }
}

