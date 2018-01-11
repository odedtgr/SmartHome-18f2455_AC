#ifndef MAIN_C
#define MAIN_C

#define ADDRESS 6//XBee address.

// includes
#include <string.h>
#include <usart.h>
#include <delays.h>
#include <stdio.h>
#include <math.h>

// Define the globals 
#pragma udata
double time=0;
char RxBuffer[100];
char TxBuffer[100];
int receivePos = 0;
double SWcount = 0;
unsigned char WHBuffer = 0;
int IR1inBuffer;
int IR2inBuffer;
int LED1buffer;
int LED2buffer;
int deviceState = 0;
char ADDRH = 0x00;
char ADDRL = 0x00;
#pragma udata

//define
//=================================
// Comment out the following line if you do not want the debug
// feature of the firmware (saves code and RAM space when off)//
// Note: if you use this feature you must compile with the large
// memory model on (for 24-bit pointers) so that the sprintf()
// function will work correctly.  If you do not require debug it's
// recommended that you compile with the small memory model and 
// remove any references to <strings.h> and sprintf().
#define DEBUGON

//Use LATA to write, PORTA to read.
//Remember to configure input/output
#define LED1            LATAbits.LATA3 //Status led 
#define WHSwitch        PORTBbits.RB1 //Switch 1 input
#define IR1in           PORTBbits.RB7 
#define LED1in          PORTBbits.RB6 
#define IR2in           PORTBbits.RB5 
#define LED2in          PORTBbits.RB4 
#define IR1out          LATBbits.LATB3 
#define IR2out          LATBbits.LATB2
#define LED1out         LATAbits.LATA1
#define LED2out         LATAbits.LATA0

// PIC18F4550/PIC18F2550 configuratin 
#pragma config PLLDIV   = 5         
#pragma config CPUDIV   = OSC1_PLL2   
#pragma config USBDIV   = 2         // Clock source from 96MHz PLL/2
#pragma config FOSC = INTOSCIO_EC// Oscillator Selection bits (Internal oscillator, port function on RA6, EC used by USB (INTIO))
#pragma config FCMEN    = OFF
#pragma config IESO     = OFF
#pragma config PWRT     = ON
#pragma config BOR      = OFF
#pragma config BORV     = 3
#pragma config VREGEN   = ON
#pragma config WDT      = OFF
#pragma config WDTPS    = 32768
#pragma config MCLRE    = ON
#pragma config LPT1OSC  = OFF
#pragma config PBADEN   = OFF
#pragma config STVREN   = ON
#pragma config LVP      = OFF
#pragma config XINST    = OFF
#pragma config CP0      = OFF
#pragma config CP1      = OFF
#pragma config CPB      = OFF
#pragma config WRT0     = OFF
#pragma config WRT1     = OFF
#pragma config WRTB     = OFF
#pragma config WRTC     = OFF
#pragma config EBTR0    = OFF
#pragma config EBTR1    = OFF
#pragma config EBTRB    = OFF

// Private function prototypes
static void initialisePic(void);
void highPriorityISRCode();
void lowPriorityISRCode();
void initUsart(void);
void initXbee(void);
void ReceiveUsart(void);
void TransmitUsartDeviceState(char device);
void TransmitUsartAT(void);
void delay1sec(void);
void handleMessge(void);
void ACcommands(char device);
void transmitIR(char device);

#pragma code HIGH_INTERRUPT_VECTOR = 0x08
void High_ISR(void) {
    _asm goto highPriorityISRCode _endasm
}

#pragma code LOW_INTERRUPT_VECTOR = 0x18
void Low_ISR(void) {
    _asm goto lowPriorityISRCode _endasm
}

#pragma code

// High-priority ISR handling function
#pragma interrupt highPriorityISRCode
void highPriorityISRCode() {
    // Application specific high-priority ISR code goes here

    //USART receive interrupt, Cleared when RCREG is read.
    if (PIR1bits.RCIF){
        ReceiveUsart();
    }

    //PORTB interrupt on change
    if (INTCONbits.RBIF) {
        //IR Signal level change
        if ((IR1inBuffer != IR1in) || (IR2inBuffer != IR2in)) {
            IR1out = IR1in;
            IR2out = IR2in;
            IR1inBuffer = IR1in;
            IR2inBuffer = IR2in;
        } else if (LED1buffer != LED1in) {
            LED1out=LED1in;
            TransmitUsartDeviceState(1);
            LED1buffer = LED1in;
        } else if (LED2buffer != LED2in) {
            LED2out=LED2in;
            TransmitUsartDeviceState(2);
            LED2buffer = LED2in;
        }
        INTCONbits.RBIF = 0; //Flag must be cleared
    }
}

// Low-priority ISR handling function
#pragma interruptlow lowPriorityISRCode

void lowPriorityISRCode() {
}

// Main program entry point

void main(void) {
    //init device
    initialisePic();
    initUsart();
    initXbee();
    //Main loop

    //AC1 init
    IR1inBuffer = IR1in;
    IR1out = IR1in;
    LED1buffer=LED1in;
    LED1out=LED1in;
    //AC2 init
    IR2inBuffer = IR2in;
    IR2out = IR2in;
    LED2buffer=LED2in;
    LED2out=LED2in;

    while (1) {
        Sleep();
    }
}

void handleMessage(void) {
    if (RxBuffer[3] == 0x81) //RX (Receive) Packet: 16-bit Address
    {
        ADDRH = RxBuffer[4];
        ADDRL = RxBuffer[5];
        switch (RxBuffer[8]) //first data byte, Device selection
        {
            case 1://Device AC (livingroom)
                ACcommands(RxBuffer[8]);
                break;

            case 2://Device AC (Rooms)
                ACcommands(RxBuffer[8]);
                break;
            default: // Unknown command received
                break;
        }
    }
}

// Initialise the PIC
static void initialisePic(void) {
    //Internal clock freq 8MHz
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF2 = 1;
    //sleep configuration
    OSCCONbits.IDLEN = 1;
    // Default all pins to digital
    ADCON1 = 0x0F; 
    // Clear all ports
    PORTA = 0b00000000;
    PORTB = 0b00000000;
    PORTC = 0b00000000;
    // Configure ports as inputs (1) or outputs(0)
    TRISA = 0b00000000;
    TRISB = 0b11110000;
    TRISC = 0b00000000;
    // Configure interrupts
    INTCON2bits.RBPU = 0; //PORTB weak pullup enabeled
    WHBuffer = PORTB; //Reading PORTB allows RBIF to be cleared.
    WHBuffer = 0;
    INTCONbits.GIEH = 1; //Enable global interrupts
    INTCONbits.GIEL = 1; //Enable lopw priority interrupts
    RCONbits.IPEN = 1; //Enable priority interrupts
    INTCONbits.RBIF = 0; //Reset the interrupt flag
    INTCON2bits.RBIP = 1; //PORTB interrupt priority set to High
    INTCONbits.RBIE = 1; //enable interrupt on input change on PORTB<7:4>
    // Timer0 configuration
    T0CONbits.T0CS = 0; //0 = Internal instruction cycle clock (CLKO)
    T0CONbits.PSA = 0; //Use pre-scalar
    T0CONbits.T0PS0 = 1;
    T0CONbits.T0PS1 = 0;
    T0CONbits.T0PS2 = 1; //prescalar 1:64, overflow every 8.192ms
    INTCON2bits.T0IP = 0; //TMR0 interrpt low priority
    INTCONbits.TMR0IE = 0; //Timer0 interrupt on overflow
}

void initUsart() {
    TRISCbits.RC6 = 1;
    TRISCbits.RC7 = 1;
    TXSTAbits.TXEN = 1; // Transmit Enable bit
    RCSTAbits.SPEN = 1; // Serial Port Enable bit
    RCSTAbits.CREN = 1; // Continuous Receive Enable bit
    TXSTAbits.BRGH = 1; // High Baud Rate Select bit
    BAUDCONbits.BRG16 = 1; //16-Bit Baud Rate Register Enable bit
    SPBRGH = 0;
    SPBRG = 103; // baudrate: 103-19,230 207-9,600
    IPR1bits.RCIP = 1; //Set USART receive interrupt low priority
    PIE1bits.RCIE = 1; //Enable USART receive interrupts
    PIR1bits.TXIF = 1; //The EUSART transmit buffer, TXREG, is empty (cleared when TXREG is written)
}

void initXbee() {
    delay1sec();
    sprintf(TxBuffer, "X");
    TransmitUsartAT();
    delay1sec();
    sprintf(TxBuffer, "+++"); //Enter At command mode
    TransmitUsartAT();
    delay1sec();
    sprintf(TxBuffer, "ATMY%d\r", ADDRESS); //Set address
    TransmitUsartAT();
    sprintf(TxBuffer, "ATBD4\r"); //Set BaudeRate 19,200
    TransmitUsartAT();
    sprintf(TxBuffer, "ATAP1,WR,AC,CN\r"); //Set API mode 1
    TransmitUsartAT();
}

void delay1sec() {
    //TCY = 1sec/(8MHz/4) =0.5 us
    Delay10KTCYx(220);//1 sec is 200, but AT commands were not working, so i increased the delay
}

void ReceiveUsart() {
    char c;
    int i;
    if (PIR1bits.RCIF == 1 && PIE1bits.RCIE == 1) //Check data in RCREG.
    {
        if (RCSTAbits.OERR == 1) {
            RCSTAbits.CREN = 0x0; //Stop continuous reception to clear the error flag FERR.
            RCSTAbits.CREN = 0x1; //Enable continuous reception again.
        }
        c = RCREG; //Read data from RCREG
        if (c == 0x7e)//API Frame start delimeter recived
            receivePos = 0;
        RxBuffer[receivePos] = c;
        //length of data received
        if (receivePos >= 2) {
            if (receivePos == RxBuffer[1]*256 + RxBuffer[2] + 3)
                handleMessage();
        }
        receivePos++;
    }
}

void TransmitUsartDeviceState(char device) {
    int i, len;

    TxBuffer[0] = 0x7e; //Start delimiter
    TxBuffer[1] = 0x00; //length MSB
    TxBuffer[2] = 0x07; //length LSB
    TxBuffer[3] = 0x01; //API identifier: TX Request, 16-bit address
    TxBuffer[4] = 0x00; //frame ID
    TxBuffer[5] = ADDRH; //Destination address MSB
    TxBuffer[6] = ADDRL; //Destination address LSB
    TxBuffer[7] = 0x00; //options -Disable ACK
    TxBuffer[8] = device; //Device selection
    if(device==1)
        TxBuffer[9] = LED1in; //Data
    else
        TxBuffer[9] = LED2in; //Data
    TxBuffer[10] = 0xff - (TxBuffer[3] + TxBuffer[4] + TxBuffer[5] + TxBuffer[6] + TxBuffer[7] + TxBuffer[8]+TxBuffer[9]); //checksum

    len = TxBuffer[1]*256 + TxBuffer[2] + 4;
    for (i = 0; i < len; i++) {
        while (TXSTAbits.TRMT != 1);
        WriteUSART(TxBuffer[i]);
    }
}

void TransmitUsartAT() {
    int i, len;
    len = strlen(TxBuffer);
    for (i = 0; i < len; i++) {
        while (TXSTAbits.TRMT != 1);
        WriteUSART(TxBuffer[i]);
    }
}

void ACcommands(char device) {
    switch (RxBuffer[9]) //Device specific command
    {
        case 0://Transmit device state
            TransmitUsartDeviceState(device);
            break;

        case 1://Transmit IR data
            transmitIR(device);
            break;

        default: // Unknown command received
            break;
    }
}

void transmitIR(char device) {
    int i;
    int j;
    for (j = 10; j <= (RxBuffer[1]*256 + RxBuffer[2] + 2); j++) {
        for (i = 7; i >= 0; i--) {
            if (device==1)
                IR1out = !((RxBuffer[j] >> i) & 1);
            else
                IR2out = !((RxBuffer[j] >> i) & 1);
            Delay100TCYx(20); // 0.5[usec]*100*20 = 1000[usec]. TCY = 8MHz/4 = 0.5[usec]
        }
    }
}