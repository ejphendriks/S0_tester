/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

// XC8 include files
#include <xc.h>             /* XC8 General Include File */
#include <stdint.h>         /* For uint8_t definition */
#include <stdbool.h>        /* For true/false definition */

// Program specific include files
#include "init.h"


//------------------------------------------------------------------------------
// Declare external variables   
//------------------------------------------------------------------------------

extern    char    buffer_UART1_RX[255];    // UART1 RX - RF RX buffer in RAM BANK 1
extern    char    buffer_UART1_TX[255];    // UART1 TX - RF TX buffer in RAM BANK 2

extern    char    buffer_UART2_RX[255];    // UART2 RX - RF RX buffer in RAM BANK 3
extern    char    buffer_UART2_TX[255];    // UART2 TX - RF TX buffer in RAM BANK 4


//------------------------------------------------------------------------------
// Peripheral assignment
//------------------------------------------------------------------------------


//    ; -------------------------------------
//    ; --- MASTER CLOCK - PCB layout
//    ; -------------------------------------
//    ; --- IR GATE PIN-OUT
//    ; --- PIC 18F46K22 at 4 x 16 MHz using Internal OSC
//    ; --- 40 pin DIP version
//    ; --------------------------------------
//    ; --- RA0 = INPUT (LDR)
//    ; --- RA1 = INPUT (DS18B20)
//    ; --- RA2 = LED RED
//    ; --- RA3 = LED GREEN 
//    ; --- RA4 = LED RED
//    ; --- RA5 = LED GREEN 
//    ; --- RA6 = OSC
//    ; --- RA7 = OSC
//    ; --------------------------------------
//    ; --- RB0 = n.c.
//    ; --- RB1 = n.c.
//    ; --- RB2 = n.c.
//    ; --- RB3 = n.c.
//    ; --- RB4 = n.c.
//    ; --- RB5 = HM-TRP Enable pin
//    ; --- RB6 = ICSP CLOCK / TX UART 2
//    ; --- RB7 = ICSP DATA / RX UART 2
//    ; --------------------------------------
//    ; --- RC0 = OSC 32kHz
//    ; --- RC1 = OSC 32kHz
//    ; --- RC2 = n.c.
//    ; --- RC3 = n.c.
//    ; --- RC4 = n.c.
//    ; --- RC5 = n.c.
//    ; --- RC6 = TX UART 1 --> USB --> PC
//    ; --- RC7 = RX UART 1 <-- USB <-- PC
//    ; --------------------------------------
//    ; --- RD0 = HM-TRP - ENABLE
//    ; --- RD1 = HM-TRP - CONFIG 
//    ; --- RD2 = n.c.
//    ; --- RD3 = n.c.
//    ; --- RD4 = n.c.
//    ; --- RD5 = n.c.
//    ; --- RD6 = TX UART 2 --> RF --> HM_TRP
//    ; --- RD7 = RX UART 2 <-- RF <-- HM_TRP
//    ; -------------------------------------
//    ; --------------------------------------
//    ; --- RE0 = n.c.
//    ; --- RE1 = n.c.
//    ; --- RE2 = n.c.
//    ; --- RE3 = MCLR
//    ; -------------------------------------
//    ; -------------------------------------



// -----------------------------------------------------------------------------
// --- Initialize Port IO ------------------------------------------------------
// -----------------------------------------------------------------------------
void    init_PORT_IO(void) 
{
        // Clear output latches 
    	LATA = 0b00000000;
        LATB = 0b00000000;
        LATC = 0b00000000;
        //LATD = 0b11011001;
        //LATE = 0b00000000;

        // LED's OFF
        LATAbits.LATA0 = 0;     // AUX --> BC549
        LATAbits.LATA1 = 0;     // LED RED
        LATAbits.LATA2 = 0;     // LED ORANGE
        LATAbits.LATA3 = 0;     // LED GREEN
        LATAbits.LATA4 = 0;     // LED YELLOW/BLUE

        // --------------------------------------------------
        // --- HM-TRP control lines CFG and ENABLE for ErwinO HUB
        // --------------------------------------------------
        // HM-TRP CONFIG on RD0 is set to DATACOM
        // LATDbits.LATD0 = 1;                 // (0=Config & 1=Datacom)
        // HM-TRP ENABLE on RD1 is set to ENABLE (inverted)
        // LATDbits.LATD1 = 0;                 // (0=On & 1=Off) Inverted!

        // --------------------------------------------------
        // --- HM-TRP control lines CFG and ENABLE for FTDI <--> MCU <--> HM-TRP (Van Raalte))
        // --------------------------------------------------
        // HM-TRP CONFIG on RD0 is set to DATACOM
        LATBbits.LATB5 = 1;                 // (0=Config & 1=Datacom)
        // HM-TRP ENABLE on RD1 is set to ENABLE (inverted)
        LATBbits.LATB4 = 0;                 // (0=On & 1=Off) Inverted!

        
        // Initialize Port IO
        
        TRISA = 0b11000000;                 // Port A - IO
        //TRISAbits.TRISA7 = 1;             // RA7 = n.c.
        //TRISAbits.TRISA6 = 1;             // RA6 = n.c.
        //TRISAbits.TRISA5 = 1;             // RA5 = n.c.
        //TRISAbits.TRISA4 = 0;             // RA4 = output LED YELLOW 
        //TRISAbits.TRISA3 = 0;             // RA3 = output LED GREEN
        //TRISAbits.TRISA2 = 0;             // RA2 = output LED ORANGE 
        //TRISAbits.TRISA1 = 0;             // RA1 = output LED RED
        //TRISAbits.TRISA0 = 1;             // RA0 = input ?
                
        TRISB = 0b11001111;                 // Port B - IO
        //TRISBbits.TRISB7 = 1;             // ICSP DAT and MCU RX <-- HM-TRP TX for 18F28K22
        //TRISBbits.TRISB6 = 1;             // ICSP CLCK and MCU TX --> HM-TRP RX for 18F28K22
        //TRISBbits.TRISB5 = 0;             // RB5 --> HM-TRP Configure
        //TRISBbits.TRISB4 = 0;             // RB4 --> HM-TRP Enable
        //TRISBbits.TRISB3 = 1;             // RB3 <-- input + pull-up to header & SWITCH
        //TRISBbits.TRISB2 = 1;             // RB2 <-- input + pull-up to header
        //TRISBbits.TRISB1 = 1;             // RB1 <-- input via Opto Coupler header: GND, RB0, GND, RB1, RB2, RB3
        //TRISBbits.TRISB0 = 1;             // RB0 <-- input via Opto Coupler header: GND, RB0, GND, RB1, RB2, RB3 

        TRISC = 0b11111111;                 // Port C - IO
        //TRISCbits.TRISC7 = 1;             // RC7 UART1 RX <-- TX USB <-- PC
        //TRISCbits.TRISC6 = 1;             // RC6 UART1 TX --> RX USB --> PC
        //TRISCbits.TRISC5 = 1;             // RC5 n.c.
        //TRISCbits.TRISC4 = 1;             // RC4 n.c.
        //TRISCbits.TRISC3 = 1;             // RC3 n.c.
        //TRISCbits.TRISC2 = 1;             // RC2 n.c.
        //TRISCbits.TRISC1 = 1;             // RC1 32 kHz Xtal
        //TRISCbits.TRISC0 = 1;             // RC0 32 kHz Xtal

        // Enable WPU internal pull-up resistors on PORT B
        WPUB = 0b11111111;                   // All PORTB

        // Enable WPU
        INTCON2bits.RBPU = 1;
}



// -----------------------------------------------------------------------------
// --- Set OSC parameters ------------------------------------------------------
// -----------------------------------------------------------------------------
void    init_OSC(void)
{

    // Select Internal OSC
	OSCCONbits.SCS1 = 0;                // AS THE PRIMARY CLOCK 
	OSCCONbits.SCS0 = 0;                

	// Set to 16 MHz
	OSCCONbits.IRCF2 = 1;                // 111 = 16 MHz
	OSCCONbits.IRCF1 = 1;
	OSCCONbits.IRCF0 = 1;

	// Enable OSCTUNE PLL 4 x Fosc --> 64 MHz
	PLLEN = 1;                          // Enable PLL 4 x Fosc for INT OSC
    
    // Tune internal RC osc 
    OSCTUNEbits.TUN5 = 0;
    OSCTUNEbits.TUN4 = 0;
    OSCTUNEbits.TUN3 = 0;
    OSCTUNEbits.TUN2 = 0;
    OSCTUNEbits.TUN1 = 0;
    OSCTUNEbits.TUN0 = 0;

}



//------------------------------------------------------------------------------
// Initialize ADC
//------------------------------------------------------------------------------
void    init_ADC(void)
{

    /* Setup analog functionality and port direction */

        ANSELA = 0b00000000;                // PORTA - Analogue pins
        // ANSELAbits.ANSA0 = 0;            // RA0 = input line
        // ANSELAbits.ANSA1 = 0;            // RA1 = input line
        // ANSELAbits.ANSA2 = 0;            // RA2 = input line
        // ANSELAbits.ANSA3 = 0;            // RA3 = LED GREEN UPPER
        // ANSELAbits.ANSA4 = 0;            // RA4 = LED BLUE
        // ANSELAbits.ANSA5 = 0;            // RA5 = HLVD = Vdd
        // ANSELAbits.ANSA6 = 0;            // RA6 = input line
        // ANSELAbits.ANSA7 = 0;            // RA7 = LED RED

        ANSELB = 0b00000000;                // All lines are digital
        ANSELC = 0b00000000;                // All lines are digital
        
        // ANSELD = 0b00000000;                // All lines are digital
        // ANSELE = 0b00000000;                // All lines are digital
}

//------------------------------------------------------------------------------
// Initialize TIMER 0 (8 or 16 bit TMR causes INT0 at overflow FF-->00 ) 
//------------------------------------------------------------------------------

void    init_TMR0(void)
{
        // TMR0 is used as a generic 100 mSec count down timer
        T0CONbits.TMR0ON = 1;               // TMR0 ON/OFF
        T0CONbits.T0SE = 0;                 // Source Edge select bit
        T0CONbits.T0CS = 0;                 // Clock Source Select bit (0 = internal)
        T0CONbits.T08BIT = 0;               // 8-bit/16-bit Control bit (0 = 16bit)
        T0CONbits.PSA = 0;                  // Pre-scaler Assignment bit (0 = pre-scaler is assigned)
        T0CONbits.T0PS2 = 1;                // Pre-scaler bits <0-2> 111 --> 1:256
        T0CONbits.T0PS1 = 1;                // Pre-scaler bits <0-2> 110 --> 1:128
        T0CONbits.T0PS0 = 1;                // Pre-scaler bits <0-2> 101 --> 1:64

        // TMR0 = 0; 16 bits counter, Pre-scaler 1:256, Fosc 64Mhz
        // Fosc             64,000,000 	Hz
        // Fosc/4			16,000,000 	Hz
        // Pre-scaler		256 --> 62,500 Hz
        // 16 bits reg      65,536 --> max delay  1.0485 Sec
        // Pre loading TMR0 with 59286 gives a delay of ~0,1 Sec (100 mSec))

        // Preload TMR0 = 59286 --> 6250 ticks --> 1.600.000 cycles--> 0,1 Sec delay
        // movlw		.231
        // movwf		TMR0H, ACCESS				; --- TMR0H needs to be loaded first
        // movlw		.150
        // movwf		TMR0L, ACCESS				; --- Copies TMR0H:TMR0L to actual TMR0 register

        TMR0H = 231;
        TMR0L = 150;

}


//------------------------------------------------------------------------------
// Initialize TMR 1/3/5 (16 bit internal or external clck source)
//------------------------------------------------------------------------------
void    init_TMR135(void)
{
    
        // TMR 1 = 32 kHz Xtal
        // TMR 3 = not used ?
        // TMR 5 = not used ?
    
    
        // Configure TMR 1 to use external 32 KHz Xtal OSC
        T1CONbits.TMR1CS1 = 1;          // Clock source select bit  0b10
        T1CONbits.TMR1CS0 = 0;          // Clock source select bit  0b10
        T1CONbits.T1CKPS1 = 0;          // Pre-scaler 1:1
        T1CONbits.T1CKPS0 = 0;          // Pre-scaler 1:1
        T1CONbits.T1OSCEN = 1;          // Enable external oscillator
        T1CONbits.NOT_T1SYNC = 1;       // Do not sync external oscillator
        T1CONbits.RD16 = 1;             // Read write 16 bit operations
        T1CONbits.TMR1ON = 0;           // Turn TMR1 ON / OFF

        // Gate Control
        T1GCONbits.TMR1GE = 0;          // Gate Control disabled
        
        // TMR1 is used with external 32kHz x-tal (1 Sec timer) 
        // Pre-load TMR1H:TMR1L with 16384 = 0b1000 0000 0000 0000 for 1 Sec
        TMR1H = 0b10000000;
        TMR1L = 0b00000000;

        // TODO TMR3

        // TMR 5 Control
        T5CONbits.TMR5ON = 0;           // TMR5 = OFF
        T5CONbits.T5CKPS1 = 1;          // Prescaler max (1:8)
        T5CONbits.T5CKPS0 = 1;          // Prescaler max (1:8)
        T5CONbits.T5SOSCEN = 0;         // Ext OSC disabled
        T5CONbits.T5RD16 = 1;           // Enable 16 bit read / write
}
//------------------------------------------------------------------------------
// Initialize TMR 2/4/6 (8 bit count down timers ) 
//------------------------------------------------------------------------------

void    init_TMR246()
{
        // TMR2 is used as a 1 mSec count down timer
        // TMR2 Prescaler = 1:16
        // TMR2 PostScaler = 1:10
        // PR2 = 100 at 64 MHz 
        // Freq = 1000.00 Hz
        // Period = 0.001000 seconds is 1 mSec

        //T2CON |= 72;                  // bits 6-3 Post scaler 1:10
        T2CONbits.T2OUTPS3 = 1;         // bits 6-3 Post scaler 1:10
        T2CONbits.T2OUTPS2 = 0;
        T2CONbits.T2OUTPS1 = 0;
        T2CONbits.T2OUTPS0 = 1;

        T2CONbits.T2CKPS1 = 1;          // bits 1-0  Pre-scaler 1:16
        T2CONbits.T2CKPS0 = 1;

        // PR2 = 100;                   // PR4 (Match value = 100 for 64MHz)
        PR2 = 104;                      // Set TMR 2 to 104 for calibrated 1 mSec period (1000 Hz)

        T2CONbits.TMR2ON = 0;           // Bit 2 turn timer 2  OFF

    
        // TMR4 is used for internal 1 mSec timer based on 64 MHz clock
        // TMR4 Prescaler = 1:16
        // TMR4 PostScaler = 1:10
        // PR2 = 100 for 64 MHz
        // Freq = 1000.00 Hz
        // Period = 0.001000 seconds is 1 mSec

        //T4CON |= 72;                  // bits 6-3 Post scaler 1:10
        T4CONbits.T4OUTPS3 = 1;         // bits 6-3 Post scaler 1:10
        T4CONbits.T4OUTPS2 = 0;
        T4CONbits.T4OUTPS1 = 0;
        T4CONbits.T4OUTPS0 = 1;

        T4CONbits.T4CKPS1 = 1;          // bits 1-0  Pre-scaler 1:16
        T4CONbits.T4CKPS0 = 1;

        // PR4 = 100;                   // PR4 (Match value = 100 for 64MHz)
        PR4 = 104;                      // Set TMR 4 to 100 for 1 mSec period (1000 Hz)

        T4CONbits.TMR4ON = 0;           // bit 2 turn timer 4  OFF

        // TMR6 not used
}


//------------------------------------------------------------------------------
// Initialize UART1 to 9600 BAUD, 8 BITS, NO PARITY  
//------------------------------------------------------------------------------

void    init_UART1(void)
{
        // Disable output
        TRISC7 = 1;                         // Disable output for UART
        TRISC6 = 1;                         // Disable output for UART

        // Receive 8 bits
        RCSTA1bits.RX9 = 0;                 // RX9: 9-bit Receive Enable bit
        // Receive 8 bits
        RCSTA1bits.SREN = 0;                // N.v.t only for Synchronous mode
        // Address Detect Enable bit
        RCSTA1bits.ADDEN = 0;               // Disables address detection, all bytes are received and ninth bit can be used as parity bit

    	// RCSTA1: FERR, OERR and RX9D are status bits

        // Use 8 bit
    	TXSTA1bits.TX9 = 0;                 // Use 8 bit --> 0
        // Async mode = 0
        TXSTA1bits.SYNC = 0;                // In ASync mode --> 0
        // Low speed --> BRGH = 0
        TXSTA1bits.BRGH = 1;                // Low speed --> 0

        // Use the 16 bit timer
        BAUDCON1bits.BRG16 = 1;             // Do NOT use the 16 bit timer for BRG
        // NO Invert RX signal
        BAUDCON1bits.DTRXP = 0;             // Receive Data Polarity Select bit for RX signal
        // NO Invert TX signal
        BAUDCON1bits.CKTXP = 0;             // Data Polarity Select bit for TX signal
        // Wake-up Enable bit
        BAUDCON1bits.WUE = 0;               // Wake-up Enable bit
        // NO Auto-Baud Detection
        BAUDCON1bits.ABDEN = 0;             // Auto-Baud Detect Enable bit

        // Set baudrate to 9600 @ 64MHz based on BRG16 = 0 (8 bit timer) --> d'103'
        // SPBRG1 = 103;

        // Set baudrate to 9600 @ 64MHz based on BRGH = 1, BRG16 = 1 
        // TABLE datasheet SPBRGH2:SPBRGH = 1666 --> 0x06 & 0x82
        SPBRGH1 = 0x06;
        SPBRG1 = 0x82;

        // Serial Port Enable bit
    	RCSTA1bits.SPEN = 1;                // Serial port enabled
        // Enable sending
        TXSTA1bits.TXEN = 1;                // Tx Enable --> 1
        // Continuous Receive ENable bit
        RCSTA1bits.CREN = 1;                // Continuous Receive ENable bit
        
}


//------------------------------------------------------------------------------
// Initialize UART2 is not used in IR GATE TX/RX
//------------------------------------------------------------------------------


void    init_UART2(void)
{
        // Disable output for 18F26K22
        TRISB7 = 1;                         // Disable output for UART2
        TRISB6 = 1;                         // Disable output for UART2
        // Disable output for 18F46K22
        // TRISD7 = 1;                         // Disable output for UART2
        // TRISD6 = 1;                         // Disable output for UART2
        
        // Receive 8 bits
        RCSTA2bits.RX9 = 0;                 // RX9: 9-bit Receive Enable bit
        // Receive 8 bits
        RCSTA2bits.SREN = 0;                // N.v.t only for Synchronous mode
        // Address Detect Enable bit
        RCSTA2bits.ADDEN = 0;               // Disables address detection, all bytes are received and ninth bit can be used as parity bit
        // RCSTA1: FERR, OERR and RX9D are status bits

        // Use 8 bit
        TXSTA2bits.TX9 = 0;                 // Use 8 bit --> 0
        // Async mode = 0
        TXSTA2bits.SYNC = 0;                // In ASync mode --> 0
        // Low speed --> BRGH = 0
        TXSTA2bits.BRGH = 1;                // High speed --> 1

        // Use the 16 bit timer
        BAUDCON2bits.BRG16 = 1;             // Use the 16 bit timer for highest accuracy
        // NO Invert RX signal
        BAUDCON2bits.DTRXP = 0;             // Receive Data Polarity Select bit for RX signal
        // NO Invert TX signal
        BAUDCON2bits.CKTXP = 0;             // Data Polarity Select bit for TX signal
        // Wake-up Enable bit
        BAUDCON2bits.WUE = 0;               // Wake-up Enable bit
        // NO Auto-Baud Detection
        BAUDCON2bits.ABDEN = 0;             // Auto-Baud Detect Enable bit

        // Set baudrate to 9600 @ 4x20MHz = 80MHz based on BRG16 = 0 (8 bit timer) --> d'129'
    	// SPBRG2 = 129;

        // Set baudrate to 9600 @ 64MHz based on BRG16 = 0 (8 bit timer) --> d'103'
        //SPBRG2 = 103;
        
        // Set baudrate to 9600 @ 64MHz based on BRGH = 1, BRG16 = 1 
        // TABLE datasheet SPBRGH2:SPBRGH = 1666 --> 0x06 & 0x82
        SPBRGH2 = 0x06;
        SPBRG2 = 0x82;

        // Serial Port Enable bit
        RCSTA2bits.SPEN = 1;                // Serial port enabled
        // Enable sending
        TXSTA2bits.TXEN = 1;                // TX Enable --> 1
        // Continuous Receive ENable bit
        RCSTA2bits.CREN = 1;                // Continuous RX Enable bit

}


//------------------------------------------------------------------------------
// Initialize Interrupts 
//------------------------------------------------------------------------------


void    init_INT(void)
{
        /* Configure the IPEN bit (1=on) in RCON to turn on/off int priorities */
        // ----- RCON Bits -----------------------------------------------------
            // NOT_BOR          EQU  H'0000'
            // NOT_POR          EQU  H'0001'
            // NOT_PD           EQU  H'0002'
            // NOT_TO           EQU  H'0003'
            // NOT_RI           EQU  H'0004'
            // SBOREN           EQU  H'0006'
            IPEN = 0;           //  INTERRUPT PRIORITY enable bit H'0006'
            // RCON = 0b00000000;

        // ----- INTCON Bits -----------------------------------------------------
            RBIF = 0;           // Port B Interrupt-On-Change (IOCx) Interrupt Flag bit - H'0000'
            INT0IF = 0;         // INT0 External Interrupt Flag bit - H'0001'
            TMR0IF = 0;         // TMR0 Overflow Interrupt Flag bit - H'0002'
            RBIE = 0;           // Port B Interrupt-On-Change (IOCx) Interrupt Enable bit - H'0003'
            INT0IE = 1;         // INT0 External Interrupt Enable bit - H'0004'
            TMR0IE = 1;         // TMR0 Overflow Interrupt Enable bit - H'0005'
            PEIE = 1;           // PEIE/GIEL: Peripheral Interrupt Enable bit - H'0006'
            GIEH = 1;           // Global Interrupt Enable - H'0007'
            // INTCON = 0b00000000;

        // ----- INTCON2 Bits -----------------------------------------------------
            RBIP = 0;           // RB Port Change Interrupt Priority bit - H'0000'
            TMR0IP = 0;         // TMR0 Overflow Interrupt Priority bit - H'0002'
            INTEDG2 = 0;        // External Interrupt 2 Edge Select bit - H'0004'
            INTEDG1 = 0;        // External Interrupt 1 Edge Select bit - H'0005'
            INTEDG0 = 0;        // External Interrupt 0 Edge Select bit - H'0006'
            NOT_RBPU = 0;       // INVERSE PORTB Pull-up Enable bit - H'0007'
            // INTCON2 = 0b00000000;
        
        // ----- INTCON3 Bits -----------------------------------------------------
            // INT1IF           EQU  H'0000'
            // INT2IF           EQU  H'0001'
            // INT1IE           EQU  H'0003'
            // INT2IE           EQU  H'0004'
            // INT1IP           EQU  H'0006'
            // INT2IP           EQU  H'0007'

            // INT1F            EQU  H'0000'
            // INT2F            EQU  H'0001'
            // INT1E            EQU  H'0003'
            // INT2E            EQU  H'0004'
            // INT1P            EQU  H'0006'
            // INT2P            EQU  H'0007'
            // INTCON3 = 0b00000000;

        // ----- PIR1 Bits -----------------------------------------------------
            // TMR1IF           EQU  H'0000'            /* 32 kHz x-tal */
            // TMR2IF           EQU  H'0001'
            // CCP1IF           EQU  H'0002'
            // SSP1IF           EQU  H'0003'
            // TX1IF            EQU  H'0004'
            // RC1IF            EQU  H'0005'
            // ADIF             EQU  H'0006'
            // SSPIF            EQU  H'0003'
            // TXIF             EQU  H'0004'
            // RCIF             EQU  H'0005'
            // PIR1 = 0b00000000;

        // ----- PIR2 Bits -----------------------------------------------------
            // CCP2IF           EQU  H'0000'
            // TMR3IF           EQU  H'0001'
            // HLVDIF           EQU  H'0002'
            // BCL1IF           EQU  H'0003'
            // EEIF             EQU  H'0004'
            // C2IF             EQU  H'0005'
            // C1IF             EQU  H'0006'
            // OSCFIF           EQU  H'0007'
            // LVDIF            EQU  H'0002'
            // BCLIF            EQU  H'0003'
            // PIR2 = 0b00000000;

        // ----- PIR3 Bits -----------------------------------------------------
            // TMR1GIF          EQU  H'0000'
            // TMR3GIF          EQU  H'0001'
            // TMR5GIF          EQU  H'0002'
            // CTMUIF           EQU  H'0003'
            // TX2IF            EQU  H'0004'
            // RC2IF            EQU  H'0005'
            // BCL2IF           EQU  H'0006'
            // SSP2IF           EQU  H'0007'
            // PIR3 = 0b00000000;

        // ----- PIR4 Bits -----------------------------------------------------
            // CCP3IF           EQU  H'0000'
            // CCP4IF           EQU  H'0001'
            // CCP5IF           EQU  H'0002'
            // PIR4 = 0b00000000;
                    
        // ---- PIR5 Bits -----------------------------------------------------
            // TMR4IF           EQU  H'0000'
            // TMR5IF           EQU  H'0001'
            // TMR6IF           EQU  H'0002'
            // PIR5 = 0b00000000;


        // ----- PIE1 Bits -----------------------------------------------------
            TMR1IE = 1;         // Enable TMR1 interrupt - EQU  H'0000'
            TMR2IE = 1;         // Enable TMR2 interrupt - EQU  H'0001' 
            // CCP1IE           EQU  H'0002'
            // SSP1IE           EQU  H'0003'
            // TX1IE            EQU  H'0004'
            // RC1IE            EQU  H'0005'
            // ADIE             EQU  H'0006'
            // SSPIE            EQU  H'0003'
            TXIE = 0;           // UART1 TX Int Enable - EQU  H'0004'
            RCIE = 1;           // UART1 RX Int Enable - EQU  H'0005'
            // PIE1 = 0b00000000;

        // ----- PIE2 Bits -----------------------------------------------------
            // CCP2IE           EQU  H'0000'
            TMR3IE = 1;         // TMR3 Enable
            // HLVDIE           EQU  H'0002'
            // BCL1IE           EQU  H'0003'
            // EEIE             EQU  H'0004'
            // C2IE             EQU  H'0005'
            // C1IE             EQU  H'0006'
            // OSCFIE           EQU  H'0007'
            // LVDIE            EQU  H'0002'
            // BCLIE            EQU  H'0003'
            // PIE2 = 0b00000000;

        // ----- PIE3 Bits -----------------------------------------------------
            // TMR1GIE = 1;     // Gate INT Enable bit
            // TMR3GIE          // Gate INT Enable bit
            // TMR5GIE = 1;     // Gate INT Enable bit
            // CTMUIE           EQU  H'0003'
            TX2IE = 0;          // UART2 TX Int Enable - EQU  H'0004'
            RC2IE = 1;          // UART2 RX Int Enable - EQU  H'0005'
            // BCL2IE           EQU  H'0006'
            // SSP2IE           EQU  H'0007'
            // PIE3 = 0b00100000;

        // ----- PIE4 Bits -----------------------------------------------------
            // CCP3IE           EQU  H'0000'
            // CCP4IE           EQU  H'0001'
            // CCP5IE           EQU  H'0002'
            // PIE4 = 0b00000000;

        // ----- PIE5 Bits -----------------------------------------------------
            TMR4IE = 1;         // Used for internal 1000 mSec timer
            TMR5IE = 1;         // Used to turn OFF RA2 after 0,025 Sec
            TMR6IE = 0;         // TMR 6 not used 
            // PIE5 = 0b00000000;


        // ----- IPR1 Bits -----------------------------------------------------
            // TMR1IP           EQU  H'0000'
            // TMR2IP           EQU  H'0001'
            // CCP1IP           EQU  H'0002'
            // SSP1IP           EQU  H'0003'
            // TX1IP            EQU  H'0004'
            // RC1IP            EQU  H'0005'
            // ADIP             EQU  H'0006'
            // SSPIP            EQU  H'0003'
            // TXIP             EQU  H'0004'
            // RCIP             EQU  H'0005'
            // IPR1 = 0b00000000;

        // ----- IPR2 Bits -----------------------------------------------------
            // CCP2IP           EQU  H'0000'
            // TMR3IP           EQU  H'0001'
            // HLVDIP           EQU  H'0002'
            // BCL1IP           EQU  H'0003'
            // EEIP             EQU  H'0004'
            // C2IP             EQU  H'0005'
            // C1IP             EQU  H'0006'
            // OSCFIP           EQU  H'0007'
            // LVDIP            EQU  H'0002'
            // BCLIP            EQU  H'0003'
            // IPR2 = 0b00000000;

        // ----- IPR3 Bits -----------------------------------------------------
            // TMR1GIP          EQU  H'0000'
            // TMR3GIP          EQU  H'0001'
            // TMR5GIP          EQU  H'0002'
            // CTMUIP           EQU  H'0003'
            // TX2IP            EQU  H'0004'
            // RC2IP            EQU  H'0005'
            // BCL2IP           EQU  H'0006'
            // SSP2IP           EQU  H'0007'
            // IPR3 = 0b00000000;        

        // ----- IPR4 Bits -----------------------------------------------------
            // CCP3IP           EQU  H'0000'
            // CCP4IP           EQU  H'0001'
            // CCP5IP           EQU  H'0002'
            // IPR4 = 0b00000000;

        // ----- IPR5 Bits -----------------------------------------------------
            // TMR4IP           EQU  H'0000'
            // TMR5IP           EQU  H'0001'
            // TMR6IP           EQU  H'0002'
            // IPR5 = 0b00000000;

}


//------------------------------------------------------------------------------
// Send data 
//------------------------------------------------------------------------------

void TX_UART1(char chr_TX1_DATA)     /* Send data to UART 1 and wait until done */
{

    NOP();

    TXREG1 = chr_TX1_DATA;          /* TX data over UART 1*/

    while(!TRMT1)                   /* While busy sending loop */
    {
        continue;
    }
    return;
}


//------------------------------------------------------------------------------
// Send data 
//------------------------------------------------------------------------------

void TX_UART2(char chr_TX2_DATA)    /* Send data to UART 2 and wait until done */
{

    NOP();

    TXREG2 = chr_TX2_DATA;          /* TX data over UART 1*/

    while(!TRMT2)                   /* While busy sending loop */
    {
        continue;
    }
    return;

}


//------------------------------------------------------------------------------
// Say hello: blink 4 LED's at startup 
//------------------------------------------------------------------------------

void init_HELLO(void)                     /* Say Hello - Flash LED's */
{
    
    // LED GREEN 
    LATAbits.LATA1 = 1;
    //__delay_ms(25);
    _delay(60000);
    LATAbits.LATA1 = 0;
    //__delay_ms(25);
    _delay(60000);
    // LED GREEN UPPER
    LATAbits.LATA1 = 1;
    //__delay_ms(25);
    _delay(60000);
    LATAbits.LATA1 = 0;
    //__delay_ms(25);
    _delay(60000);
    
    // LED ORANGE
    LATAbits.LATA2 = 1;
    //__delay_ms(25);
    _delay(60000);
    LATAbits.LATA2 = 0;
    //__delay_ms(25);
    _delay(60000);
    // LED GREEN UPPER
    LATAbits.LATA2 = 1;
    //__delay_ms(25);
    _delay(60000);
    LATAbits.LATA2 = 0;
    //__delay_ms(25);
    _delay(60000);

    // LED GREEN 
    LATAbits.LATA3 = 1;
    //__delay_ms(25);
    _delay(60000);
    LATAbits.LATA3 = 0;
    //__delay_ms(25);
    _delay(60000);
    // LED GREEN UPPER
    LATAbits.LATA3 = 1;
    //__delay_ms(25);
    _delay(60000);
    LATAbits.LATA3 = 0;
    //__delay_ms(25);
    _delay(60000);
    
    // LED YELLOW
    LATAbits.LATA4 = 1;
    //__delay_ms(25);
    _delay(60000);
    LATAbits.LATA4 = 0;
    //__delay_ms(25);
    _delay(60000);
    // LED GREEN UPPER
    LATAbits.LATA4 = 1;
    //__delay_ms(25);
    _delay(60000);
    LATAbits.LATA4 = 0;
    //__delay_ms(25);
    _delay(60000);

}

/******************************************************************************/
/* Delay Functions                                                             */
/******************************************************************************/

void sub_Delay_1S(void)           // Initialize the GPS module
{
    uint8_t     cntrLoop;               /* delay cntr 250 x 4 mSec = 1 Sec */

    for(cntrLoop = 0; cntrLoop < 250; cntrLoop++)
    {
        __delay_ms(5);
    }
}



//------------------------------------------------------------------------------
// Initialize buffers 
//------------------------------------------------------------------------------


void    init_BUFFERS(void)
{
    int     ptBUFFER;

    NOP();

    for(ptBUFFER = 0; ptBUFFER <= 255; ptBUFFER++) 
    {
        buffer_UART1_RX[ptBUFFER] = 0;    // UART1 RX - GPS string in RAM BANK 1
        buffer_UART1_TX[ptBUFFER] = 0;    // UART1 TX - DISP buffer in RAM BANK 2
        buffer_UART2_RX[ptBUFFER] = 0;    // UART1 RX - GPS string in RAM BANK 1
        buffer_UART2_TX[ptBUFFER] = 0;    // UART1 TX - DISP buffer in RAM BANK 2

    }


    /* Initialize TX buffer for UART 1 */
    buffer_UART1_TX[0]  = 'i';
    buffer_UART1_TX[1]  = 'o';
    buffer_UART1_TX[2]  = 'W';
    buffer_UART1_TX[3]  = 'o';
    buffer_UART1_TX[4]  = 'r';
    buffer_UART1_TX[5]  = 'k';
    buffer_UART1_TX[6]  = 's';
    buffer_UART1_TX[7]  = '.';
    buffer_UART1_TX[8]  = 'n';
    buffer_UART1_TX[9]  = 'l';
    buffer_UART1_TX[10] = '\r';
    buffer_UART1_TX[11] = '\n';
    buffer_UART1_TX[12] = 'U';
    buffer_UART1_TX[13] = 'A';
    buffer_UART1_TX[14] = 'R';
    buffer_UART1_TX[15] = 'T';
    buffer_UART1_TX[16] = '1';  
    buffer_UART1_TX[17] = ' ';
    buffer_UART1_TX[18] = '9';  
    buffer_UART1_TX[19] = '6';
    buffer_UART1_TX[20] = '0';  
    buffer_UART1_TX[21] = '0';
    buffer_UART1_TX[22] = '\r';  
    buffer_UART1_TX[23] = '\n';
    buffer_UART1_TX[24] = 'S';  
    buffer_UART1_TX[25] = '0';
    buffer_UART1_TX[26] = ' '; 
    buffer_UART1_TX[27] = 'p';
    buffer_UART1_TX[28] = 'u';
    buffer_UART1_TX[29] = 'l';
    buffer_UART1_TX[30] = 's';
    buffer_UART1_TX[31] = 'e';
    buffer_UART1_TX[32] = ' ';
    buffer_UART1_TX[33] = ' ';
    buffer_UART1_TX[34] = ' ';
    buffer_UART1_TX[35] = ' ';
    buffer_UART1_TX[36] = 'V';
    buffer_UART1_TX[37] = MASTER_VERSION_MSD;
    buffer_UART1_TX[38] = '.';
    buffer_UART1_TX[39] = MASTER_VERSION_LSD;
    buffer_UART1_TX[40] = '\r';
    buffer_UART1_TX[41] = '\n';
    buffer_UART1_TX[42] = 0;


    // SLAVE EVENT
    buffer_UART2_TX[0] = 'i';
    buffer_UART2_TX[1] = 'o';
    buffer_UART2_TX[2] = 'W';
    buffer_UART2_TX[3] = 'o';
    buffer_UART2_TX[4] = 'r';
    buffer_UART2_TX[5] = 'k';
    buffer_UART2_TX[6] = 's';
    buffer_UART2_TX[7] = ' ';       

    buffer_UART2_TX[8] = 'S';
    buffer_UART2_TX[9] = 'L';
    buffer_UART2_TX[10] = 'A';
    buffer_UART2_TX[11] = 'V';
    buffer_UART2_TX[12] = 'E';
    buffer_UART2_TX[13] = ' ';
    buffer_UART2_TX[14] = 'E';
    buffer_UART2_TX[15] = 'V';
    buffer_UART2_TX[16] = 'E';
    buffer_UART2_TX[17] = 'N';
    buffer_UART2_TX[18] = 'T';
    buffer_UART2_TX[19] = ' ';
    buffer_UART2_TX[20] = 'V';
    buffer_UART2_TX[21] = MASTER_VERSION_MSD;
    buffer_UART2_TX[22] = '.';
    buffer_UART2_TX[23] = MASTER_VERSION_LSD;
    buffer_UART2_TX[24] = '\r';
    buffer_UART2_TX[25] = '\n';
    buffer_UART2_TX[26] = 0;

}
