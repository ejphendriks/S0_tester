// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// --- AGILE XS SO SIMULATOR
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
//
//    ; ---  S0 telegram: ID:a:I:b:M1:c:d:M2:e:f:M3:g:h:M4:i:j:M5:k:l
//    ; -------------------------------------
//    ; --- PCB : USB <--> PIC18F26K22 <--> HM-TRP 
//    ; --- RCB : REV 1.2 - 03-01-2016
//    ; -------------------------------------
//    ; --- PIC 18F26K22 at 4 x 16 MHz using Internal OSC
//    ; --- 28 pin SOIC version
//    ; --------------------------------------
//    ; --- RA0 = INPUT (LDR)
//    ; --- RA1 = LED BLUE
//    ; --- RA2 = LED GREEN
//    ; --- RA3 = LED RED 
//    ; --- RA4 = LED IR
//    ; --- RA5 = n.c.
//    ; --- RA6 = OSC
//    ; --- RA7 = OSC
//    ; --------------------------------------
//    ; --- RB0 = INPUT OPTO
//    ; --- RB1 = INPUT OPTO
//    ; --- RB2 = INPUT - PULL-UP
//    ; --- RB3 = INPUT - PULL-UP + SWITCH
//    ; --- RB4 = HM-TRP Enable pin
//    ; --- RB5 = HM-TRP Configure pin
//    ; --- RB6 = ICSP CLOCK / TX UART 2 (HM-TRP)
//    ; --- RB7 = ICSP DATA / RX UART 2 (HM-TRP)
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
//    ; --- RE3 = MCLR
//    ; -------------------------------------
//    ; -------------------------------------


//****************************************************************************
// Revision history MASTER CLOCK - 18F46K22
//****************************************************************************

// Release V0.2 - 07-02-2016    INITIAL REALESE based on USB-MCU-RF PCB


//****************************************************************************
// Include files                                                              
//****************************************************************************

#include <xc.h>              // For XC8 compiler
#include <stdio.h>           // For standard function definition 
#include <stdint.h>          // For uint8_t definition 
#include <stdlib.h>          // For itoa() definition 
#include <stdbool.h>         // For true/false definition 

#include "delays.h"          // Delay functions 
#include "system.h"          // System functions and params, osc/peripheral  
#include "init.h"            // User funct/params, such as InitApp 
#include "i2a.h"             // My INT8 to ASCII routines: DATE/TIME conversion


//****************************************************************************
// Constant declaration                                                       
//****************************************************************************

// LED pin assignment 
#define LED_RA1_RED              1          // RA1
#define LED_RA2_ORANGE           2          // RA2
#define LED_RA3_GREEN            3          // RA3
#define LED_RA4_YELLOW           4          // RA4

//****************************************************************************
// State declaration                                                       
//****************************************************************************


// States for TIMER state machine 
#define STATE_TIMER_INIT            1
#define STATE_TIMER_SEND_INIT       2
#define STATE_TIMER_CHECK_IR        3
#define STATE_TIMER_SEND_RFS        4
#define STATE_TIMER_WAIT_FS         5
#define STATE_TIMER_RUNNING         6
#define STATE_TIMER_FINISHED        7
#define STATE_TIMER_SEND_TIME       8
#define STATE_TIMER_WAIT_ACK        9
#define STATE_TIMER_ERROR          10


// States for UART1 and UART2 TX state machine 
#define STATE_TX_INIT               1            // Initialize TX state machine
#define STATE_TX_WAIT               2            // Wait for start signal
#define STATE_TX_SEND               3            // Send data
#define STATE_TX_PENDING            4            // TX pending
#define STATE_TX_DONE               5            // TX Done
#define STATE_TX_ERROR              6            // TX Error




//****************************************************************************
// External Function declaration                                                       
//****************************************************************************


//****************************************************************************
// Function declaration                                                       
//****************************************************************************


// State machine that controls behavior of the application 
void    sm_TIMER(void);

// Initialize EEPROM and read EEPROM 
void    sub_READ_EEPROM(void);
void    sub_WRITE_EEPROM(void);

// Initialize UART SEND routines
void    vSEND_STR_UART1(void);
void    vSEND_BYTE_UART1(uint8_t);
void    vSEND_STR_UART2(void);
void    vSEND_BYTE_UART2(uint8_t);

// Send data from UART1 TX buffer via UART 2 --> RF
void    vSEND_STR_UART2_2_RF(void);
void    v_SEND_TMR1_SYNC(void);

// Error Handlers 
void    sub_ERR_UART1(void);
void    sub_ERR_UART2(void);

// Can not be moved because too many local variables in main.c 
void    init_VAR(void);

// Declare Delay 1 Sec routine
void    sub_Delay_1S(void);

// Declare LOG TIME function every whole second on TMR1 roll-over
void    sub_LOG_TIME(void);

// Reset TMR2 and flash GREEN LEDS
void    sub_GREEN_LED(void);

// Analyze incoming data on UART 1
void    sub_CHECK_DATA(void);

// Declare DISP function
void    sub_FINISH_TIME(void);

// Declare UPDATE BUFFER
void    sub_UPDATE_TX_BUFFER_TIME(void);
void    sub_UPDATE_TX_BUFFER_EVENT(void);

// Declare Device 32kHz function
uint8_t iDevide_328_328_327(int);

// Declare UART TX state machines
void    sm_UART1_TX(void);
void    sm_UART2_TX(void);


char    cBin2Hex_H(char);
char    cBin2Hex_L(char);
int8_t  iHex2Bin(char, char);

void    v_CREATE_S0_TELEGRAM(void);


//****************************************************************************
// Global buffer declaration                                                  
//****************************************************************************

    __bank(1) char buffer_UART1_RX[255];    // UART1 RX - UART1 RX buffer in RAM BANK 1
    __bank(2) char buffer_UART1_TX[255];    // UART1 TX - UART1 TX buffer in RAM BANK 2
    __bank(3) char buffer_UART2_RX[255];    // UART2 RX - UART2 RX buffer in RAM BANK 3
    __bank(4) char buffer_UART2_TX[255];    // UART2 TX - UART2 TX buffer in RAM BANK 4

//****************************************************************************
// Global struct and union declaration                                                
//****************************************************************************

    union REG_CONTROL 
    {
        unsigned char byte;
        struct 
        {
            unsigned SEC:1;             // 1 SEC has expired from 32kHz xtal
            unsigned START_RF:1;        // START RF signal
            unsigned LAP_RF:1;          // LAP RF signal
            unsigned FINISH_RF:1;       // FINISH RF signal
            unsigned TRIGGER:1;         // LOG TRIGGER EVENT
            unsigned LOG2USB:1;         // LOG TIMESTAMP 2 USB
            unsigned AUTO_SYNC:1;       // ENABLE AUTO SYNC
            unsigned SYNC:1;            // SET SYNC flag
        } bitv;
    } REG_CONTROL;

    // This should become obsolete from Version 2.1 onward
    union REG_UART 
    {
        unsigned char byte;
        struct 
        {
            unsigned RX1_NewData:1;         // New data packet received on RX1
            unsigned RX2_NewData:1;         // New data packet received on RX2
            unsigned RX1_InProgress:1;      // RX1 receiving packet in progress
            unsigned RX2_InProgress:1;      // RX2 receiving packet in progress
            unsigned TX1_DataSetReady:1;    // TX1 Data Ready for TX
            unsigned TX2_DataSetReady:1;    // TX2 Data Ready for TX 
            unsigned TX1_InProgress:1;        // Locked for GPS initialization 
            unsigned TX2_InProgress:1;        // Locked for ??? (not shared) 
        } bitv;
    } REG_UART;

    union REG_TEST 
    {
        unsigned char byte;
        struct 
        {
            unsigned LSB:1, b1:1, b2:1, b3:1, b4:1, b5:1, b6:1, MSB:1;
        } bitv;
    } REG_TEST;
    
    // SNAPSHOT of TMR1 (2 bytes)
    union
    {
         uint8_t uc_bytes[2];
         unsigned int ui_word;
    } union_SNAPSHOT_TMR1;
    
    // SNAPSHOT of MASTER SECONDS (2 bytes)
    union
    {
         uint8_t uc_bytes[2];
         unsigned int ui_word;
    } union_SNAPSHOT_SECONDS;

    
    
//****************************************************************************
// Global variable declaration                                                
//****************************************************************************

    unsigned int    iCntr;                                                      // For next loop
    
    char            cdtmr_S0_ON;              // Count Down timer for SO ON duration (TMR2)
    char            cdtmr_S0_OFF;             // Count Down timer for SO OFF duration (TMR2)
    char            cdtmr_UART1_OFF;          // Count Down timer UART1 data OFF (TMR2)
    
    unsigned char   c_RX2_DATA;
    unsigned char   strTMR1[6];
    
    unsigned char   uc_ID_0;                                                    // ID 0
    unsigned char   uc_ID_1;                                                    // ID 1
    unsigned char   uc_ID_2;                                                    // ID 2
    unsigned char   uc_ID_3;                                                    // ID 3

    // Delay timers
    uint8_t         tmr_CountDown_IR_VALIDE;                                    // Count Down timer for validation of IR BEAM
    uint8_t         tmr_CountDown_IR_BROKEN;                                    // Count Down timer for broken of IR BEAM
        
// TX BUFFERS
//    uint8_t         pntr_UART1_TX_BUFFER;                                     // Pointer to UART1 TX buffer
//    uint8_t         pntr_UART2_TX_BUFFER;                                     // Pointer to UART2 TX buffer

    // Count down counter for AUTO SYNC in TMR1 ISR
    int8_t          uc_AUTO_SYNC_COUNTER;
    
    // Variables for TIMER 1 - 32kHz x-tal
    int8_t          int_TMR1_MASTER_SEC_MSB;
    int8_t          int_TMR1_MASTER_SEC_LSB;

    // Variables for TIMER 1 - 32kHz x-tal
    int8_t          int_TMR1_Seconds;
    int8_t          int_TMR1_Minutes;
    int8_t          int_TMR1_Hours;

    // Milli seconds counters
    int8_t          int_SNAPSHOT_TMR4_mSec0;                                                  // mSec x 1 counter
    int8_t          int_SNAPSHOT_TMR4_mSec10;                                                 // mSec x 10 counter
    int8_t          int_SNAPSHOT_TMR4_mSec100;                                                 // mSec x 100 counter
    int8_t          int_SNAPSHOT_TMR4_Seconds;
    int8_t          int_SNAPSHOT_TMR4_Minutes;
    int8_t          int_SNAPSHOT_TMR4_Hours;
        
    // Variables for TIMER 4 - internal RC OCS
    int8_t          int_TMR4_mSec0;
    int8_t          int_TMR4_mSec10;
    int8_t          int_TMR4_mSec100;
    int8_t          int_TMR4_Seconds;
    int8_t          int_TMR4_Minutes;
    int8_t          int_TMR4_Hours;
    
    // Flag to start and stop the TIMER
    int8_t          bool_TMR1_MATCH_ON;
    
    // DELTA Correction Interval 
    uint8_t         ui8_DELTA_INTERVAL;  
    int8_t          i8_DELTA_VALUE;  

    // Test variable
    //char            cTest;

    
//****************************************************************************
// Structure declaration                                                      
//****************************************************************************

// Marco's for setting and clearing bits in a byte 
#define testbit(var, bit) ((var) & (1 <<(bit)));
#define setbit(var, bit) ((var) |= (1 << (bit)));
#define clrbit(var, bit) ((var) &= ~(1 << (bit)));
        
//****************************************************************************
// EEPROM DATA
//****************************************************************************

__EEPROM_DATA('W', '4', 'A', '1', 0xFF, 0xFF, 0x00, '?');
    
//****************************************************************************
// Main Program                                                               
//****************************************************************************

void main(void)                         // UU$W3S0*AB
{

    NOP();

    sub_READ_EEPROM();
    
    NOP();
    
    // iCntr = iHex2Bin('A', 'B');
    
    // TEST 
    // setbit(REG_CONTROL, 0);
    // setbit(REG_CONTROL, 1);
    // clrbit(REG_CONTROL, 0);
    // setbit(REG_CONTROL, 7);

    // Clear all flags
    REG_CONTROL.byte = 0;
    REG_UART.byte = 0;
    REG_TEST.byte = 0;
    
    // Configure the oscillator for the device 
    ConfigureOscillator();

    // Initialize I/O and MCU internal peripherals  
    init_PORT_IO();
    init_OSC();
    init_ADC();
    init_TMR0();
    init_TMR135();
    init_TMR246();
    init_UART1();
    init_UART2();        
    init_INT();
    
    // Initialize variables from main.c 
    //init_VAR();

    uc_AUTO_SYNC_COUNTER = 3;           // Initial AUTO SYNC after 3 seconds
    
    // Clear all BUFFERS {NULL} 
    init_BUFFERS();

    // Set version numbers
    //buffer_UART1_TX[34] = IR_RX_VERSION_MSD;
    //buffer_UART1_TX[36] = IR_RX_VERSION_LSD;

    // Initialize CONTROL register 
    // REG_CONTROL.byte = 0;                // Reset all 
    REG_CONTROL.bitv.SEC = 0;               // New SECond from 32kHz xtal
    REG_CONTROL.bitv.START_RF = 0;          // START CMD RF
    REG_CONTROL.bitv.LAP_RF = 0;            // LAP CMD RF
    REG_CONTROL.bitv.FINISH_RF = 0;         // FINISH CMD RF
    REG_CONTROL.bitv.TRIGGER = 0;           // SEND TRIGGER via UART1 --> RF
    REG_CONTROL.bitv.LOG2USB = 0;           // LOG TIMESTAMP --> USB
    REG_CONTROL.bitv.AUTO_SYNC = 0;         // ENABLE AUTO SYNC
    REG_CONTROL.bitv.SYNC = 0;              // SYNC 

    // Initialize UART register 
    // REG_UART.byte = 0;                   // Reset all 
    REG_UART.bitv.RX1_NewData = 0;          // UART 1 
    REG_UART.bitv.RX2_NewData = 0;          // UART 2 
    REG_UART.bitv.RX1_InProgress = 0;       // UART 1 
    REG_UART.bitv.RX2_InProgress = 0;       // UART 2 
    REG_UART.bitv.TX1_DataSetReady = 0;     // UART 1 
    REG_UART.bitv.TX2_DataSetReady = 0;     // UART 2 
    REG_UART.bitv.TX1_InProgress = 0;       // UART 1 
    REG_UART.bitv.TX2_InProgress = 0;       // UART 2 
    
    // For debugging
    // Real value is read in CONTROL_INIT
    // sub_WRITE_EEPROM();                          // Write data to EEPROM for testing

    // READ DEFAULT VALUES FROM EEPROM 
    // sub_READ_EEPROM();                              // Read configuration data from EEPROM

    
    // Say hello LED
    init_HELLO();                       // Say hello 
    init_HELLO();                       // Say hello 
    init_HELLO();                       // Say hello 
    init_HELLO();                       // Say hello 
    init_HELLO();                       // Say hello 
    init_HELLO();                       // Say hello     

    // UART 1 SHOW VERSION NUMBER --> USB PC 
    vSEND_STR_UART1();                  // Send string to UART 1
    // UART 1 SHOW VERSION NUMBER --> RF HM_TRP
    vSEND_STR_UART2();                  // Send string to UART 2

    NOP();
    
    v_CREATE_S0_TELEGRAM();             // Initialize UART1 buffer
    
    // Always start SM in INIT
//    state_CLOCK           = STATE_CLOCK_INIT;                                 // Initialize CLOCK State Machine 
//    state_GPS_MODULE      = STATE_GPS_INIT;                                   // Initialize GPS State Machine 
//    state_TIMER           = STATE_TIMER_INIT;                                 // Initialize RF State Machine 

    // Variables for TIMER 4 - internal RC OCS        
    int_TMR4_mSec0 = 0;
    int_TMR4_mSec10 = 0;
    int_TMR4_mSec100 = 0;
    int_TMR4_Seconds = 0;
    int_TMR4_Minutes = 0;
    int_TMR4_Hours = 0;

    int_SNAPSHOT_TMR4_mSec0 = 0;
    int_SNAPSHOT_TMR4_mSec10 = 0;
    int_SNAPSHOT_TMR4_mSec100 = 0;
    int_SNAPSHOT_TMR4_Seconds = 0;
    int_SNAPSHOT_TMR4_Minutes = 0;
    int_SNAPSHOT_TMR4_Hours = 0;
    
    // RESET MASTER CLOCK separate byte SECONDS 64k --> ~ 18 hours
    int_TMR1_MASTER_SEC_MSB = 0;
    int_TMR1_MASTER_SEC_LSB = 0;

    // RESET Internal clock
    int_TMR1_Seconds = 0;
    int_TMR1_Minutes = 0;
    int_TMR1_Hours = 0;

    // Pre-load TMR1H:TMR1L with 32768 = 0b1000 0000 0000 0000
    TMR1H = 0b10000000;         // Preset for TMR1 MSB register
    TMR1L = 0b00000000;         // Preset for TMR1 MSB register
        
    // Every 14 seconds subtract 1 clock cycle from TMR1
    ui8_DELTA_INTERVAL = 13;
    i8_DELTA_VALUE = -1;

    // Clear TIMER 2 and 4 
    TMR2 = 0;                   // Clear TMR2
    TMR4 = 0;                   // Clear TMR4
    TMR6 = 0;                   // Clear TMR6
    
    // Enable timers
    T1CONbits.TMR1ON = 0;       // Turn TMR1 ON - 32 kHz x-tal MASTER TIMER
    T4CONbits.TMR4ON = 1;       // Turn TMR4 ON - milli-second timer    
    
    // Used for 30 mSec count down timer
    T2CONbits.TMR2ON = 1;       // Turn TMR2 ON 
    
    // S0 LED and NPN off
    LATAbits.LATA3 = 0;
    LATAbits.LATA0 = 0;

    // Clear count down timer 
    cdtmr_S0_ON = 0;
    cdtmr_S0_OFF = 0;
    
    //***********************************************************************
    //                              MAIN LOOP                                
    //***********************************************************************

    NOP();
    
    while(1)                                     // MainLoop 
    {
        // The following interrupts are activated 
        // UART 1 RX interrupt for receiving RF data 

        NOP();
        
                
        // RBO --> LED BLUE
        if(PORTBbits.RB0 == 0)                  
        {
            // LATAbits.LATA1 = 1;
        }
        else
        {
            // LATAbits.LATA1 = 0;
        }


        // RB1 --> LED GREEN
        if(PORTBbits.RB1 == 0)                  
        {
            // LATAbits.LATA2 = 1;
        }
        else
        {
            // LATAbits.LATA2 = 0;
        }
        
        // RB2 --> LED RED
        if(PORTBbits.RB2 == 0)                  
        {
            // LATAbits.LATA2 = 1;                 // TOGGLE RED LED   
        }
        else
        {
            // LATAbits.LATA2 = 0;
        }

        // RB3 is ON-BOARD SWITCH
        if(PORTBbits.RB3 == 0)
        {
//            LATAbits.LATA3 = 1;
//            LATAbits.LATA0 = 1;
//            cdtmr_S0_ON = 30;             // Set timer to 30 mSec
//            T2CONbits.TMR2ON = 1;               // Turn TMR2 ON 
        }
        else
        {
            //LATAbits.LATA3 = 0;
            //LATAbits.LATA0 = 0;
        }
        
        // Check incoming UART 1 RX data
        //sub_CHECK_DATA();
        
        // The main TIMER state machine (controls appl behavior))
        //sm_TIMER();
        
        // TX new data to RF module
        //sm_UART1_TX();

        // TX new data - UART2 not used
        //sm_UART2_TX();

        // Deal with UART RX errors 
        sub_ERR_UART1();          // Recover from UART framing and overrun errors
        sub_ERR_UART2();          // Recover from UART framing and overrun errors
    }
}


/******************************************************************************/
/* Send TMR1 over UART2 to receiving counter                                  */
/******************************************************************************/


void v_SEND_TMR1_SYNC(void)
{
//    union
//    {
//         uint8_t uc_bytes[2];               // array of 2 bytes that make up TRM1H:TMR1L
//         unsigned int ui_word;              // integer that allows direct integer manipulation
//    } union_SNAPSHOT_TMR1;

    uint8_t             uc_Seconds;         // Used to make snapshot of the expired time
    unsigned char       uc_Seconds_H;       // Stores the Lo byte in ASCII representation
    unsigned char       uc_Seconds_L;       // Stores the Lo byte in ASCII representation

    
    // Sample data strings
    // U$ORG1S0#0000AB*CS
    // U$ORG1S0#00523B*CS
    // U$ORG1S0#2809FA*CS
    // U$ORG1S0#3B7DC1*CS
    
    //  0 '$'
    //  1 'O'     = 0RG ID 
    //  2 'O'     = 0RG ID 
    //  3 'O'     = 0RG ID 
    //  4 'O'     = 0RG ID 
    //  5 'C'     = COMMAND       (S = START and A = ABORT)
    //  6 'S'     = SUB-COMMAND
    //  7 '#'     = START TMR1  <<==== marker to start internal TMR1 clock !!!
    //  8 '0'     = HEX_H SEC
    //  9 '6'     = HEX_L SEC 
    // 11 'A'     = HEX_H TMR1_H
    // 12 'B'     = HEX_L TMR1_H
    // 13 'C'     = HEX_H TMR1_L
    // 14 'D'     = HEX_L TMR1_L
    // 15 '*'     = Separator
    // 16 'C'     = HEX_H CHECKSUM 
    // 17 'S'     = HEX_L CHECKSUM
    // 18 '\r'    = Carriage Return
    // 19 '\n'    = New Line

    
    vSEND_BYTE_UART2('$');
    vSEND_BYTE_UART2('O');
    vSEND_BYTE_UART2('R');
    vSEND_BYTE_UART2('G');
    vSEND_BYTE_UART2('1');
    vSEND_BYTE_UART2('S');
    vSEND_BYTE_UART2('0');
    
    // SEND TIMESTAMP MARKER
    
    TXREG2 = '#';               // Push TIMESTAMP marker to UART 
    
    // CREATE TIMESTAMP SNAPSHOT
    
    // TMR1 is running asynchronously while copying TMR1H and TMR1H 
    // XC8 will use two separate MOVFF instructions to copy TMR1
    // Therefor we need to check if the two registers are copied correctly
    // Otherwise this can cause inconsistencies when TMR1L carries over during copy
    // The potential error would be: 256 * 1/32768Hz = 7.8125 mSec 
    
    uc_Seconds = int_TMR1_Seconds;

    union_SNAPSHOT_TMR1.uc_bytes[1] = TMR1H;
    union_SNAPSHOT_TMR1.uc_bytes[0] = TMR1L;

    // Check if async TMR1 had a carry over in between copying TMR1H and TMR1L ?
    if(union_SNAPSHOT_TMR1.uc_bytes[1] != TMR1H)
    {
        union_SNAPSHOT_TMR1.uc_bytes[1] = TMR1H;         // Copy TMR1H again
        uc_Seconds = int_TMR1_Seconds;          // Could be altered in TMR1_ISR
    }
            
    // Convert TMR1 to 0..32768 by removing our 'own' overflow bit that we put there in ISR
    union_SNAPSHOT_TMR1.ui_word = union_SNAPSHOT_TMR1.ui_word - 32768;
    
    // Check if TRMT2 has overrun already?
    if (TRMT2)
    {
        NOP();
    }
    
    // Wait here while still busy sending 
    while(!TRMT2)           
    {
        continue;
    }

    vSEND_BYTE_UART2(cBin2Hex_H(uc_Seconds));                   // Hex Sec H
    vSEND_BYTE_UART2(cBin2Hex_L(uc_Seconds));                   // Hex Sec L
    
    vSEND_BYTE_UART2(cBin2Hex_H(union_SNAPSHOT_TMR1.uc_bytes[1]));       // Hex TMR1H_H
    vSEND_BYTE_UART2(cBin2Hex_L(union_SNAPSHOT_TMR1.uc_bytes[1]));       // Hex TMR1H_L
    
    vSEND_BYTE_UART2(cBin2Hex_H(union_SNAPSHOT_TMR1.uc_bytes[0]));       // Hex TMR1L_H
    vSEND_BYTE_UART2(cBin2Hex_L(union_SNAPSHOT_TMR1.uc_bytes[0]));       // Hex TMR1L_L
    
    vSEND_BYTE_UART2('*');
    
    vSEND_BYTE_UART2('C');
    vSEND_BYTE_UART2('S');
    
    vSEND_BYTE_UART2('\r');
    vSEND_BYTE_UART2('\n');
    
}

//******************************************************************************
// Convert Hex to binary (8 bit))
//******************************************************************************

int8_t iHex2Bin(char cHex_H, char cHex_L)
{
    
    int8_t iRetVal;                                // Return value
    int8_t iHex_H;                                 // 
    int8_t iHex_L;                                 // 
        
    // HEX_H for 0..9
    if((cHex_H >= '0') && (cHex_H <= '9'))
    {
        iHex_H = cHex_H - '0';                    // Return 0..9
    }
    // HEX_H for A..F
    if((cHex_H >= 'A') && (cHex_H <= 'F')) 
    {
        iHex_H = cHex_H - 55;                    // Return 10..15 (55 is offset in ASCII table))
    }
    
    // HEX_L for 0..9
    if((cHex_L >= '0') && (cHex_L <= '9'))
    {
        iHex_L = cHex_L - '0';                    // Return 0..9
    }
    // HEX_L for A..F
    if((cHex_L >= 'A') && (cHex_L <= 'F')) 
    {
        iHex_L = cHex_L - 55;                    // Return 10..15 (55 is offset in ASCII table))
    }
    
    // Return the added hex values
    iRetVal = ((iHex_H << 4) | iHex_L);
    
    return iRetVal;
        
}


//******************************************************************************
// Convert binary to hex (Hign Nibble)
//******************************************************************************

char cBin2Hex_H(char value)
{
    
    value = (value >> 4) | (value << 4);            // Uses SWAPF in assembler !
    value = value & 0x0F;
    
    if (value - 10 >= 0)
    {
        value = value + 55;             // 55 is offset in ASCCI table to get to 'A'..'F'
        return value;
    }
    else
    {
        value = value + '0';            // '0' is offset in ASCCI table to get to '0'..'9'
        return value;
    }
        
}

//*****************************************************************************/
// Convert binary to hex (Low Nibble)
//*****************************************************************************/

char cBin2Hex_L(char value)
{
    
    value = value & 0x0F;
    
    if (value - 10 >= 0)
    {
        value = value + 55;             // 55 is offset in ASCCI table to get to 'A'..'F'
        return value;
    }
    else
    {
        value = value + '0';            // '0' is offset in ASCCI table to get to '0'..'9'
        return value;
    }
    
}





// -----------------------------------------------------------------------------
// SEND OUT FINISH TIME 
// -----------------------------------------------------------------------------

void sub_CHECK_DATA(void)
{
    // Responding to START and STOP
    if(REG_UART.bitv.RX1_NewData == 1)
    {
        REG_UART.bitv.RX1_NewData = 0;                     // Clear flag

        // Check UART 1 received data string UU$W3S0*AB
        if(buffer_UART1_RX[0] == '$')
        {
            if(buffer_UART1_RX[1] == 'W')
            {
                if(buffer_UART1_RX[2] == '3')
                {
                    if(buffer_UART1_RX[3] == 'S')
                    {
                        REG_CONTROL.bitv.START_RF = 1;
                    }
                    else if (buffer_UART1_RX[3] == 'F') 
                    {
                        REG_CONTROL.bitv.FINISH_RF = 1;
                    }                            
                }
            }
        }
    }
}


// -----------------------------------------------------------------------------
// UPDATE_TX_BUFFER_TIME (NON BLOCKING TX))
// -----------------------------------------------------------------------------

void sub_UPDATE_TX_BUFFER_TIME(void)           // send TMR 1 & 4 delta to RF
{
    // Local variables
    uint8_t             uc_Hours;
    uint8_t             uc_Minutes;
    uint8_t             uc_Seconds;
    uint8_t             uc_TMR1_mSec100;
    char                str_ASCII_TMR1[5];
    
    
    NOP();
        
    // Initialize buffer string
    str_ASCII_TMR1[0] = ' ';
    str_ASCII_TMR1[1] = ' ';
    str_ASCII_TMR1[2] = ' ';
    str_ASCII_TMR1[3] = ' ';
    str_ASCII_TMR1[4] = ' ';
            
    // TMR1 is running asynchronously while copying TMR1H and TMR1H 
    // XC8 will use two separate MOVFF instructions to copy TMR1
    // Therefor we need to check if the two registers are copied correctly
    // Otherwise this can cause inconsistencies when TMR1L carries over during copy
    // The potential error would be: 256 * 1/32768Hz = 7.8125 mSec 

    // THIS HAS MOVED TO the RB0-INT0 ISR
    
//    // Copy both SECONDS 8 bit registers
//    union_SNAPSHOT_SECONDS.uc_bytes[1] = int_TMR1_MASTER_SEC_MSB;
//    union_SNAPSHOT_SECONDS.uc_bytes[0] = int_TMR1_MASTER_SEC_LSB;
//    
//    // Copy both TMR1 8 bit registers
//    union_SNAPSHOT_TMR1.uc_bytes[1] = TMR1H;
//    union_SNAPSHOT_TMR1.uc_bytes[0] = TMR1L;
//    
//    // Check for TMR1 carry over
//    if(union_SNAPSHOT_TMR1.uc_bytes[1] != TMR1H)
//    {
//        union_SNAPSHOT_TMR1.uc_bytes[1] = TMR1H;
//    }
//    
//    // Check for SECONDS carry over
//    if(union_SNAPSHOT_SECONDS.uc_bytes[1] != int_TMR1_MASTER_SEC_MSB)
//    {
//        union_SNAPSHOT_SECONDS.uc_bytes[1] = int_TMR1_MASTER_SEC_MSB;
//    }
//            
    // Clear all counter
    uc_Hours = 0;
    uc_Minutes = 0;
    uc_Seconds = 0;
    
    // Calculate hours that have past since start
    while(union_SNAPSHOT_SECONDS.ui_word > 3600)
    {
        union_SNAPSHOT_SECONDS.ui_word = union_SNAPSHOT_SECONDS.ui_word - 3600;
        uc_Hours++;
    }
    
    // Calculate minutes that have past since start
    while(union_SNAPSHOT_SECONDS.ui_word > 60)
    {
        union_SNAPSHOT_SECONDS.ui_word = union_SNAPSHOT_SECONDS.ui_word - 60;
        uc_Minutes++;
    }
    
    // What is remaining is the number of seconds (minus 1 because it starts at 0)
    uc_Seconds = union_SNAPSHOT_SECONDS.ui_word - 1;

    // Convert TMR1 to 0..32768 by removing the most significant (overflow) bit 
    union_SNAPSHOT_TMR1.ui_word = union_SNAPSHOT_TMR1.ui_word - 32768;

    // Calculate the number of 100th of seconds
    // Devide TMR1 register by 328, 328, 327 etc...
    uc_TMR1_mSec100 = iDevide_328_328_327(union_SNAPSHOT_TMR1.ui_word);
    
    // Create ASCII representation of TMR1
    itoa(str_ASCII_TMR1, union_SNAPSHOT_TMR1.ui_word, 10); 
    
    // Replace \NULL with <SPACE> for printing to UART
    if(str_ASCII_TMR1[4] == 0)
    {
        str_ASCII_TMR1[4] = ' ';
    }
    if(str_ASCII_TMR1[3] == 0)
    {
        str_ASCII_TMR1[3] = ' ';
    }
    if(str_ASCII_TMR1[2] == 0)
    {
        str_ASCII_TMR1[2] = ' ';
    }
    if(str_ASCII_TMR1[1] == 0)
    {
        str_ASCII_TMR1[1] = ' ';
    }
    if(str_ASCII_TMR1[0] == 0)
    {
        str_ASCII_TMR1[0] = ' ';
    }
        
    buffer_UART1_TX[0]  = 'M';
    buffer_UART1_TX[1]  = 'A';
    buffer_UART1_TX[2]  = 'S';
    buffer_UART1_TX[3]  = 'T';    
    buffer_UART1_TX[4]  = 'E';
    buffer_UART1_TX[5]  = 'R';
    buffer_UART1_TX[6]  = ' ';
    buffer_UART1_TX[7]  = ':';
    buffer_UART1_TX[8]  = ' ';
    buffer_UART1_TX[9]  = cBin2Hex_H(union_SNAPSHOT_SECONDS.uc_bytes[1]);
    buffer_UART1_TX[10] = cBin2Hex_L(union_SNAPSHOT_SECONDS.uc_bytes[1]);
    buffer_UART1_TX[11] = cBin2Hex_H(union_SNAPSHOT_SECONDS.uc_bytes[0]);
    buffer_UART1_TX[12] = cBin2Hex_L(union_SNAPSHOT_SECONDS.uc_bytes[0]);
    buffer_UART1_TX[13] = cBin2Hex_H(union_SNAPSHOT_TMR1.uc_bytes[1]);
    buffer_UART1_TX[14] = cBin2Hex_L(union_SNAPSHOT_TMR1.uc_bytes[1]);
    buffer_UART1_TX[15] = cBin2Hex_H(union_SNAPSHOT_TMR1.uc_bytes[0]);
    buffer_UART1_TX[16] = cBin2Hex_L(union_SNAPSHOT_TMR1.uc_bytes[0]);
    buffer_UART1_TX[17] = ' ';
    buffer_UART1_TX[18] = '-';
    buffer_UART1_TX[19] = ' ';
    buffer_UART1_TX[20] = i2a010(uc_Hours);
    buffer_UART1_TX[21] = i2a001(uc_Hours);
    buffer_UART1_TX[22] = ':';
    buffer_UART1_TX[23] = i2a010(uc_Minutes);
    buffer_UART1_TX[24] = i2a001(uc_Minutes);
    buffer_UART1_TX[25] = ':';
    buffer_UART1_TX[26] = i2a010(uc_Seconds);
    buffer_UART1_TX[27] = i2a001(uc_Seconds);
    buffer_UART1_TX[28] = '.';
    buffer_UART1_TX[29] = i2a010(uc_TMR1_mSec100);
    buffer_UART1_TX[30] = i2a001(uc_TMR1_mSec100);
    buffer_UART1_TX[31] = ' ';
    buffer_UART1_TX[32] = 'T';
    buffer_UART1_TX[33] = 'M';
    buffer_UART1_TX[34] = 'R';
    buffer_UART1_TX[35] = '1';
    buffer_UART1_TX[36] = ':';
    buffer_UART1_TX[37] = ' ';
    buffer_UART1_TX[38] = str_ASCII_TMR1[0];
    buffer_UART1_TX[39] = str_ASCII_TMR1[1];
    buffer_UART1_TX[40] = str_ASCII_TMR1[2];
    buffer_UART1_TX[41] = str_ASCII_TMR1[3];
    buffer_UART1_TX[42] = str_ASCII_TMR1[4];
    buffer_UART1_TX[43] = ' ';
    buffer_UART1_TX[44] = ' ';
    buffer_UART1_TX[45] = ' ';
    buffer_UART1_TX[46] = ' ';
    buffer_UART1_TX[47] = '\r';
    buffer_UART1_TX[48] = '\n';
    
}





// -----------------------------------------------------------------------------
// UPDATE_TX_BUFFER_EVENT (BLOCKING TX)
// -----------------------------------------------------------------------------

void sub_UPDATE_TX_BUFFER_EVENT(void)           // send TMR 1 & 4 delta to RF
{
    // Local variables
    uint8_t             uc_Hours;
    uint8_t             uc_Minutes;
    uint8_t             uc_Seconds;
    uint8_t             uc_TMR1_mSec100;
    char                str_ASCII_TMR1[5];
    
    
    NOP();
        
    // Initialize buffer string
    str_ASCII_TMR1[0] = ' ';
    str_ASCII_TMR1[1] = ' ';
    str_ASCII_TMR1[2] = ' ';
    str_ASCII_TMR1[3] = ' ';
    str_ASCII_TMR1[4] = ' ';
            
    // TMR1 is running asynchronously while copying TMR1H and TMR1H 
    // XC8 will use two separate MOVFF instructions to copy TMR1
    // Therefor we need to check if the two registers are copied correctly
    // Otherwise this can cause inconsistencies when TMR1L carries over during copy
    // The potential error would be: 256 * 1/32768Hz = 7.8125 mSec 

    // THIS HAS MOVED TO the RB0-INT0 ISR
    
//    // Copy both SECONDS 8 bit registers
//    union_SNAPSHOT_SECONDS.uc_bytes[1] = int_TMR1_MASTER_SEC_MSB;
//    union_SNAPSHOT_SECONDS.uc_bytes[0] = int_TMR1_MASTER_SEC_LSB;
//    
//    // Copy both TMR1 8 bit registers
//    union_SNAPSHOT_TMR1.uc_bytes[1] = TMR1H;
//    union_SNAPSHOT_TMR1.uc_bytes[0] = TMR1L;
//    
//    // Check for TMR1 carry over
//    if(union_SNAPSHOT_TMR1.uc_bytes[1] != TMR1H)
//    {
//        union_SNAPSHOT_TMR1.uc_bytes[1] = TMR1H;
//    }
//    
//    // Check for SECONDS carry over
//    if(union_SNAPSHOT_SECONDS.uc_bytes[1] != int_TMR1_MASTER_SEC_MSB)
//    {
//        union_SNAPSHOT_SECONDS.uc_bytes[1] = int_TMR1_MASTER_SEC_MSB;
//    }
//            
    // Clear all counter
    uc_Hours = 0;
    uc_Minutes = 0;
    uc_Seconds = 0;
    
    // Calculate hours that have past since start
    while(union_SNAPSHOT_SECONDS.ui_word > 3600)
    {
        union_SNAPSHOT_SECONDS.ui_word = union_SNAPSHOT_SECONDS.ui_word - 3600;
        uc_Hours++;
    }
    
    // Calculate minutes that have past since start
    while(union_SNAPSHOT_SECONDS.ui_word > 60)
    {
        union_SNAPSHOT_SECONDS.ui_word = union_SNAPSHOT_SECONDS.ui_word - 60;
        uc_Minutes++;
    }
    
    // What is remaining is the number of seconds (minus 1 because it starts at 0)
    uc_Seconds = union_SNAPSHOT_SECONDS.ui_word - 1;

    // Convert TMR1 to 0..32768 by removing the most significant (overflow) bit 
    union_SNAPSHOT_TMR1.ui_word = union_SNAPSHOT_TMR1.ui_word - 32768;

    // Calculate the number of 100th of seconds
    // Devide TMR1 register by 328, 328, 327 etc...
    uc_TMR1_mSec100 = iDevide_328_328_327(union_SNAPSHOT_TMR1.ui_word);
    
    // Create ASCII representation of TMR1
    itoa(str_ASCII_TMR1, union_SNAPSHOT_TMR1.ui_word, 10); 
    
    // Replace \NULL with <SPACE> for printing to UART
    if(str_ASCII_TMR1[4] == 0)
    {
        str_ASCII_TMR1[4] = ' ';
    }
    if(str_ASCII_TMR1[3] == 0)
    {
        str_ASCII_TMR1[3] = ' ';
    }
    if(str_ASCII_TMR1[2] == 0)
    {
        str_ASCII_TMR1[2] = ' ';
    }
    if(str_ASCII_TMR1[1] == 0)
    {
        str_ASCII_TMR1[1] = ' ';
    }
    if(str_ASCII_TMR1[0] == 0)
    {
        str_ASCII_TMR1[0] = ' ';
    }
        
    buffer_UART1_TX[0]  = '$';
    buffer_UART1_TX[1]  = 'W';
    buffer_UART1_TX[2]  = '4';
    buffer_UART1_TX[3]  = 'A';    
    buffer_UART1_TX[4]  = '2';      // 0 = MASTER, 1 = FINISH, 2 is START, 3,4,5 are LAP time 
    buffer_UART1_TX[5]  = 'R';
    buffer_UART1_TX[6]  = 'C';
    buffer_UART1_TX[7]  = ' ';
    buffer_UART1_TX[8]  = '#';
    buffer_UART1_TX[9]  = cBin2Hex_H(union_SNAPSHOT_SECONDS.uc_bytes[1]);
    buffer_UART1_TX[10] = cBin2Hex_L(union_SNAPSHOT_SECONDS.uc_bytes[1]);
    buffer_UART1_TX[11] = cBin2Hex_H(union_SNAPSHOT_SECONDS.uc_bytes[0]);
    buffer_UART1_TX[12] = cBin2Hex_L(union_SNAPSHOT_SECONDS.uc_bytes[0]);
    buffer_UART1_TX[13] = cBin2Hex_H(union_SNAPSHOT_TMR1.uc_bytes[1]);
    buffer_UART1_TX[14] = cBin2Hex_L(union_SNAPSHOT_TMR1.uc_bytes[1]);
    buffer_UART1_TX[15] = cBin2Hex_H(union_SNAPSHOT_TMR1.uc_bytes[0]);
    buffer_UART1_TX[16] = cBin2Hex_L(union_SNAPSHOT_TMR1.uc_bytes[0]);
    buffer_UART1_TX[17] = '*';
    buffer_UART1_TX[18] = ' ';
    buffer_UART1_TX[19] = ' ';
    buffer_UART1_TX[20] = i2a010(uc_Hours);
    buffer_UART1_TX[21] = i2a001(uc_Hours);
    buffer_UART1_TX[22] = ':';
    buffer_UART1_TX[23] = i2a010(uc_Minutes);
    buffer_UART1_TX[24] = i2a001(uc_Minutes);
    buffer_UART1_TX[25] = ':';
    buffer_UART1_TX[26] = i2a010(uc_Seconds);
    buffer_UART1_TX[27] = i2a001(uc_Seconds);
    buffer_UART1_TX[28] = '.';
    buffer_UART1_TX[29] = i2a010(uc_TMR1_mSec100);
    buffer_UART1_TX[30] = i2a001(uc_TMR1_mSec100);
    buffer_UART1_TX[31] = ' ';
    buffer_UART1_TX[32] = 'T';
    buffer_UART1_TX[33] = 'M';
    buffer_UART1_TX[34] = 'R';
    buffer_UART1_TX[35] = '1';
    buffer_UART1_TX[36] = ':';
    buffer_UART1_TX[37] = ' ';
    buffer_UART1_TX[38] = str_ASCII_TMR1[0];
    buffer_UART1_TX[39] = str_ASCII_TMR1[1];
    buffer_UART1_TX[40] = str_ASCII_TMR1[2];
    buffer_UART1_TX[41] = str_ASCII_TMR1[3];
    buffer_UART1_TX[42] = str_ASCII_TMR1[4];
    buffer_UART1_TX[43] = ' ';
    buffer_UART1_TX[44] = ' ';
    buffer_UART1_TX[45] = ' ';
    buffer_UART1_TX[46] = ' ';
    buffer_UART1_TX[47] = '\r';
    buffer_UART1_TX[48] = '\n';
    
}


// -----------------------------------------------------------------------------
// SEND OUT FINISH TIME (BLOCKING) 
// -----------------------------------------------------------------------------

void sub_FINISH_TIME(void)           // send TMR 1 & 4 delta to RF
{
    // Local variables
    uint8_t     uc_TMR1_mSec100;
    
    // Initialize
    uc_TMR1_mSec100 = 0;
            
    NOP();

    // Device TMR1 register by 328, 328, 327 etc...
    uc_TMR1_mSec100 = iDevide_328_328_327(TMR1);

    // Send bytes to RF modules
    vSEND_BYTE_UART1('$');      // START
    vSEND_BYTE_UART1('W');      // TEAM
    vSEND_BYTE_UART1('3');      // BAAN
    vSEND_BYTE_UART1('D');      // DISP
    
    vSEND_BYTE_UART1(i2a010(int_TMR1_Seconds));
    vSEND_BYTE_UART1(i2a001(int_TMR1_Seconds));
    vSEND_BYTE_UART1('.');
    vSEND_BYTE_UART1(i2a010(uc_TMR1_mSec100));
    vSEND_BYTE_UART1(i2a001(uc_TMR1_mSec100));
    
//    vSEND_BYTE_UART1(int_TMR1_Seconds);
//    vSEND_BYTE_UART1(uc_TMR1_mSec100);
    
    vSEND_BYTE_UART1('*');      // END
    vSEND_BYTE_UART1('A');      // CRC
    vSEND_BYTE_UART1('B');      // CRC
    vSEND_BYTE_UART1('\r');
    vSEND_BYTE_UART1('\n');
}


// -----------------------------------------------------------------------------
// --- LOG TIME EVERY WHOLE SECOND
// -----------------------------------------------------------------------------

uint8_t iDevide_328_328_327(int int_TMR1)
{
    uint8_t  int_mSec100_Counter;
    
    // Clear int_mSec100_Counter 
    int_mSec100_Counter = 0;
    
    // TMR1 is 2^16 bit wide --> that would be 2 seconds
    // int_TMR1 = int_TMR1 - 32768;
    
    // From here on int_TMR1 is always positive
    
    if (int_TMR1 < 328)
    {
        return 0;          
    }
    
    // Subtract the following 328, 328, 327 etc... 
    // To get to 1/100th of a mSec from the TMR1 32768 Hz counter
    while(1)
    {
        if (int_TMR1 >= 328)
        {
            int_TMR1 = int_TMR1 - 328;
            int_mSec100_Counter++;
        }
        else
        {
            return int_mSec100_Counter;
        }

        if (int_TMR1 >= 328)
        {
            int_TMR1 = int_TMR1 - 328;
            int_mSec100_Counter++;
        }
        else
        {
            return int_mSec100_Counter;
        }

        if (int_TMR1 >= 327)
        {
            int_TMR1 = int_TMR1 - 327;
            int_mSec100_Counter++;
        }
        else
        {
            return int_mSec100_Counter;
        }
    }
    return 0;   // Should never come here
}






//****************************************************************************
// SEND DATA to UART                                                          
//****************************************************************************
// Pre-defined states for UART TX state machines                              
// STATE_TX_INIT                0x01            Initialize TX state machine   
// STATE_TX_WAIT                0x02            Wait for start signal         
// STATE_TX_SEND                0x03            Send data                     
// STATE_TX_PENDING             0x04            TX pending                    
// STATE_TX_ERROR               0x05            TX Error                      
//****************************************************************************
// UART 1 TX BUFFER[255] in RAM BANK 2                                        
//****************************************************************************

void vSEND_STR_UART1(void)
{
    // pointer for TX buffer
    uint8_t     pntr_UART1_TX_BUFFER;

    // Initialize pointer to buffer...
    pntr_UART1_TX_BUFFER = 0;

    // Send string terminated with 0 or max length <= 64
    while((buffer_UART1_TX[pntr_UART1_TX_BUFFER] != 0) && (pntr_UART1_TX_BUFFER <= 64)) 
    {
        // Send byte to TX1
        vSEND_BYTE_UART1(buffer_UART1_TX[pntr_UART1_TX_BUFFER]);
        // Next char from string
        pntr_UART1_TX_BUFFER++;                                                    // next char
    }
}


//****************************************************************************
// UART1 TX BYTE                                                              
//****************************************************************************

void vSEND_BYTE_UART1(char cBYTE)
{
    // Push data to UART 
    TXREG1 = cBYTE;

    while(!TRMT1)              // Wait here while busy sending 
    {
        NOP();
    }
}



//****************************************************************************
// SEND DATA to UART 2 --> RF MODULE                                                         
//****************************************************************************
// UART 2 TX BUFFER[255] in RAM BANK 2                                        
//****************************************************************************

void vSEND_STR_UART2(void)
{
    // pointer for TX buffer
    uint8_t     ptUART2_TX_BUFFER;

    // Initialize pointer to buffer...
    ptUART2_TX_BUFFER = 0;

    // Send string terminated with 0 or max length <= 64
    while((buffer_UART2_TX[ptUART2_TX_BUFFER] != 0) || (ptUART2_TX_BUFFER <= 64)) 
    {
        vSEND_BYTE_UART2(buffer_UART2_TX[ptUART2_TX_BUFFER]);
        ptUART2_TX_BUFFER++;                                                    // next char
    }
}

//****************************************************************************
// UART2 TX BYTE                                                              
//****************************************************************************

void vSEND_BYTE_UART2(char cBYTE)
{
    // Push data to UART 
    TXREG2 = cBYTE;

    while(!TRMT2)           // Wait here while busy sending 
    {
        continue;
    }
}



//****************************************************************************
// UART1 TX BYTE                                                              
//****************************************************************************

void vSEND_STR_UART2_2_RF(void)
{
    // pointer for TX buffer
    uint8_t     ptUART1_TX_BUFFER;
    volatile uint8_t     flag_DONE;

    // Initialize pointer to buffer...
    ptUART1_TX_BUFFER = 0;
    flag_DONE = 0;
    
    // Wait until RF spectrum is free and until the given time slot
    // A flag is set by the RX ISR after receiving RF data 
    // This flag is cleared after by TMR4 after 
    
    while(flag_DONE == 0)     // Check if RF spectrum is free
    {   
        if((REG_UART.bitv.RX2_InProgress == 0) && (BAUDCON2bits.RCIDL == 1))     // Check if RF spectrum is free 
        {
            // Send string from UART1 TX buffer via UART2 --> RF 
            while((buffer_UART1_TX[ptUART1_TX_BUFFER] != 0) || (ptUART1_TX_BUFFER <= 64)) 
            {
                vSEND_BYTE_UART2(buffer_UART1_TX[ptUART1_TX_BUFFER]);
                ptUART1_TX_BUFFER++;                                                // next char
            }
            flag_DONE = 1;
        }
    }
}


/******************************************************************************/
/* UART 1 TX STATE MACHINE                                                    */
/******************************************************************************/
/* Pre-defined states for UART TX state machines                              */
/* STATE_TX_INIT                0x01            Initialize TX state machine   */
/* STATE_TX_WAIT                0x02            Wait for start signal         */
/* STATE_TX_SEND                0x03            Send data                     */
/* STATE_TX_PENDING             0x04            TX pending                    */
/* STATE_TX_ERROR               0x05            TX Error                      */
/******************************************************************************/
/* UART 1 TX BUFFER[255] in RAM BANK 2                                        */
/******************************************************************************/

void sm_UART1_TX(void)
{

    static int8_t   state_UART1_TX = STATE_TX_INIT;

    NOP();

    int8_t     pntr_UART1_TX_BUFFER;

    switch(state_UART1_TX)
    {
        case STATE_TX_INIT :                                                    // Wait here for DATA ready flag
        {
            // Initialize ...
            pntr_UART1_TX_BUFFER = 0;
            state_UART1_TX = STATE_TX_WAIT;                                     // Set next state SEND DATA
            break;
        }
        case STATE_TX_WAIT :                                                    // Wait for start TX flag
        {
            if(REG_UART.bitv.TX1_DataSetReady == 1)                             // Check DataSetReady flag
            {
                state_UART1_TX = STATE_TX_SEND;                                 // Set next state
            }
            break;
        }
        case STATE_TX_SEND :                                                    // Send byte of data
        {
            NOP();
            TXREG1 = buffer_UART1_TX[pntr_UART1_TX_BUFFER];
            state_UART1_TX = STATE_TX_PENDING;                                  // Set next state
            break;
        }
        case STATE_TX_PENDING :                                                 // Wait for TX to complete
        {
            if(!TRMT1)                                                          // While busy sending exit 
                break;

            pntr_UART1_TX_BUFFER++;                                             // point to next char

            if (pntr_UART1_TX_BUFFER <= 64)                                     // If not overflow buffer
            {
                if (buffer_UART1_TX[pntr_UART1_TX_BUFFER] == 0) 
                {
                    state_UART1_TX = STATE_TX_DONE;                             // Done sending OK
                }
                else 
                {
                    state_UART1_TX = STATE_TX_SEND;                             // Next BYTE
                }
            }
            else 
            {
                state_UART1_TX = STATE_TX_ERROR;                                // BUFFER overrun ... ?
            }
            break;
        }
        case STATE_TX_DONE :                                                    // Done sending
        {
            REG_UART.bitv.TX1_DataSetReady = 0;                                 // Reset TX flag when done
            state_UART1_TX = STATE_TX_INIT;                                     // Set next state
            break;
        }
        case STATE_TX_ERROR :                                                   // Buffer not terminated
        {
            // TODO: Handle TX1 Error 
            REG_UART.bitv.TX1_DataSetReady = 0;                                 // Reset TX flag when done
            state_UART1_TX = STATE_TX_INIT;                                     // Set next state
            break;
        }
        default :                   // Default
        {
            REG_UART.bitv.TX1_DataSetReady = 0;                                 // Reset TX flag when done
            state_UART1_TX = STATE_TX_INIT;                                     // Should never come here
            break;
        }
    }
}

/******************************************************************************/
/* UART 2 TX STATE MACHINE                                                    */
/******************************************************************************/
/* Provides feedback over HM-TRP                                              */
/******************************************************************************/
/* Pre-defined states for UART TX state machines                              */
/* STATE_TX_INIT                0x01            Initialize TX state machine   */
/* STATE_TX_WAIT                0x02            Wait for start signal         */
/* STATE_TX_SEND                0x03            Send data                     */
/* STATE_TX_PENDING             0x04            TX pending                    */
/* STATE_TX_ERROR               0x05            TX Error                      */
/******************************************************************************/
/* UART2 TX BUFFER[255] in RAM BANK 3                                         */
/******************************************************************************/

void sm_UART2_TX(void)
{
    static int8_t   state_UART2_TX = STATE_TX_INIT;
    
    int8_t     pntr_UART2_TX_BUFFER;

    NOP();

    switch(state_UART2_TX)
    {
        case STATE_TX_INIT :                            // Wait here for DATA ready flag
        {
            // Initialize ...
            pntr_UART2_TX_BUFFER = 0;
            state_UART2_TX = STATE_TX_WAIT;             // Set next state SEND DATA
            break;
        }
        case STATE_TX_WAIT :                            // Wait for start TX flag
        {
            if(REG_UART.bitv.TX2_DataSetReady == 1)     // Check DataSetReady flag
            {
                state_UART2_TX = STATE_TX_SEND;         // Set next state
            }
            break;
        }
        case STATE_TX_SEND :                            // Send byte of data
        {
            NOP();
            LATAbits.LATA3 = 1;                         // Turn TX LED RED ON
            TXREG2 = buffer_UART2_TX[pntr_UART2_TX_BUFFER];
            state_UART2_TX = STATE_TX_PENDING;         // Set next state
            break;
        }
        case STATE_TX_PENDING :                         // Wait for TX to compleet
        {
            if(!TRMT2)                                  /* While busy sending exit */
            {
                break;
            }

            pntr_UART2_TX_BUFFER++;                      // point to next char

            if (pntr_UART2_TX_BUFFER <= 64)              // If not overflow buffer
            {
                if (buffer_UART2_TX[pntr_UART2_TX_BUFFER] == 0) 
                {
                    state_UART2_TX = STATE_TX_DONE;      // Done sending OK
                }
                else 
                {
                    state_UART2_TX = STATE_TX_SEND;      // Next BYTE
                }
            }
            else 
            {
                state_UART2_TX = STATE_TX_ERROR;          // BUFFER overrun ... ?
            }
            break;
        }
        case STATE_TX_DONE :                              // Done sending
        {
            REG_UART.bitv.TX2_DataSetReady = 0;           // Reset flag
            state_UART2_TX = STATE_TX_INIT;               // Set next state
            break;
        }
        case STATE_TX_ERROR :                             // Buffer not terminated
        {
            REG_UART.bitv.TX2_DataSetReady = 0;           // Reset flag
            state_UART2_TX = STATE_TX_INIT;               // Set next state
            break;
        }
        default :                                         // Default state
        {
            REG_UART.bitv.TX2_DataSetReady = 0;           // Reset flag
            state_UART2_TX = STATE_TX_INIT;               // Set next state
            break;
        }
    }
}




void sub_ERR_UART1(void)                      // UART error handler 
{
    char    char_DATA_RX1;

    NOP();

    // UART1 Frame Error Handling 
    if(RCSTA1bits.FERR == 1) 
    {              
        char_DATA_RX1 = RCREG1;             // Read and clear RX buffer 
    }
    
    // UART Overrun Error Handling 
    if(RCSTA1bits.OERR == 1) 
    {
        RCSTA1bits.CREN = 0;               // Reset Continues Receive bit 
        NOP();
        RCSTA1bits.CREN = 1;
        NOP();
    }

}

void sub_ERR_UART2(void)                      // UART error handler 
{
    char    char_DATA_RX2;
    NOP();

    // UART2 Frame Error Handling 
    if(RCSTA2bits.FERR == 1)
    {
        NOP();
        char_DATA_RX2 = RCREG2;             // Read and clear RX buffer 
    }

    // UART Overrun Error Handling 
    if(RCSTA2bits.OERR == 1)
    {
        NOP();    
        RCSTA2bits.CREN = 0;               // Reset Continues Receive bit 
        NOP();
        RCSTA2bits.CREN = 1;
        NOP();
    }
}

void init_VAR(void) 
{
//    // Milli seconds 
//    int_mSec_0 = 0;                // mSec x 1 counter
//    int_mSec_10 = 0;               // mSec x 10 counter
//    int_mSec_100 = 0;              // mSec x 100 counter
//
//    // TIME 
//    int_MATCH_OP_Seconds = 0;           // Decimal representation of ..
//    int_MATCH_OP_Minutes = 0;           // Decimal representation of ..
//    int_MATCH_OP_Hours = 0;             // Decimal representation of ..
//
//    // DATE 
//    //int8_Days = 0;                // Decimal representation of ..
//    //int_months = 0;               // Decimal representation of ..
//    //int8_Years = 0;               // Decimal representation of ..
//
//    tmr_CountDown_TIME = 0;         // Count Down timer for TIME on display
//    tmr_CountDown_TEMP = 0;         // Count Down timer for TEMP on display
//    tmr_CountDown_DISP = 0;         // Count Down timer for DISP UPDATE
//    tmr_CountDown_BEEP = 0;         // Count Down timer for DISP UPDATE
}



void    v_CREATE_S0_TELEGRAM(void)
{
    // ID:a:I:b:M1:c:d:M2:e:f:M3:g:h:M4:i:j:M5:k:l    

    /* Initialize TX buffer for UART 1 */
    buffer_UART1_TX[0]  = 'I';
    buffer_UART1_TX[1]  = 'D';
    buffer_UART1_TX[2]  = ':';
    buffer_UART1_TX[3]  = 'A';
    buffer_UART1_TX[4]  = 'B';
    buffer_UART1_TX[5]  = 'C';
    buffer_UART1_TX[6]  = ':';
    buffer_UART1_TX[7]  = 'I';
    buffer_UART1_TX[8]  = ':';
    buffer_UART1_TX[9]  = '1';
    buffer_UART1_TX[10] = '0';
    buffer_UART1_TX[11] = ':';
    buffer_UART1_TX[12] = 'M';
    buffer_UART1_TX[13] = '1';
    buffer_UART1_TX[14] = ':';
    buffer_UART1_TX[15] = '0';
    buffer_UART1_TX[16] = '1';  
    buffer_UART1_TX[17] = ':';
    buffer_UART1_TX[18] = '0';  
    buffer_UART1_TX[19] = '2';
    buffer_UART1_TX[20] = ':';  
    buffer_UART1_TX[21] = 'M';
    buffer_UART1_TX[22] = '2';  
    buffer_UART1_TX[23] = ':';
    buffer_UART1_TX[24] = '0';  
    buffer_UART1_TX[25] = '3';
    buffer_UART1_TX[26] = ':'; 
    buffer_UART1_TX[27] = '0';
    buffer_UART1_TX[28] = '4';
    buffer_UART1_TX[29] = ':';
    buffer_UART1_TX[30] = 'M';
    buffer_UART1_TX[31] = '3';
    buffer_UART1_TX[32] = ':';
    buffer_UART1_TX[33] = '0';
    buffer_UART1_TX[34] = '5';
    buffer_UART1_TX[35] = ':';
    buffer_UART1_TX[36] = '0';
    buffer_UART1_TX[37] = '6';
    buffer_UART1_TX[38] = ' ';
    buffer_UART1_TX[39] = ' ';            // MASTER_VERSION_LSD;
    buffer_UART1_TX[40] = '\n';
    buffer_UART1_TX[41] = 0;
    buffer_UART1_TX[42] = 0;
    
}




//****************************************************************************
// read configuration data from EEPROM                                        
//****************************************************************************

void sub_READ_EEPROM(void)
{
    volatile unsigned char value = 'T';
    unsigned char address = 0xE5;

    NOP();

    // Writing value 'T' to EEPROM address 0xE5        
    eeprom_write(address, value);     // Writing value 0x9 to EEPROM address 0xE5
    
    // Writing value 'T' to EEPROM address 0xE5        
    value = eeprom_read(address);    // Reading the value from address 0xE5

    NOP();
    
    value = EEPROM_READ(address);    // Reading the value from address 0xE5 using the macro
    
    NOP();
    
    // Read ID's from EEPROM 
    uc_ID_0 = eeprom_read(0x00);                         // ID 0
    uc_ID_1 = eeprom_read(0x01);                         // ID 1
    uc_ID_2 = eeprom_read(0x02);                         // ID 2
    uc_ID_3 = eeprom_read(0x03);                         // ID 3

}


//****************************************************************************
// Write configuration data to EEPROM                                         
//****************************************************************************

void sub_WRITE_EEPROM(void)
{
//        // Write ID bytes to Internal EEPROM 
//        Write_b_eep(0x00, '0');
//        Busy_eep();
//        Write_b_eep(0x01, '1');
//        Busy_eep();
//        Write_b_eep(0x02, '2');
//        Busy_eep();
//        Write_b_eep(0x03, '3');
//        Busy_eep();
//
//        // Write MATCH time to Internal EEP (ADDRESS 0x10)
//        // Gebaseerd op een oude flag vandaar gekke value
//        Write_b_eep(0x10, 45);                                                  // Half Time in minutes
//        Busy_eep();
//
//        // Write OP/AF tellen to Internal EEPROM (ADDRESS 0x20)
//        Write_b_eep(0x20, 0x40);                                                // b 0100 0000 as in old version
//        Busy_eep();
//
//        // Write DISP tellen to Internal EEPROM (ADDRESS 0x30 and 0x31)
//        Write_b_eep(0x30, 30);                                                  // Display GPS TIME for 0 Sec
//        //Write_b_eep(0x30, 00);                                                  // Display GPS TIME for 3 Sec
//        Busy_eep();
//        Write_b_eep(0x31, 30);                                                  // Display DS18 TEMP for 3 Sec
//        //Write_b_eep(0x31, 00);                                                  // Display DS18 TEMP for 0 Sec
//        Busy_eep();
//
//        // Write BEEP interval to Internal EEPROM (ADDRESS 0x40)
//        Write_b_eep(0x40, 15);                                                // b 0100 0000 as in old version
//        Busy_eep();
//
//        // Write GOAL interval to Internal EEPROM (ADDRESS 0x50)
//        Write_b_eep(0x50, 15);                                                // b 0100 0000 as in old version
//        Busy_eep();
//
}

