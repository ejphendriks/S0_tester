/******************************************************************************/
/*Files to Include                                                            */
/******************************************************************************/

#include <xc.h>             // XC8 General Include File 
#include <stdint.h>         // For uint8_t definition 
#include <stdbool.h>        // For true/false definition 

//#include "eep.h"            // EEPROM library 

#include "i2a.h"            // For int to ascii conversion 


/******************************************************************************/
/* Function declaration                                                       */
/******************************************************************************/

void    vSTATE_MACHINE_UART1_RX(void);         // RX1 state machine
void    vSTATE_MACHINE_UART2_RX(void);         // RX2 state machine
void    v_SEND_MASTER_SYNC_UART2(void);        // MASTER SYNC routine


/******************************************************************************/
/* External sub-routines                                                      */
/******************************************************************************/

extern    void vSEND_BYTE_UART2(char);

/******************************************************************************/
/* Macro declaration                                                          */
/******************************************************************************/

#define testbit(var, bit) ((var) & (1 <<(bit)));
#define setbit(var, bit) ((var) |= (1 << (bit)));
#define clrbit(var, bit) ((var) &= ~(1 << (bit)));

/******************************************************************************/
/* Define constants                                                          */
/******************************************************************************/

#define     STATE_RX1_START      1       // START $
#define     STATE_RX1_DATA       2       // Load data
#define     STATE_RX1_CS1        3       // Checksum HEX_H
#define     STATE_RX1_CS2        4       // Checksum HEX_L
#define     STATE_RX1_CR         5       // \r
#define     STATE_RX1_LF         6       // \n
#define     STATE_RX1_ERROR      7       // ERROR ?

#define     STATE_RX2_START      1       // START $
#define     STATE_RX2_DATA       2       // Load data
#define     STATE_RX2_SEC_H      3
#define     STATE_RX2_SEC_L      4
#define     STATE_RX2_TMR1_H_H   5
#define     STATE_RX2_TMR1_H_L   6
#define     STATE_RX2_TMR1_L_H   7
#define     STATE_RX2_TMR1_L_L   8
#define     STATE_RX2_SEPARATOR  9      // * separator
#define     STATE_RX2_CS1       10      // Checksum HEX_H
#define     STATE_RX2_CS2       11      // Checksum HEX_L
#define     STATE_RX2_CR        12      // \r
#define     STATE_RX2_LF        13      // \n
#define     STATE_RX2_ERROR     14      // ERROR ?

//  Set count down timer for AUTO SYNC (in Seconds)
#define AUTO_SYNC_COUNT_DOWN    60          // AUTO SYNC every 60 seconds

//  Set count down timer for RF RELEASE (in mSec))
#define RF_TIME_SLOT            20          // Release RF for TX after x mSec


/******************************************************************************/
/* External union struct declaration                                          */
/******************************************************************************/

extern      union REG_CONTROL 
            {
                unsigned char byte;
                struct 
                {
                    unsigned SEC:1;             // 1 SEC has expired from 32kHz xtal
                    unsigned START_RF:1;        // START RF signal
                    unsigned LAP_RF:1;          // LAP RF signal
                    unsigned FINISH_RF:1;       // FINISH RF signal
                    unsigned TRIGGER:1;         // SEND TRIGGER via UART1 --> RF 
                    unsigned LOG2USB:1;         // LOG TIMESTAMP 2 USB
                    unsigned AUTO_SYNC:1;       // ENABLE AUTO SYNC
                    unsigned SYNC:1;            // SET SYNC flag
                } bitv;
            } REG_CONTROL;

            
extern      union REG_UART 
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
            
            
    // SNAPSHOT of TMR1 (2 bytes)
extern      union
            {
                 uint8_t uc_bytes[2];
                 unsigned int ui_word;
            } union_SNAPSHOT_TMR1;
    
    // SNAPSHOT of MASTER SECONDS (2 bytes)
extern      union
            {
                 uint8_t uc_bytes[2];
                 unsigned int ui_word;
            } union_SNAPSHOT_SECONDS;
     

/******************************************************************************/
/* External function declaration                                              */
/******************************************************************************/
            
            
char    cBin2Hex_H(char);
char    cBin2Hex_L(char);
int8_t  iHex2Bin(char, char);

void    vSEND_STR_UART1(void);                  // Send string to UART 1

/******************************************************************************/
/* External variable declaration                                              */
/******************************************************************************/

extern      char        cdtmr_S0_ON;              // Count Down timer for S0 pulse
extern      char        cdtmr_S0_OFF;             // Count Down timer for S0 pulse
extern      char        cdtmr_UART1_OFF;          // Count Down timer UART1 off (TMR0)

/* RX buffers */
extern      char        buffer_UART1_RX[255];           // Declared global in main.c
extern      char        buffer_UART2_RX[255];           // Declared global in main.c

/* TX buffer */
extern      char        buffer_UART1_TX[255];           // 
extern      char        buffer_UART2_TX[255];           // 

// Count Down timers 
extern      uint8_t     tmr_CountDown_IR_VALIDE;
extern      uint8_t     tmr_CountDown_IR_BROKEN;


//extern      uint8_t     tmr_CountDown_TIME;
//extern      uint8_t     tmr_CountDown_TEMP;
//extern      uint8_t     tmr_CountDown_DISP;
//extern      uint8_t     tmr_CountDown_FLASH;
//extern      uint8_t     tmr_CountDown_BEEP;
//extern      uint8_t     tmr_CountDown_RF;

//uint8_t     tmr_CountDown_RF_150;                        // Local !!!

//extern      unsigned char   uc_ID_0;
//extern      unsigned char   uc_ID_1;
//extern      unsigned char   uc_ID_2;
//extern      unsigned char   uc_ID_3;

// Variables for TIMER 1 - 32kHz x-tal
extern      int8_t     int_TMR1_MASTER_SEC_MSB;
extern      int8_t     int_TMR1_MASTER_SEC_LSB;

// Variables for TIMER 1 - 32kHz x-tal
extern      int8_t     int_TMR1_Seconds;
extern      int8_t     int_TMR1_Minutes;
extern      int8_t     int_TMR1_Hours;

// TMR4 Millisecond timer
extern      int8_t     int_TMR4_mSec0;
extern      int8_t     int_TMR4_mSec10;
extern      int8_t     int_TMR4_mSec100;
extern      int8_t     int_TMR4_Seconds;
extern      int8_t     int_TMR4_Minutes;
extern      int8_t     int_TMR4_Hours;

// Snapshot taken when 32kHz x-tal is at exactly 1 sec
extern      int8_t     int_SNAPSHOT_TMR4_mSec0;                                // mSec x 1 counter
extern      int8_t     int_SNAPSHOT_TMR4_mSec10;                               // mSec x 10 counter
extern      int8_t     int_SNAPSHOT_TMR4_mSec100;                              // mSec x 100 counter
extern      int8_t     int_SNAPSHOT_TMR4_Seconds;
extern      int8_t     int_SNAPSHOT_TMR4_Minutes;
extern      int8_t     int_SNAPSHOT_TMR4_Hours;

// Flag for extern bool_TMR1_CLOCK_ON
extern      int8_t     bool_TMR1_MATCH_ON;

// Count down timer for AUTO SYNC
extern      int8_t      uc_AUTO_SYNC_COUNTER;

/******************************************************************************/
/* Local variable declaration                                              */
/******************************************************************************/

// Time out period before TX2 is allowed
uint8_t     cntr_RX2_RELEASE;

// Local Count Down timers 
uint8_t     tmr_CountDown_LOG2USB;  


// DELTA INTERVAL 
extern    uint8_t     ui8_DELTA_INTERVAL;  
extern    int8_t      i8_DELTA_VALUE;  



/******************************************************************************/
/* Interrupt Routines                                                         */
/******************************************************************************/

/* High-priority service */

void interrupt high_isr(void)
{
    static uint8_t      local_pntr_UART1_BUFFER;                                // Pointer to UART1 BUFFER (GPS)

    // Disable new interrupts
    // Is taken care of by XC8
    // INTCONbits.GIEH should never be used inside ISR !!

    NOP();

    // -------------------------------------------------------------------------
    // ---  INT 0 - RB0  =   TACT SWITCH (GoTo SLEEP / WAKE-UP)
    // -------------------------------------------------------------------------
    if(INT0IF)      // INT0 Interrupt triggered via pin RB0
    {                                                                           
        INT0IF = 0;      // Clear flag 
        
        if(cdtmr_UART1_OFF == 0)
        {
            LATAbits.LATA1 = 1;
            LATAbits.LATA2 = 1;
            vSEND_STR_UART1();
            cdtmr_UART1_OFF = 10;
            LATAbits.LATA2 = 0;
        }
        
        
    }

    // -------------------------------------------------------------------------
    // ---  INT 1 - RB1     -  EXTRENAL START TRIGGER (start veer) 
    // -------------------------------------------------------------------------
    else if (INT1IF)    // RB1 is pressed
    {
        INT1IF = 0;     // Clear flag
    }

    // -------------------------------------------------------------------------
    // ---  INT 2 - RB2 
    // -------------------------------------------------------------------------
    else if (INT2IF)    // RB2 is pressed
    {
        INT2IF = 0;     // Clear flag

        if(cdtmr_S0_OFF == 0)
        {
            LATAbits.LATA3 = 1;
            LATAbits.LATA0 = 1;
            cdtmr_S0_ON = 30;                   // Set timer to 30 mSec
            cdtmr_S0_OFF = 250;                 // Set timer to 250 mSec
            
        }
    }
    
    /* Determine which flag generated the interrupt */
    if(INT0IF) 
    {                                                                           // IO Interrupt on RB0
        NOP();
        INT0IF = 0;                                                             // Clear Interrupt Flag */
        
        if(REG_CONTROL.bitv.TRIGGER != 1)               // Check if not already busy sending snapshot
        {
            // Copy both MASTER SECONDS 8 bit registers
            union_SNAPSHOT_SECONDS.uc_bytes[1] = int_TMR1_MASTER_SEC_MSB;
            union_SNAPSHOT_SECONDS.uc_bytes[0] = int_TMR1_MASTER_SEC_LSB;

            // Copy both TMR1 8 bit registers
            union_SNAPSHOT_TMR1.uc_bytes[1] = TMR1H;
            union_SNAPSHOT_TMR1.uc_bytes[0] = TMR1L;

            // Check for TMR1 carry over
            if(union_SNAPSHOT_TMR1.uc_bytes[1] != TMR1H)
            {
                union_SNAPSHOT_TMR1.uc_bytes[1] = TMR1H;
            }

            // Check for SECONDS carry over
            if(union_SNAPSHOT_SECONDS.uc_bytes[1] != int_TMR1_MASTER_SEC_MSB)
            {
                union_SNAPSHOT_SECONDS.uc_bytes[1] = int_TMR1_MASTER_SEC_MSB;
            }

            // Start sending trigger via routine in Main loop
            REG_CONTROL.bitv.TRIGGER = 1;
        }
    }
    else if (TMR0IF) 
    {                                                                           // TMR0 is 0,1 second time-out
        TMR0IF = 0;                                                             // Clear interrupt flag
        
        // Pre-load the interval of 0.1 Sec (65536-59286=6250)
        // Instr/sec: Fosc 64MHz ==> Fosc x 1/4 = 16MHz and pre-scaler 1:256
        // TMR0 = 59286;                                                        // 59286 gives a delay of 0.1 sec @ 64MHz
        TMR0H = 231;
        TMR0L = 150;
        
        
        // Count down timer for next UART data TX
        if (cdtmr_UART1_OFF != 0)
        {
            cdtmr_UART1_OFF--;                                          // Count Down 
        }
        else    
        {
            LATAbits.LATA1 = 0;
        }
                
                
        // Count down timer for IR BEAM VALIDATION 
        if (tmr_CountDown_IR_VALIDE != 0)
        {
            tmr_CountDown_IR_VALIDE--;                                          // Count Down 
        }

        // Count down timer for IR BEAM VALIDATION 
        if (tmr_CountDown_IR_BROKEN != 0)
        {
            tmr_CountDown_IR_BROKEN--;                                          // Count Down 
        }
    }
    else if (TMR1IF)    // TMR1 = 32kHz X-tal
    {                                                                           // TMR1 is One second pulse from external 32786Hz Xtal
        TMR1IF = 0;                                                             // Clear Interrupt Flag */

    }
    
    else if (TMR2IF)  // Internal countdown timer for IR BEAM broken
    {                                                                           // TMR2 is the internal count down timer
        TMR2IF = 0;                                                             // Clear Interrupt Flag
        
        NOP();      
        
        // Count down ON duration
        if (cdtmr_S0_ON == 0)
        {
            LATAbits.LATA3 = 0;             //  Turn off led
            LATAbits.LATA0 = 0;             //  Turn off NPN 
        }
        else
        {
            cdtmr_S0_ON = cdtmr_S0_ON - 1;
        }
        
        // Count down OFF timer - stop if 0
        if (cdtmr_S0_OFF != 0)
        {
            cdtmr_S0_OFF = cdtmr_S0_OFF - 1;
        }
        
    }
    
    else if (TMR3IF) 
    {                                                                           // TMR3 
        TMR3IF = 0;                                                             // Clear Interrupt Flag 
        // Turn LED BLUE RA4 OFF (SINK))
        // LATAbits.LATA4 = 1;                                                  // Turn LED BLUE RA4 OFF (inverted)
        // Turn TMR5 off
        TMR3ON = 0;                                                             // Disable TMR3
    }
    else if (TMR4IF)            // Clear RF spectrum flag UNTIL TIME-SLOT
    {
        // Clear flag
        TMR4IF = 0;                                             // Clear Interrupt Flag 
        // Count Down Timers
        if(cntr_RX2_RELEASE != 0)
        {
            cntr_RX2_RELEASE--;                                 // Count down 10 mSec for MASTER and 20 mSec for SLAVE
            if(cntr_RX2_RELEASE == 0)
            {
                REG_UART.bitv.RX2_InProgress = 0;               // Clear RF in Progress flag
            }
        }
        
        // Increment milli-second counter 
        int_TMR4_mSec0++;
        if (int_TMR4_mSec0 == 10) 
        {
            int_TMR4_mSec0 = 0;
            int_TMR4_mSec10++;
            if (int_TMR4_mSec10 == 10) 
            {
                int_TMR4_mSec10 = 0;
                int_TMR4_mSec100++;
                if (int_TMR4_mSec100 == 10) 
                {
                    int_TMR4_mSec100 = 0;                                           // Roll-over after 1 second 
                    // Turn Seconds Indicator LED BLUE RA4 ON (SINK))
                    // LATAbits.LATA4 = 0;                                         // Turn LED RA4 ON (inverted)
                    // Pre-load TMR3 for 0,032 Sec delay to turn LED_LATA4 OFF
                    // TMR3 = 0;                                                // TMR 3 is a 16 bit register
                    TMR3H = 0;             // Preset for TMR3 MSB register
                    TMR3L = 0;             // Preset for TMR3 LSB register
                    // Turn TMR3 ON
                    TMR3ON = 1;                                                 // Enable TMR5

                    int_TMR4_Seconds++;
                    if(int_TMR4_Seconds == 60)
                    {
                        int_TMR4_Seconds = 0;
                        int_TMR4_Minutes++;
                        if(int_TMR4_Minutes == 60)
                        {
                            int_TMR4_Minutes = 0;
                            int_TMR4_Hours++;
                            if(int_TMR4_Hours == 24)
                            {
                                int_TMR4_Hours = 0;
                            }
                        }
                    }
                }
            }
        }
    }
    else if (TMR5IF) 
    {                                                                           // Turn Sec LED on RA2 green OFF */
        TMR5IF = 0;                                                             // Clear Interrupt Flag */

        NOP();
        
        // Turn Seconds Indicator LED BLUE RA4 OFF 
        LATAbits.LATA4 = 0;                                                     // Turn LED RA4 OFF 

        // Turn TMR5 off
        TMR5ON = 0;                                                             // Disable TMR5*/
    }
    else if (RC1IF) 
    {                                                                           // Data from GPS module or from MCU */
        RC1IF = 0;                                                              // Clear Interrupt flag UART 1 */
        vSTATE_MACHINE_UART1_RX();
        NOP();
    }
    else if (RC2IF) 
    {                                                                           // Data from HM-TRP */
        RC2IF = 0;                                                              // Clear Interrupt flag UART 2
        vSTATE_MACHINE_UART2_RX();
        NOP();
    }
    else 
    {
        NOP();                                                                  // Unhandled interrupts */
    }
      
    // Enable new interrupts is taken care of by MCU internals
    // NEVER CHANGE INTCONbits.GIEH = 1 in the ISR !!!
}


void low_priority interrupt low_isr(void)
{

      /* This code stub shows general interrupt handling.  Note that these
      conditional statements are not handled within 3 separate if blocks.
      Do not use a separate if block for each interrupt flag to avoid run
      time errors. */

#if 0

      /* TODO Add Low Priority interrupt routine code here. */

      /* Determine which flag generated the interrupt */
      if(INT0IF)
      {
          <Interrupt Flag 1=0>; /* Clear Interrupt Flag 1 */
      }
      else if (<Interrupt Flag 2>)
      {
          <Interrupt Flag 2=0>; /* Clear Interrupt Flag 2 */
      }
      else
      {
          /* Unhandled interrupts */
      }

#endif

}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// --- SEND MASTER SYNC SIGNAL over RF
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------


void v_SEND_MASTER_SYNC_UART2(void)         // 
{

    // Sample data strings
    // $ORG1RT#AABBCCDD*CS
    // $ORG1RT#165C0230*CS
    
    //  0 '$'
    //  1 'O'     = 0RG ID 
    //  2 'O'     = 0RG ID 
    //  3 'O'     = 0RG ID 
    //  4 'O'     = 0RG ID 
    //  5 'R'     = RETRIES       
    //  6 'T'     = COMMAND         (T = SYNC, S = START and A = ABORT)
    //  7 '#'     = Separator #
    //  8 '0'     = SEC1 HEX_H 
    //  9 '6'     = SEC1 HEX_L 
    //  8 '0'     = SEC0 HEX_H 
    //  9 '6'     = SEC0 HEX_L 
    // 11 'A'     = TMR1_H HEX_H 
    // 12 'B'     = TMR1_H HEX_L 
    // 13 'C'     = TMR1_L HEX_H 
    // 14 'D'     = TMR1_L HEX_L 
    // 15 '*'     = Separator *
    // 16 'C'     = CHECKSUM HEX_H 
    // 17 'S'     = CHECKSUM HEX_L 
    // 18 '\r'    = Carriage Return
    // 19 '\n'    = New Line

    union
    {
         uint8_t uc_bytes[2];               // array of 2 bytes that make up TRM1H:TMR1L
         unsigned int ui_word;              // integer that allows direct integer manipulation
    } union_TMR1;

    
    uint8_t             uc_Seconds;         // Used to make snapshot of the expired time
    unsigned char       uc_Seconds_H;       // Stores the Lo byte in ASCII representation
    unsigned char       uc_Seconds_L;       // Stores the Lo byte in ASCII representation

    vSEND_BYTE_UART2('$');
    vSEND_BYTE_UART2('O');
    vSEND_BYTE_UART2('R');
    vSEND_BYTE_UART2('G');
    vSEND_BYTE_UART2('1');
    vSEND_BYTE_UART2('R');
    vSEND_BYTE_UART2('T');
    vSEND_BYTE_UART2('#');
    
    // SEND TIMESTAMP MARKER 
    
    vSEND_BYTE_UART2(cBin2Hex_H(int_TMR1_MASTER_SEC_MSB));          // Hex Sec H
    vSEND_BYTE_UART2(cBin2Hex_L(int_TMR1_MASTER_SEC_MSB));          // Hex Sec L
    vSEND_BYTE_UART2(cBin2Hex_H(int_TMR1_MASTER_SEC_LSB));          // Hex Sec H
    vSEND_BYTE_UART2(cBin2Hex_L(int_TMR1_MASTER_SEC_LSB));          // Hex Sec L

    vSEND_BYTE_UART2('0');                                          // Hex TMR1H_H
    vSEND_BYTE_UART2('2');                                          // Hex TMR1H_L
    vSEND_BYTE_UART2('3');                                          // Hex TMR1L_H
    vSEND_BYTE_UART2('0');                                          // Hex TMR1L_L
    
    vSEND_BYTE_UART2('*');
    
    vSEND_BYTE_UART2('C');
    vSEND_BYTE_UART2('S');
    
    vSEND_BYTE_UART2('\r');
    vSEND_BYTE_UART2('\n');
    
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// --- UART1 is connected to USB --> PC
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

void vSTATE_MACHINE_UART1_RX(void) 
{

    // declare variables
    char                char_DATA_RX1;              // Holds incoming data for UART1
    static uint8_t      local_pntr_UART1_BUFFER;    // Pointer to UART1 BUFFER (RF / RC)
    static uint8_t      state_UART1_RX;             // Hold state for RX1 state machine

    // Copy data
    char_DATA_RX1 = RCREG1;                         // Save RX1 data

    if(char_DATA_RX1 == '$')                       // Upon $ always goto to START
    {
        state_UART1_RX = STATE_RX1_START;
    }

    switch(state_UART1_RX) 
    {
        case STATE_RX1_START :                      // $
        {
            if(char_DATA_RX1 == '$') 
            {
                // New incoming string 
                REG_UART.bitv.RX1_InProgress = 1;
                // Auto reset State Machine after 10 mSec 
                // cntr_RESET_UART1_RX = 10;
                // TOGGLE LED ORANGE 
                LATAbits.LATA1 =! LATAbits.LATA1;
                // Start at beginning of buffer 
                local_pntr_UART1_BUFFER = 0;
                // Store RX data in UART1 buffer 
                buffer_UART1_RX[local_pntr_UART1_BUFFER++] = char_DATA_RX1;
                // Next state 
                state_UART1_RX = STATE_RX1_DATA;
            }
            break;
        }
        
        case STATE_RX1_DATA :
        {
            // Store RX data in UART1 buffer 
            buffer_UART1_RX[local_pntr_UART1_BUFFER++] = char_DATA_RX1;
            
            // Check if *
            if(char_DATA_RX1 == '*')                                // TODO: And LENGTH is correct
            {
                state_UART1_RX = STATE_RX1_CS1;                     // Next state 
            }
            
            if(local_pntr_UART1_BUFFER <= 24)                       // If too long ...
            {
                REG_UART.bitv.RX1_InProgress = 0;                   // Reset flag
                state_UART1_RX = STATE_RX1_START;                   // Reset to start state 
            }
            break;
        }
        
        case STATE_RX1_CS1 :
        {
            NOP();
            
            // Store RX data in UART1 buffer 
            buffer_UART1_RX[local_pntr_UART1_BUFFER++] = char_DATA_RX1;

            state_UART1_RX = STATE_RX1_START;                   // Reset to START
            break;
        }
        
        case STATE_RX1_CS2 :
        {
            // Store RX data in UART1 buffer 
            buffer_UART1_RX[local_pntr_UART1_BUFFER++] = char_DATA_RX1;
            
            // TODO: Check CHECKSUM
            // ???

            NOP();    

            state_UART1_RX = STATE_RX1_LF;
            
            break;
        }
        
        case STATE_RX1_LF :
        {
            buffer_UART1_RX[local_pntr_UART1_BUFFER++] = char_DATA_RX1;
            state_UART1_RX = STATE_RX1_CR;
            break;
        }
        
        case STATE_RX1_CR :
        {
            buffer_UART1_RX[local_pntr_UART1_BUFFER++] = char_DATA_RX1;
            // Clear RX in progress
            REG_UART.bitv.RX1_InProgress = 0;
            // Next state    
            state_UART1_RX = STATE_RX1_START;
            break;
        }
        
        case STATE_RX1_ERROR :           // Not used because it requires an RX t0 get out of
        {
            state_UART1_RX = STATE_RX1_START;
            break;
        }
        
        default :                                           // Always recover from undefined state
        {
            // Do nothing. Don't change states here. 
            // Next $ it will jump to start again.
            NOP();
            
            break;
        }
    }
}



// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// --- UART2 is connected to RF module HM-TRP
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

// EXPECTING something like this : $ORG1RT#00490230*CS
//  0 '$'
//  1 'O'     = 0RG ID 
//  2 'O'     = 0RG ID 
//  3 'O'     = 0RG ID 
//  4 'O'     = 0RG ID 
//  5 'R'     = RETRIES       
//  6 'T'     = COMMAND         (T = SYNC, S = START and A = ABORT)
//  7 '#'     = Separator #
//  8 '0'     = SEC1 HEX_H 
//  9 '6'     = SEC1 HEX_L 
//  8 '0'     = SEC0 HEX_H 
//  9 '6'     = SEC0 HEX_L 
// 11 'A'     = TMR1_H HEX_H 
// 12 'B'     = TMR1_H HEX_L 
// 13 'C'     = TMR1_L HEX_H 
// 14 'D'     = TMR1_L HEX_L 
// 15 '*'     = Separator *
// 16 'C'     = CHECKSUM HEX_H 
// 17 'S'     = CHECKSUM HEX_L 			// ACTUAL SYNC MOMENT at 18 char @ 9600 BAUD = ~ 18mSec
// 18 '\r'    = Carriage Return
// 19 '\n'    = New Line

void vSTATE_MACHINE_UART2_RX(void) 
{
    // declare variables
    char                char_DATA_RX2;              // Holds incoming data for UART2
    static uint8_t      local_pntr_UART2_BUFFER;    // Pointer to UART2 BUFFER (RF / RC)
    static uint8_t      state_UART2_RX;             // Hold state for RX2 state machine

    // Copy data
    char_DATA_RX2 = RCREG2;                         // Save RX2 data

    // Set flag and count down timer
    REG_UART.bitv.RX2_InProgress = 1;               // Indication that an RF transmissions is is progress
    cntr_RX2_RELEASE = RF_TIME_SLOT;                // Release InProgress flag after TIMESLOT has expired
            
    if(char_DATA_RX2 == '$')                        // Upon $ always goto to START
    {
        state_UART2_RX = STATE_RX2_START;
    }

    switch(state_UART2_RX) 
    {
        case STATE_RX2_START :                      // $
        {
            if(char_DATA_RX2 == '$') 
            {
                // Start at beginning of buffer 
                local_pntr_UART2_BUFFER = 0;
                /* Store RX data in UART2 buffer */
                buffer_UART2_RX[local_pntr_UART2_BUFFER++] = char_DATA_RX2;
                /* Next state */
                state_UART2_RX = STATE_RX2_DATA;
            }
            break;
        }
        case STATE_RX2_DATA :
        {
            // Store RX data in UART2 buffer 
            buffer_UART2_RX[local_pntr_UART2_BUFFER++] = char_DATA_RX2;
            
            // Check if *
            if(char_DATA_RX2 == '*')                                // TODO: And LENGTH is correct
            {
                state_UART2_RX = STATE_RX2_CS1;                     // Next state 
            }

            if(local_pntr_UART2_BUFFER >= 24)                       // If too long ...
            {
                // REG_UART.bitv.RX2_InProgress = 0;                // Reset flag is done by TMR4
                state_UART2_RX = STATE_RX2_START;                   // Reset to start state 
            }
            break;

        }
        
        case STATE_RX2_CS1 :
        {
            NOP();              // $ORG1RT#12760230*CS
            
            // Store RX data in UART2 buffer 
            buffer_UART2_RX[local_pntr_UART2_BUFFER++] = char_DATA_RX2;

            if(buffer_UART2_RX[1] != 'O')                   // ID 1 correct ?
            {
                state_UART2_RX = STATE_RX2_START;
                break;
            }
            if(buffer_UART2_RX[2] != 'R')                   // ID 1 correct ?
            {
                state_UART2_RX = STATE_RX2_START;
                break;
            }
            if(buffer_UART2_RX[3] != 'G')                   // ID 1 correct ?
            {
                state_UART2_RX = STATE_RX2_START;
                break;
            }
            if(buffer_UART2_RX[4] != '1')                   // ID 1 correct ?
            {
                state_UART2_RX = STATE_RX2_START;
                break;
            }

            state_UART2_RX = STATE_RX2_CS2;
            break;
        }
        
        case STATE_RX2_CS2 :
        {
            // Store RX data in UART2 buffer             
            buffer_UART2_RX[local_pntr_UART2_BUFFER++] = char_DATA_RX2;
            
            // TODO: Check CHECKSUM
            
            // Toggle LED
            LATAbits.LATA2 =! LATAbits.LATA2;               // Toggle LEFT LED
                    
            // $ORG1RT#12760230*CS
            
            TMR1ON = 0;

                // Set MASTER SECONDS 16 bit register 
                int_TMR1_MASTER_SEC_MSB = iHex2Bin(buffer_UART2_RX[8], buffer_UART2_RX[9]);
                int_TMR1_MASTER_SEC_LSB = iHex2Bin(buffer_UART2_RX[10], buffer_UART2_RX[11]);

                // Set actual TMR1 16 bit register and set carry over flag for 2^15 = 1 sec...
                //TMR1H = iHex2Bin(buffer_UART2_RX[12], buffer_UART2_RX[13]) | 0b10000000; 
                //TMR1L = iHex2Bin(buffer_UART2_RX[14], buffer_UART2_RX[15]);

                TMR1 = 35233;           // (2^15 +) 2465 ~ 75.2 mSec... !
                
            TMR1ON = 1;
            
            // Indicate START event received
            REG_UART.bitv.RX2_NewData = 1;

            state_UART2_RX = STATE_RX2_LF;
            break;
        }
        case STATE_RX2_LF :
        {
            buffer_UART2_RX[local_pntr_UART2_BUFFER++] = char_DATA_RX2;
            state_UART2_RX = STATE_RX2_CR;
            break;
        }
        case STATE_RX2_CR :
        {
            buffer_UART2_RX[local_pntr_UART2_BUFFER++] = char_DATA_RX2;
            // RX in progress
            // REG_UART.bitv.RX2_InProgress = 0;        Cleared by TMR4
            // Next state    
            state_UART2_RX = STATE_RX2_START;
            break;
        }
        case STATE_RX2_ERROR :
        {
            state_UART2_RX = STATE_RX2_START;
            break;
        }
        default :                                           // Always recover from undefined state
        {
            state_UART2_RX = STATE_RX2_START;
            break;
        }
    }
}
