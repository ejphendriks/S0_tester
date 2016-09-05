/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

/* TODO Application specific user parameters used in user.c may go here */

/* Used for absolute delay _delay_ms() */
#define _XTAL_FREQ      64000000

// VERSION NUMBER 
#define MASTER_VERSION_MSD       '0'        // MSD
#define MASTER_VERSION_LSD       '1'        // LSD


/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/

/* TODO User level functions prototypes (i.e. InitApp) go here */
/* MCU IO and Peripheral Initialization */

void    init_PORT_IO(void);
void    init_OSC(void);
void    init_ADC(void);
void    init_TMR0(void);
void    init_TMR135(void);
void    init_TMR246(void);
void    init_UART1(void);
void    init_UART2(void);
void    init_INT(void);
void    init_HELLO(void);

void    TX_UART1(char);                     /* Send data to UART 1 */
void    TX_UART2(char);                     /* Send data to UART 2 */

void    init_UART1_BAUDRATE_9600(void);     /* Initialize baudrate */
void    init_UART1_BAUDRATE_19200(void);    /* Initialize baudrate */

void    Delay1S(void);                      /* Delay 1 Sec blocking */

void    init_BUFFERS(void);


