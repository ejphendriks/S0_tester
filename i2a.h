/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

/* TODO Application specific user parameters used in user.c may go here */

/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/

char    i2a100(int8_t);                /* Convert integer to ascii --> Hundreds */
char    i2a010(int8_t);                /* Convert integer to ascii --> Tens */
char    i2a001(int8_t);                /* Convert integer to ascii --> Ones */

void    i2a_XX(int8_t, char *, char *);
void    i2aXXX(int8_t, char *, char *, char* );