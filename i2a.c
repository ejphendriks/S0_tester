//*****************************************************************************/
// Files to Include                                                           */
//*****************************************************************************/


#include <xc.h>             /* XC8 General Include File */
#include <stdint.h>         /* For uint8_t definition */
#include "i2a.h"


//*****************************************************************************
// Declare variables                                                          
//*****************************************************************************


//*****************************************************************************
// User Functions                                                             
//*****************************************************************************


//*****************************************************************************
// Convert all signed 8 bit integer to ascii 100th decimals
//*****************************************************************************

char i2a100(int8_t intA)
{
    int8_t      resultaat;
    int8_t      honderdtallen = 0;

    // Check if intA >= 100 ... intA is max 127
    resultaat = intA - 100;
    if (resultaat >= 0)
    {
        intA = intA - 100;
        honderdtallen++;
    }

    // Check if intA >= 200     ...  this is onzin
    resultaat = intA - 100;
    if (resultaat >= 0)
    {
        intA = intA - 100;
        honderdtallen++;
    }

    return honderdtallen + 0x30;

}


/******************************************************************************/
/* Converteer unsigned 8 bit integer naar ascii honderdtallen                 */
/******************************************************************************/

char i2a010(int8_t intA)
{
    int8_t      resultaat;
    int8_t      honderdtallen = 0;
    int8_t      tientallen = 0;

    // Check if intA >= 200
    resultaat = intA - 100;
    if (resultaat >= 0)
    {
        intA = intA - 100;
        honderdtallen++;
    }

    // Check if intA >= 100
    resultaat = intA - 100;
    if (resultaat >= 0)
    {
        intA = intA - 100;
        honderdtallen++;
    }

    // Tel het aantal resterende tientallen
    resultaat = intA - 10;
    while(resultaat >= 0)
    {
        tientallen++;
        intA = intA - 10;
        resultaat = intA - 10;
    }

    NOP();

    return tientallen + 0x30;

}


/******************************************************************************/
/* Converteer unsigned 8 bit integer naar ascii eenheden                      */
/******************************************************************************/

char i2a001(int8_t intA)
{
    int8_t      resultaat;
    int8_t      honderdtallen = 0;
    int8_t      tientallen = 0;

    /* Check if intA >= 200 */
    resultaat = intA - 100;
    if (resultaat >= 0)
    {
        intA = intA - 100;
        honderdtallen++;
    }

    /* Check if intA >= 100 */
    resultaat = intA - 100;
    if (resultaat >= 0)
    {
        intA = intA - 100;
        honderdtallen++;
    }

    /* Tel het aantal resterende tientallen */
    resultaat = intA - 10;
    while(resultaat >= 0) 
    {
        tientallen++;
        intA = intA - 10;
        resultaat = intA - 10;
    }

    NOP();

    return intA + 0x30;             /* return aantal eenheden in ASCII formaat*/

}



/******************************************************************************/
/* Converteer 8 bit integer naar ascii representatie tot max 99               */
/******************************************************************************/

void i2ai2a_XX(int8_t intA, char *pntr_Dx, char *pntr_xD)
{
    int8_t      resultaat;
    int8_t      honderdtallen = 0;
    int8_t      tientallen = 0;

    // Check if intA >= 200
    resultaat = intA - 100;
    if (resultaat >= 0){
        intA = intA - 100;
        honderdtallen++;
    }

    // Check if intA >= 100
    resultaat = intA - 100;
    if (resultaat >= 0){
        intA = intA - 100;
        honderdtallen++;
    }

    // Tel het aantal resterende tientallen
    resultaat = intA - 10;
    while(resultaat >= 0) {
        tientallen++;
        intA = intA - 10;
        resultaat = intA - 10;
    }

    NOP();

    *pntr_Dx = tientallen + 0x30;
    *pntr_xD = intA + 0x30;

}


/******************************************************************************/
/* Converteer 8 bit integer naar hun respectievelijke ASCII digits            */
/******************************************************************************/

void i2aXXX(int8_t intA, char *pntr_Dxx, char *pntr_xDx, char *pntr_xxD)
{
    int8_t      resultaat;
    int8_t      honderdtallen = 0;
    int8_t      tientallen = 0;

    // Check if intA >= 200
    resultaat = intA - 100;
    if (resultaat >= 0){
        intA = intA - 100;
        honderdtallen++;
    }

    // Check if intA >= 100
    resultaat = intA - 100;
    if (resultaat >= 0){
        intA = intA - 100;
        honderdtallen++;
    }

    // Tel het aantal resterende tientallen
    resultaat = intA - 10;
    while(resultaat >= 0) {
        tientallen++;
        intA = intA - 10;
        resultaat = intA - 10;
    }

    NOP();

    *pntr_Dxx = honderdtallen + 0x30;
    *pntr_xDx = tientallen + 0x30;
    *pntr_xxD = intA + 0x30;

}


