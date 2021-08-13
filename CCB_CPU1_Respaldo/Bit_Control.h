/*
 * Bit_Control.h
 *
 *  Created on: 02-12-2020
 *      Author: Matias Bravo
 */

#ifndef BIT_CONTROL_H_
#define BIT_CONTROL_H_

#include <stdint.h>
#include <stdbool.h>


//*****************************************************************************
//
//! pWriteOn: Variable to be modified
//! posSetParameter: Position of the variable to be modified
//! modeToSet: Value to write
//
//! Syntax: Set_FLG_CMD(&pWriteOn, posSetParameter, modeToSet)
//
//*****************************************************************************
void Bit_Write(uint16_t *pWriteOn, uint16_t posSetParameter, bool modeToSet)
{
    *pWriteOn = (*pWriteOn & ~(1UL << posSetParameter)) | ((modeToSet > 0) << posSetParameter);
}

//*****************************************************************************
//
//! readTo: Variable to be evaluated
//! posRead: Position of the variable to be read
//!
//! Return: Bit state
//
//*****************************************************************************
bool Bit_Read(uint16_t readFrom, uint16_t posRead)
{
    bool currentState = (readFrom >> posRead) & 1;
    return currentState;
}

//*****************************************************************************
//
//! pToggleTo: Variable whose status is changed
//! posToggle: Bit that changes state
//
//! Syntax: Bit_Toggle(&pToggleTo, posToggle)
//
//*****************************************************************************
void Bit_Toggle(uint16_t *pToggleTo, uint16_t posToggle)
{
    *pToggleTo ^= (1 << posToggle);
}

#endif /* BIT_CONTROL_H_ */
