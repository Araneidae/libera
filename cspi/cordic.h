// $Id: cordic.h,v 1.2 2005/10/28 10:06:29 miha Exp $

//! \file cordic.h
//! Public CORDIC declarations.

#if !defined(_CORDIC_H)
#define _CORDIC_H

/** Private.
 *  Calculates the amplitude from I and Q (sin and cos) value.
 *  Returns amplitude.
 *
 *  @param I I (sin) component of the amplitude.
 *  @param Q Q (cos) component of the amplitude.
 */
int cordic_amp( int I, int Q );

#endif	// _CORDIC_H
