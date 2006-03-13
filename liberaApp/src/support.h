/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2005-2006  Michael Abbott, Diamond Light Source Ltd.
 *
 * The Libera EPICS Driver is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * The Libera EPICS Driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Contact:
 *      Dr. Michael Abbott,
 *      Diamond Light Source Ltd,
 *      Diamond House,
 *      Chilton,
 *      Didcot,
 *      Oxfordshire,
 *      OX11 0DE
 *      michael.abbott@diamond.ac.uk
 */

/* High efficiency support routines:
 *
 *      CLZ             Counts leading zeros.
 *      MulUU           Scaled multiplication routines
 *      MulSS                   "
 *      MulUS                   "
 *      Reciprocal      Computes 1/x
 *      nmTOmm          Multiplies by 1e-6 and converts to float.
 */


/* Returns the number of leading zeros in an integer. */
inline unsigned int CLZ(unsigned int x) 
{
    unsigned int result; 
    __asm__("clz     %0, %1" : "=r"(result) : "r"(x)); 
    return result; 
}


/* Returns 2^-32 * x * y.  This is particularly convenient for fixed point
 * arithmetic, and is reasonably inexpensive (approximately 30ns). */
inline unsigned int MulUU(
    unsigned int x, unsigned int y)
{
    return ((unsigned long long) x * y) >> 32;
}


inline int MulSS(int x, int y)
{
    return ((long long) x * y) >> 32;
}


/* To retain the maximum possible number of bits we have to take a bit of
 * care when multiplying a signed by an unsigned integer.  This routine works
 * by writing the signed part as
 *      y = y0 - s*2^31
 * where s is the sign bit and y0 the rest of y.  We can then use unsigned
 * multiplication to compute
 *      x*y*2^-32 = x*y0*2^-32 - s*x*2^-1.
 *      
 * If it is known that x < 2^31 (and so cannot be mistaken for a signed
 * value) then it will be faster to use MulSS instead. */
inline int MulUS(unsigned int x, int y)
{
    unsigned int y0 = y & 0x7FFFFFFF;
    int result = (int) MulUU(x, y0);
    if (y < 0)
        result -= x >> 1;
    return result;
}


/* Returns 2^61 / X after normalising X by shift amount shift (so, strictly
 * speaking, returns 2^(61-shift)/X together with shift).  The value returned
 * is in the range 2^29 to 2^30. */
unsigned int Reciprocal(unsigned int X, int &shift);


#if 0
/* Fast conversion of integer value in nm to floating point value in mm.  Not
 * actually so useful at the moment... */
void nmTOmm(int nm, float &mm);
#endif
/* Not so fast return of a double as actually expected by EPICS. */
inline double nmTOmm(int nm) { return 1e-6 * nm; }


/* A rather randomly placed helper routine.  This and its equivalents are
 * defined all over the place, but there doesn't appear to be a definitive
 * definition anywhere. */
#define ARRAY_SIZE(a)   (sizeof(a)/sizeof((a)[0]))
