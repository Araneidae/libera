/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2005-2009  Michael Abbott, Diamond Light Source Ltd.
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

/* High efficiency numeric support routines. */

#include <limits.h>

#include "test_error.h"
#include "numeric.h"

#include "numeric-lookup.h"


/* Computes (2^s/D, s) with 30 or 31 bits of precision (the bottom couple of
 * bits are a little tricky to get right and certainly aren't worth the
 * trouble).  The result is scaled so that it lies in the range 2^31..2^32-1,
 * ensuring the maximum available precision.
 *    The processing cost of this algorithm is one table lookup (using a 256
 * byte table, so the cache impact should be small) and four 32x32->64 bit
 * multiplies. */

unsigned int Reciprocal(unsigned int D, int &shift)
{
    /* Start by normalising D.  This ensures that we have as many bits as
     * possible (and is required for the rest of the algorithm to work). */
    unsigned int norm = CLZ(D);
    D <<= norm;
    shift += 63 - norm;

    if (unlikely(D == 0x80000000))
    {
        /* Need to handle the case of a one bit quotient specially, as in this
         * one case the clever stuff below just overflows.  This overflow is
         * also evident in this shift fixup. */
        shift -= 1;
        return D;
    }
    else
    {
        /* Get our first 8 significant bits by table lookup.  We use a nice
         * small table to ensure a small cache footprint (256 bytes). */
        unsigned int A = (D >> 23) & 0xff;
#ifdef __arm__
        /* It turns out that doing a byte fetch here is slower than doing a
         * full word fetch!  Here we take advantage of a curious quirk of ARM
         * addressing: a longword fetch from a non-aligned address returns the
         * word rotated so the addressed byte is in the bottom 8 bits! */
        unsigned int L = * (int *) & DivideLookup[A];
#else
#error "Gosh: a non ARM target!  Are you sure you've tested this?"
        unsigned int L = DivideLookup[A];
#endif
        unsigned int X = 0x80000000 | (L << 23);

        /* The calculation below is rather tricky.  Essentially we are
         * applying two rounds of Newton-Raphson to solve the equation
         *
         *      1/x - D = 0
         *
         * This, rather fortunately, has the Newton-Raphson step
         *
         *      x' = x(2 - xD)
         *
         * which we can do with two multiplies per step.  The initial estimate
         * above gives us a worst error of one part in 2^-8, and as it's easy
         * to see that this process squares the error, two rounds are enough
         * to reduce the error to one bit.
         *
         * Tricky scaling allows us to perform the subtraction on an invisible
         * bit: we work with X = 2^63 x ~~ 2^63 / D, and recall that
         * MulUU(A,B) returns 2^-32 A B, then:
         *
         *      2 MulUU(X, -MulUU(D, X)) = 2 2^-32 X * - 2^-32 D X
         *          = 2 2^-32 2^63 x * - 2^-32 2^63 D x
         *          = 2^32 x * - 2^31 D x
         *
         *  At this point we'll inject some magic: we know that
         *      2^32 - 2^31 D x = 2^31 (2 - D x)
         *  is very close to 2^31 (as x is close to 1/D), and so it doesn't
         *  matter that 2^32 isn't really there, and we can continue:
         *
         *          = 2^32 x * 2^31 (2-Dx)
         *          = 2^63 x'
         *          = X' .
         *
         * Sweet, eh? */
        X = MulUU(X, -MulUU(D, X)) << 1;
        X = MulUU(X, -MulUU(D, X)) << 1;
        return X;
    }
}



/* Denormalising, the conversion of a number together with its shift, into a
 * simple integer, is on the face of it as simple as returning X >> shift.
 * However, here we also take overflow into account, which complicates
 * things. */

unsigned int Denormalise(unsigned int X, int shift)
{
    if (shift < 0)
        /* Negative residual shift is a sign of probable trouble: numbers
         * should be arranged so there's some shift left to play with!  Never
         * mind, let's do the best we can... */
        if (CLZ(X) >= (unsigned int) -shift)
            /* Ok, we can afford this much left shift. */
            return X << -shift;
        else
            /* Out of bits.  Return maximum possible value, unless X is zero,
             * in which case we just return 0. */
            if (X == 0)
                return 0;
            else
                return ULONG_MAX;
    else if (shift < 32)
        /* The normal case. */
        return X >> shift;
    else
        /* Shifting by more than 32 is not properly defined, but we know to
         * return 0! */
        return 0;
}



/* Computes logarithm base 2 of input to about 22 bits of precision using
 * table lookup and linear interpolation, as for Reciprocal above.
 *
 * Here the input argument is taken to have 16 bits of fraction and 16 bits
 * of integer: this gives us a sensible output dynamic range, with an output
 * in the range +- 16.
 *
 * Computation proceeds as follows:
 *
 *  1. The input is normalised.  The normalising shift will simply be added
 *     into the final result (and is part of the reason for choosing base 2).
 *  2. The normalised input is separated into three fields, 1, A, B, exactly
 *     as for Reciprocal.
 *  3. The logarithm of A is computed by direct lookup.
 *  4. The remaining offset B is corrected for by linear interpolation.  In
 *     this case the scaling factor for B is also looked up.
 *
 * After normalisation write X = 2^31 x and X is decomposed into
 *
 *           31    m
 *      X = 2   + 2  A + B    (A n bits wide, B m bits wide, n+m=31).
 *
 *                -n                -31      m-1
 * Write a = 1 + 2  (A + 0.5), b = 2   (B - 2   ) and then x = a+b and we
 * compute
 *                                   b                      b
 *      log x = log (a+b) = log (a(1+-)) = log a + log (1 + -)
 *         2       2           2     a        2       2     a
 *                         b
 *            ~~ log a + ------
 *                  2    a ln 2
 *                  
 * The values log_2 a and 1/(a ln 2) are precomputed.  The offsets on A and B
 * used to calculate a and be are used to reduce the maximum value of b to
 * 2^(n+1), thus reducing the residual error. */

int log2(unsigned int X)
{
    /* First need to check for overflow.  Because linear approximation
     * overestimates the logarithm, we can't go all the way to the maximum
     * possible input without overflow.  Also, we have to return something
     * for log2(0), and we might as well return the smallest value (rather
     * than something close to the largest!) */
    if (X >= 0xFFFFFF80)
        return 0x7FFFFFFF;
    else if (X == 0)
        return 0x80000000;
    else
    {
        int shift = CLZ(X);
        X <<= shift;
        unsigned int A = (X & 0x7FFFFFFF) >> LOG2_M_BITS;
        int B = (X & LOG2_M_MASK) - LOG2_B_OFFSET;
        LOG2_LOOKUP Lookup = Log2Lookup[A];
        return ((15 - shift) << 27) + Lookup.Log + MulSS(Lookup.Scale, B);
    }
}



/* Computes exponential to the power 2 to about 22 bits of precision using
 * algorithms similar to those for Reciprocal and log2 above.
 *
 * Here the input argument has 27 bits of fraction and 5 signed bits of
 * integer, yielding 16 bits of fraction and 16 bits of integer.
 *
 * The computation process is very similar to that for log2, but the input
 * does not need normalisation: instead, the integer part of the input is
 * treated separately (as a shift on the final output).
 *
 * The input X = 2^27 x is decomposed into
 *
 *           27     m
 *      X = 2  S + 2 A + B      (A n bits wide, B m bits wide, n+m=27)
 *
 *                   -n                -27      m-1
 * and we write a = 2  (A + 0.5), b = 2   (B - 2   ) and x = S+a+b.  Then
 *
 *       x    S+a+b    S  a  b     S  a
 *      2  = 2      = 2  2  2  ~~ 2  2  (1 + b ln 2)
 *
 *             a      a
 *         = (2  + b 2  ln 2) << S.
 *
 * The constant 2^a is precomputed.  The multiplier 2^a ln 2 could also be
 * precomputed, but in this implementation is multiplied on the fly.
 *
 * The final required shift is returned instead of being applied to the
 * result: this allows accumulation of shifts if required without loss of
 * precision. */

unsigned int exp2(int X, int &shift)
{
    shift += 15 - (X >> 27);
    unsigned int A = (X & 0x07FFFFFF) >> EXP2_M_BITS;
    int B = (X & EXP2_M_MASK) - EXP2_B_OFFSET;
    unsigned int E = Exp2Lookup[A];
    return E + MulSS(B << 6, MulUU(E, EXP2_LN2));
}



/* Returns 1e6 * 20 * log_10(X), used for computing dB values for output to
 * the user.
 *
 * Calculate
 *                                 2e7            2e7     -27
 *      to_dB(X) = 2e7 * log  X = ------ log X = ------ (2   log2(X) + 16)
 *                          10    log 10    2    log 10
 *                                   2              2
 *
 * The two constants, 2^32 * 2^-27 * 2e7 / log_2 10, and 16 * 2e7 / log_2 10,
 * are precomputed by support-header.ph. */

int to_dB(unsigned int X)
{
    return TO_DB_OFFSET + MulSS(log2(X), TO_DB_FACTOR);
}


/* Returns 2^s * 10^(X/(20 * 1e6)), intended as an inverse to to_dB above,
 * where s is a shift normalisation to be applied by the caller.  Calculated
 * as:
 *                                            log 10
 *                      X               X        2
 *                     ---   ( log 10) ---    ------ X
 *                     2e7   (    2  ) 2e7      2e7      KX
 *      from_dB(X) = 10    = (2      )     = 2        = 2
 *
 *                    -s-16      27
 *                 = 2     exp2(2  KX)
 *
 * where
 *          log 10
 *             2
 *      K = ------
 *            2e7
 *
 * We now have to be rather careful about scaling X.  The factor 2^27*K above
 * is about 22.3, which restricts the maximum value of X/1e6 to 93.
 * Furthermore, to avoid losing precision, represent K below as 2^27 * K. */

unsigned int from_dB(int X, int &shift)
{
    /* Check for limits: if computing X<<5 loses bits then we overflow. */
    int XX = X << 5;
    if ((XX >> 5) == X)
    {
        unsigned int result = exp2(MulUS(FROM_DB_FACTOR, XX), shift);
        shift += 16;
        return result;
    }
    else
    {
        /* Oops.  Overflow!  Return a limiting value. */
        if (X > 0)
            shift += 16;
        else
            shift += 48;
        return 0xFFFFFFFF;
    }
}
