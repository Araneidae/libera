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


/* Conversion from cartesian to polar coordinates using purely integer
 * arithmetic: the CORDIC algorithm.
 *
 * The implementation here has been distilled from references and examples and
 * analysed in detail by Michael Abbott, Diamond Light Source, 2005.
 *
 * The original Cordic paper is:
 *      Volder, J.E., 1959; "The CORDIC Trigonometric Computing Technique",
 *      IRE Transactions on Electronic Computers, V.  EC-8, No. 3, pp. 330-334
 * 
 * Other sources for the Cordic algorithm can be found at:
 * 
 * http://www.dspguru.com/info/faqs/cordic.htm
 *      This page includes an overview of the algorithm.  Most of the web
 *      links are dead, however.
 *      
 * http://www.andraka.com/cordic.htm
 *      Contains a preprint of the paper "A survey of CORDIC algorithms for
 *      FPGA based computers", Ray Andraka, 1998.
 *
 * Google for CORDIC for other references.
 *
 * Note that variants of the Cordic algorithm can be used to compute the
 * trigonometric and hyperbolic functions as well as their inverses.
 *      
 *
 * The Cordic algorithm written here iteratively computes the magnitude of a
 * vector (x,y) using simple arithmetic and shifts: this is therefore well
 * suited to implementation on a device such as the ARM or on an FPGA.
 *
 * The algorithm works as follows.  Let (x,y) be given.  First compute
 *      x_1 = max(|x|, |y|)
 *      y_1 = min(|x|, |y|)
 * so that the initial condition below is satisfied.  The iterative step is
 * then applied to obtain the required precision.
 *
 *      Initial condition:
 *              0 <= y_1 <= x_1
 *
 *      Iterative step (for n >= 1):
 *              x_{n+1} = x_n + y_n/2^n 
 *              y_{n+1} = |y_n - x_n/2^n|
 *
 * At each step it is easy to calculate that
 *      |(x_{n+1}, y_{n+1})| = sqrt(1 + 2^{-2n}) * |(x_n,y_n)|
 * The correction factor sqrt((1+1/2)*(1+1/4)*(1+1/8)*...) can be calculated
 * as approximately 1.1644353.
 *
 * Note that a simpler initial condition (x_0, y_0 non-negative) can be used
 * by starting the iteration at n=0, but this affects overflow compensation:
 * see comments in the code below.
 *
 * After N steps the magnitude can be read from x_{N+1}, as y_{N+1} rapidly
 * converges to zero.  Indeed, it can be shown that at each stage the
 * following loop invariant is satisfied:
 *
 *      Loop invariant:
 *              0 <= y_n <= 2 * 2^-n * x_n .
 *
 * It is then also easy to see that the relative error, which we can define
 * as 
 *
 *            sqrt(x_n^2 + y_n^2) - x_n       (    (x_n)2)
 *      e_n = ------------------------- = sqrt(1 + (---) ) - 1 ,
 *                       x_n                  (    (y_n) )
 *
 * satisfies the inequality
 *
 *      0 <= e_n <= 2^-2n .
 *
 * In particular, it's clear from this that the Cordic algorithm generates
 * two significant bits of result per iteration. */


#include "cordic.h"



int CordicMagnitude(int x, int y, int Iterations)
{
    /* Get the vector into the first quadrant of the plane. */
    if (x < 0)  x = -x;
    if (y < 0)  y = -y;

    /* The Cordic algorithm scales up the result by a factor of approximately
     * 1.16 (multiply out sqrt(1+2^-2n) for n counting up from 1), so we
     * prescale by dividing by two to avoid overflow.
     *     The signed/unsigned dance is to avoid trouble with 0x80000000:
     * remember that (x,y) are supposed to be *unsigned* at this point! */
    x = (int) ((unsigned int) x >> 1);
    y = (int) ((unsigned int) y >> 1);

    /* Ensure that x >= y to ensure the loop invariant is satisfied for n=1
     * before entering the iteration.
     *    Note that it is possible to omit this prescaling step, as in fact
     * the initial condition 0<=x_0, 0<=y_0 is sufficient to ensure that
     * 0 <= y_1 <= x_1 after one iteration of the loop.  However doing this
     * adds an extra factor of sqrt(2) to the result, which would force the
     * scaling above to be by a factor of 4 instead of 2.  We get another 
     * half bit of result this way and the adjustment below is marginally
     * cheaper than an extra iteration. */
    if (y > x)
    {
        int oldx = x;
        x = y;
        y = oldx;
    }

    /* Now apply the CORDIC iteration algorithm.  The transformation
     *    (x,y) := (x+a*y, y-a*x)
     * has two quite important properties:
     * 1. The magnitude of the vector |(x,y)| grows by a factor sqrt(1+a^2)
     *    on each pass.  In general this must be compensated for in some way,
     *    but this is at the the heart of why the CORDIC algorithm works so
     *    well.
     * 2. So long as we start with x>=y and take a = (1/2, 1/4, 1/8, ...)
     *    then the magnitude of y will rapidly go to zero. */
    for (int i = 1; i <= Iterations; i ++)
    {
        int oldx = x;
        x += y >> i;
        y -= oldx >> i;
        if (y < 0)  y = -y;
    }

    /* In our application the remaining scaling factor is not important, so
     * we just return the reduced data directly. */
    return x;
}
