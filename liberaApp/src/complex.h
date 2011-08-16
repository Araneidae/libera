/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2005-2011  Michael Abbott, Diamond Light Source Ltd.
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

/* Support for complex numbers.  Complex arithmetic is only used in
 * situations where efficiency is not a particular issue. */

/* We use the C++ complex types: one of the few cases where the C++ type
 * system works really nicely -- almost as if it was designed for it ;) */
#include <complex>

/* The type REAL identifies the underlying floating point type of the complex
 * numbers that we use. */
typedef double REAL;
/* Corresponding to the typedef above, the EPICS database type mark for
 * complex numbers is defined here. */
#define DBF_REAL DBF_DOUBLE
typedef std::complex<REAL> complex;

/* I = sqrt(-1) */
#define I (complex(0, 1.0))
