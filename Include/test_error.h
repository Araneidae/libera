/* This file is part of the Libera EPICS Driver,
 *
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


#include <errno.h>

void print_error(const char * Message, const char * FileName, int LineNumber);


/* Macro derived from the kernel to tell the compiler that x is quite
 * unlikely to be true. */
#define unlikely(x)   __builtin_expect((x), 0)

/* Helper macros for OS calls: in all cases the call is wrapped by a macro
 * which converts the return code into a boolean value.  If the call fails
 * then an automatically generated error message (including filename and line
 * number) is printed.
 *     In all cases true is returned iff the call is successful.
 *
 * There are three main cases, depending on how the return value is
 * interpreted:
 *
 *  TEST_IO(expression)
 *      Computes expression, typically of the form
 *          result = function(arguments)
 *      and reports an error message if the result is -1.
 *
 *  TEST_NULL(expression)
 *      Reports an error message if expression is NULL.
 *
 *  TEST_0(expression)
 *      Computes expression and reports an error message if the expressionnn
 *      is non zero.  This is designed for use with the pthread_ family of
 *      functions, and the expression is assigned to errno if non zero.
 *
 *  TEST_OK(expression)
 *      Reports an error message if the expression is false.
 */


/* Building block for the four helper macros listed above, parameterised by
 * test to perform on expression.
 *
 * The natural way to write this is:
 *
 *  #define TEST_(COND, expr) \
 *      (unlikely(!COND(expr)) ? \
 *          print_error(#expr, __FILE__, __LINE__), false : true)
 *
 * Unfortunately this version makes the compiler very noisy when the result
 * of TEST_(...) is ignored, so we go for this quieter implementation
 * instead: */

#define TEST_(COND, expr) \
    ( { \
        bool __ok__ = COND(expr); \
        if (unlikely(!__ok__)) \
            print_error(#expr, __FILE__, __LINE__); \
        __ok__; \
    } )

#define COND_IO(expr)       ((int) (expr) != -1)
#define COND_NULL(expr)     ((expr) != NULL)
#define COND_OK(expr)       ((bool) (expr))
#define COND_0(expr) \
    ( { \
        int __rc__ = (expr); \
        if (unlikely(__rc__ != 0)) \
            errno = __rc__; \
        __rc__ == 0; \
    } )

#define TEST_IO(expr)       TEST_(COND_IO,   expr)
#define TEST_NULL(expr)     TEST_(COND_NULL, expr)
#define TEST_OK(expr)       TEST_(COND_OK,   expr)
#define TEST_0(expr)        TEST_(COND_0,    expr)


/* These two macros facilitate using the macros above by creating if
 * expressions that're slightly more sensible looking than ?: in context. */
#define DO_(action)                     ({action; true;})
#define IF_(test, iftrue)               ((test) ? (iftrue) : true)
#define IF_ELSE(test, iftrue, iffalse)  ((test) ? (iftrue) : (iffalse))



/* A rather randomly placed helper routine.  This and its equivalents are
 * defined all over the place, but there doesn't appear to be a definitive
 * definition anywhere. */
#define ARRAY_SIZE(a)   (sizeof(a)/sizeof((a)[0]))
