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


/* This handles the publishing of variables and other internal state as EPICS
 * process variables. */



/* Publication for generic read and write methods. */

/* This macro is used to declare general purpose EPICS variable publishing
 * methods for publishing an I_<record> interface by name. */
#define DECLARE_PUBLISH(record) \
    void Publish_##record(const char * Name, I_##record & Record)
    

/* Record implementation lookup, used by the drivers.cpp implementation of
 * Libera record support. */
#define DECLARE_SEARCH(record) \
    I_##record * Search_##record(const char *Name)



/* Declaration of Publish_<record> methods for each supported record type.
 * Every visible PV should be made available through a call to the
 * appropriate one of these, or perhaps one of the wrappers below. */
DECLARE_PUBLISH(longin);
DECLARE_PUBLISH(longout);
DECLARE_PUBLISH(ai);
DECLARE_PUBLISH(ao);
DECLARE_PUBLISH(bi);
DECLARE_PUBLISH(bo);
DECLARE_PUBLISH(waveform);


/* Declaration of the search methods used in driver implementation. */
DECLARE_SEARCH(longin);
DECLARE_SEARCH(longout);
DECLARE_SEARCH(ai);
DECLARE_SEARCH(ao);
DECLARE_SEARCH(bi);
DECLARE_SEARCH(bo);
DECLARE_SEARCH(waveform);



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                        Publish Simple Variables                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* For the special case of variables which are just read or written in-place
 * with no further action, we also provide publish routines to support this. */

#define DECLARE_PUBLISH_VAR_IN(record, type) \
    void Publish_##record(const char * Name, type &Variable)

#define DECLARE_PUBLISH_VAR_OUT(record, type) \
    void Publish_##record(const char * Name, type &Variable)


/* Declaration of Publish_<record> methods for simple variable PV access.  For
 * each of these the record implementation simply returns the current value. */
DECLARE_PUBLISH_VAR_IN(longin, int);
DECLARE_PUBLISH_VAR_IN(ai, double);
DECLARE_PUBLISH_VAR_IN(bi, bool);

DECLARE_PUBLISH_VAR_OUT(longout, int);
DECLARE_PUBLISH_VAR_OUT(ao, double);
DECLARE_PUBLISH_VAR_OUT(bo, bool);



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                    Publish Support Class and Macros                       */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* The following code supports the reading and writing of simple single
 * values (all except waveform) through the use of access routines.   To
 * simplify the use of the Publish_ methods we have to jump through some
 * rather tricky C++ hoops below: indeed, some of this code is right on (or
 * maybe over) the edge of what the compiler will swallow.
 *    This is all because C++ doesn't understand closures. */

/* Support class to actually implement the appropriate interface.
 * Unfortunately we need one of these for each interface we're going to
 * support!
 *    This class simply hangs onto the interested class T and a member
 * pointer to the function that's actually going to implement the reading or
 * writing, and calls this function to implement the required EPICS support
 * interface. */
#define DECLARE_CLOSURE(record, type, action) \
    template<class T> class CLOSURE_##record : public I_##record \
    { \
    public: \
        CLOSURE_##record(T&t, bool (T::*f)(type)) : t(t), f(f) {} \
        bool action(type arg) { return (t.*f)(arg); } \
    private: \
        T & t; \
        bool (T::*f)(type); \
    }

DECLARE_CLOSURE(longin, int&, read);
DECLARE_CLOSURE(longout, int, write);
DECLARE_CLOSURE(ai, double&, read);
DECLARE_CLOSURE(ao, double, write);
DECLARE_CLOSURE(bi, bool&, read);
DECLARE_CLOSURE(bo, bool, write);

/* This is the evil bit.  The cute macro at the end needs to know the type of
 * `this`, and normally typeof(this) does the trick: but not when taking the
 * address of a member function!
 *     For this macro to work it is necessary that the class T have a default
 * constructor.  However, it isn't required to be accessible or even to be
 * implemented: a private unimplemented declaration suffices! */
template<class T> class ID : public T { };

/* This rather naughtly macro assumes that it is always used to publish
 * methods of the calling class. */
#define PUBLISH_METHOD(record, name, method) \
    Publish_##record(name, \
        * new CLOSURE_##record<typeof(*this)>( \
            *(this), & ID<typeof(*this)>::method))


/* We can play the same game for unbound functions, though of course here the
 * story is somewhat simpler. */

#define DECLARE_FUNCTION_WRAPPER(record, type, action) \
    class WRAPPER_##record : public I_##record \
    { \
    public: \
        WRAPPER_##record(bool (*f)(type, void*), void *c) : f(f), c(c) {} \
        bool action(type arg) { return (*f)(arg, c); } \
    private: \
        bool (*const f)(type, void*); \
        void * const c; \
    }


DECLARE_FUNCTION_WRAPPER(longin, int&, read);
DECLARE_FUNCTION_WRAPPER(longout, int, write);
DECLARE_FUNCTION_WRAPPER(ai, double&, read);
DECLARE_FUNCTION_WRAPPER(ao, double, write);
DECLARE_FUNCTION_WRAPPER(bi, bool&, read);
DECLARE_FUNCTION_WRAPPER(bo, bool, write);

#define PUBLISH_FUNCTION(record, name, function, context) \
    Publish_##record(name, * new WRAPPER_##record(function, context))


/* Helper routine for concatenating strings. */
const char * Concat(const char * Prefix, const char * Suffix);
