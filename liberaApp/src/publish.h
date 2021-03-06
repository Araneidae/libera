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


/* This handles the publishing of variables and other internal state as EPICS
 * process variables.
 *
 * The code here is a scary and extremely horrible mix of macros and
 * templates: the goal is to make the job of publishing values to EPICS as
 * smooth as possible by hiding as much of the nasty stuff as possible here.
 *
 * This code is also getting in need of reengineering: there is far to much
 * duplication of incomplete functionality! */



/* Helper routine for concatenating or just copying strings. */
const char * Concat(
    const char * Prefix, const char * Body="", const char * Suffix="");



/*****************************************************************************/
/*                                                                           */
/*                        Publish Simple Variables                           */
/*                                                                           */
/*****************************************************************************/


/* For the special case of variables which are just read or written in-place
 * with no further action, we also provide publish routines to support this. */

#define DECLARE_PUBLISH_VAR_IN(record) \
    void Publish_##record(const char * Name, TYPEOF(record) &Variable)

#define DECLARE_PUBLISH_VAR_OUT(record) \
    void Publish_##record(const char * Name, TYPEOF(record) &Variable)


/* Declaration of Publish_<record> methods for simple variable PV access.  For
 * each of these the record implementation simply returns the current value. */
DECLARE_PUBLISH_VAR_IN(longin);
DECLARE_PUBLISH_VAR_OUT(longout);
DECLARE_PUBLISH_VAR_IN(ai);
DECLARE_PUBLISH_VAR_OUT(ao);
DECLARE_PUBLISH_VAR_IN(bi);
DECLARE_PUBLISH_VAR_OUT(bo);
DECLARE_PUBLISH_VAR_IN(stringin);
DECLARE_PUBLISH_VAR_OUT(stringout);
DECLARE_PUBLISH_VAR_IN(mbbi);
DECLARE_PUBLISH_VAR_OUT(mbbo);



/*****************************************************************************/
/*                                                                           */
/*                    Publish Support Class and Macros                       */
/*                                                                           */
/*****************************************************************************/

/* The following code supports the reading and writing of simple single
 * values (all except waveform) through the use of access routines.   To
 * simplify the use of the Publish_ methods we have to jump through some
 * rather tricky C++ hoops below: indeed, some of this code is right on (or
 * maybe over) the edge of what the compiler will swallow.
 *    A large chunk of the nastyness here is because C++ doesn't understand
 * closures.  */


/* Support class to actually implement the appropriate interface.
 * Unfortunately we need one of these for each interface we're going to
 * support!
 *    This class simply hangs onto the interested class T and a member
 * pointer to the function that's actually going to implement the reading or
 * writing, and calls this function to implement the required EPICS support
 * interface. */
template<class T, class R>
class CLOSURE_IN : public I_READER<R>
{
public:
    CLOSURE_IN(T&t, bool (T::*f)(R&)) : t(t), f(f) {}
    bool read(R& arg) { return (t.*f)(arg); }
private:
    T & t;
    bool (T::*f)(R&);
};

template<class T, class R>
class CLOSURE_OUT : public I_WRITER<R>
{
public:
    CLOSURE_OUT(T&t, bool (T::*f)(R), bool (T::*i)(R&)) :
        t(t), i(i), f(f), u(NULL), v(NULL) {}
    CLOSURE_OUT(T&t, bool (T::*f)(R), R T::*v) :
        t(t), i(NULL), f(f), u(NULL), v(v) {}
    CLOSURE_OUT(T&t, void (T::*u)(), R T::*v) :
        t(t), i(NULL), f(NULL), u(u), v(v) {}
    bool init(R& arg)
    {
        if (v == (R T::*) NULL)
            return (t.*i)(arg);
        else
        {
            arg = t.*v;
            return true;
        }
    }
    bool write(R arg)
    {
        if (f == (bool (T::*)(R)) NULL)
        {
            t.*v = arg;
            (t.*u)();
            return true;
        }
        else
            return (t.*f)(arg);
    }
private:
    T & t;
    bool (T::*i)(R&);
    bool (T::*f)(R);
    void (T::*u)();
    R T::*v;
};


/* This bit is evil.  The cute macros below need to know the type of `this`,
 * and normally __typeof__(this) does the trick: but not when taking the
 * address of a member function!
 *     For this macro to work it is necessary that the class T have a default
 * constructor.  However, it isn't required to be accessible or even to be
 * implemented: a private unimplemented declaration suffices! */
template<class T> class ID : public T { };

/* This rather naughty macro assumes that it is always used to publish
 * methods of the calling class. */
#define PUBLISH_METHOD_IN(record, name, method) \
    Publish_##record(name, \
        * new CLOSURE_IN<__typeof__(*this), TYPEOF(record)>( \
            *(this), &ID<__typeof__(*this)>::method))

#define PUBLISH_METHOD_OUT(record, name, method, init) \
    Publish_##record(name, \
        * new CLOSURE_OUT<__typeof__(*this), TYPEOF(record)>( \
            *(this), &ID<__typeof__(*this)>::method, \
            &ID<__typeof__(*this)>::init))

#define PUBLISH_METHOD_IN_OUT( \
        record_in, record_out, name, method_read, method_write) \
    PUBLISH_METHOD_IN(record_in, name, method_read); \
    PUBLISH_METHOD_OUT(record_out, name, method_write, method_read)


/* Something quite a bit simpler: a closure for a method with no arguments,
 * used for simple actions where the value of the record is immaterial. */

template<class T>
class CLOSURE_ACTION : public I_bo
{
public:
    CLOSURE_ACTION(T&t, bool (T::*f)()): t(t), f(f) {}
    bool init(bool& arg) { arg = true; return true; }
    bool write(bool) { return (t.*f)(); }
private:
    T & t;
    bool (T::*f)();
};

#define PUBLISH_METHOD_ACTION(name, method) \
    Publish_bo(name, \
        * new CLOSURE_ACTION<__typeof__(*this)>( \
            *this, &ID<__typeof__(*this)>::method))



/* We can play the same game for unbound functions, though of course here the
 * story is somewhat simpler. */

template<class T>
class WRAPPER_IN : public I_READER<T>
{
public:
    WRAPPER_IN(bool (*f)(T&)) : f(f) {}
    bool read(T &arg) { return (*f)(arg); }
private:
    bool (*const f)(T&);
};


#define PUBLISH_FUNCTION_IN(record, name, function) \
    Publish_##record(name, \
        * new WRAPPER_IN<TYPEOF(record)>(function))




/* Yet another approach to publishing values! */

template<class T>
class CONFIGURATION_VALUE : public I_WRITER<T>
{
public:
    /* In this case the update is performed by this class, but an update
     * method will also be called on completion. */
    CONFIGURATION_VALUE(T &Parameter, void (*OnUpdate)()) :
        Parameter(Parameter),
        OnUpdate(OnUpdate),
        DoUpdate(NULL)
    {
    }

    /* In this case the callback method needs to do the update as well. */
    CONFIGURATION_VALUE(T &Parameter, bool (*DoUpdate)(T)) :
        Parameter(Parameter),
        OnUpdate(NULL),
        DoUpdate(DoUpdate)
    {
    }

    bool init(T &Result)
    {
        Result = Parameter;
        return true;
    }
    bool write(T Value)
    {
        if (DoUpdate)
            return DoUpdate(Value);
        else
        {
            Parameter = Value;
            if (OnUpdate)
                OnUpdate();
            return true;
        }
    }

private:
    T &Parameter;
    void (*OnUpdate)();
    bool (*DoUpdate)(T);
};


/* Publishes managed persistent configuration values with associated recout
 * record.  The Action function is called whenever the recout record is
 * updated and the configuration value is automatically managed as a
 * persistent value. */
#define PUBLISH_CONFIGURATION(record, Name, Value, Action) \
    ( { \
        CONFIGURATION_VALUE<__typeof__(Value)> & ConfigValue = \
            * new CONFIGURATION_VALUE<__typeof__(Value)>(Value, Action); \
        Publish_##record(Name, ConfigValue); \
        Persistent(Name, Value); \
    } )

/* To specify no action for PUBLISH_CONFIGURATION the NULL_ACTION action can
 * be specified. */
#define NULL_ACTION ((void(*)())NULL)

#define PUBLISH_FUNCTION_OUT(record, Name, Value, Action) \
    Publish_##record(Name, \
        * new CONFIGURATION_VALUE<__typeof__(Value)>(Value, Action))


/* Yet another wrapper.  A bit of refactoring is going to be needed soon...
 * This wraps a record with an action but no associated data. */
class ACTION_VALUE : public I_bo
{
public:
    ACTION_VALUE(void (*Action)()) : Action(Action) {}
    bool init (bool &Result) { Result = true; return true; }
    bool write(bool) { Action(); return true; }
private:
    void (*Action)();
};

/* Associates an action with processing a record.  No value is handled. */
#define PUBLISH_ACTION(Name, Action) \
    Publish_bo(Name, * new ACTION_VALUE(Action))



/*****************************************************************************/
/*                                                                           */
/*                        Publish Readback Variable                          */
/*                                                                           */
/*****************************************************************************/


/* A simple class for publishing self-updating values. */

template<class T>
    class UPDATER : public I_READER<T>
{
public:
    UPDATER(T InitialValue);
    void Write(T NewValue);
    T Read() { return Value; }
private:
    bool read(T &Value);
    bool EnableIoIntr(I_INTR & Intr);
    T Value;
    I_INTR * iIntr;
};

typedef UPDATER<bool> UPDATER_bool;
typedef UPDATER<int>  UPDATER_int;



template<class T> class READBACK
{
public:
    READBACK(T InitialValue, bool (*OnUpdate)(T));
    void Write(T NewValue);

private:
    bool UserUpdate(T NewValue);
    T Value;
    bool (*const OnUpdate)(T);
public:
    UPDATER<T> Writer;
    CLOSURE_OUT<READBACK<T>, T> Reader;
};

typedef READBACK<bool> READBACK_bool;
typedef READBACK<int>  READBACK_mbb;
typedef READBACK<int>  READBACK_int;


#define PUBLISH_READBACK(recin, recout, Name, InitialValue, Action) \
    ( { \
        READBACK<__typeof__(InitialValue)> * Readback = \
            new READBACK<__typeof__(InitialValue)>(InitialValue, Action); \
        Publish_##recin(Name "_R", Readback->Writer); \
        Publish_##recout(Name, Readback->Reader); \
        Readback; \
    } )

#define PUBLISH_READBACK_CONFIGURATION(recin, recout, Name, Value, Action) \
    ( { \
        Persistent(Name, Value); \
        READBACK<__typeof__(Value)> * Readback = \
            PUBLISH_READBACK(recin, recout, Name, Value, Action); \
        Readback; \
    } )
