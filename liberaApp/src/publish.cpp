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
 * The interface consists of two parts:
 *  1.  A Publish_<record> method for each supported PV type.  This adds the
 *      named variable to a dictionary of EPICS names.
 *  2.  A Search_<record> method for each PV type.  This provides the binding
 *      point to the EPICS record drivers code.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "device.h"
#include "persistent.h"
#include "publish.h"


//#define PRINTF(args...) printf(args)
#define PRINTF(args...)



/* Simple helper routine for building published names.  As we never have to
 * worry about end of lifetime, this is pretty easy. */
const char * Concat(
    const char * Prefix, const char * Body, const char * Suffix)
{
    char * Result = new char[
        strlen(Prefix) + strlen(Body) + strlen(Suffix) + 1];
    strcpy(Result, Prefix);
    strcat(Result, Body);
    strcat(Result, Suffix);
    return Result;
}


/****************************************************************************/
/*                                                                          */
/*                      Map Variables to Input PVs                          */
/*                                                                          */
/****************************************************************************/


/* This class encapsulates the task of making a variable of type T available
 * as an EPICS process variable.  The class implements the appropriate reader
 * interface expected by the driver support layer. */

template<class T> class PUBLISH_READ : public I_READER<T>
{
public:
    PUBLISH_READ(const T &Variable, const char *Name) :
        Variable(Variable), Name(Name)
    {
        PRINTF("Publish %s: %p\n", Name, &Variable);
    }

    /* Implement the read method of the I_READER interface. */
    bool read(T & Value)
    {
        PRINTF("Read %s: %d\n", Name, (int)  Variable);
        Value = Variable;
        return true;
    }

private:
    const T & Variable;
    const char *Name;
};


#define DEFINE_PUBLISH_VAR_IN(record) \
    DECLARE_PUBLISH_VAR_IN(record) \
    { \
        Publish_##record(Name, \
            *new PUBLISH_READ<TYPEOF(record)>(Variable, Name)); \
    }




template<class T> class PUBLISH_WRITE : public I_WRITER<T>
{
public:
    PUBLISH_WRITE(T &Variable) : Variable(Variable) { }

    /* For the init method just read the current value of the variable! */
    bool init(T &Result)
    {
        Result = Variable;
        return true;
    }

    /* Implement the write method of the I_WRITER interface. */
    bool write(T Value)
    {
        Variable = Value;
        return true;
    }

private:
    T & Variable;
};


#define DEFINE_PUBLISH_VAR_OUT(record) \
    DECLARE_PUBLISH_VAR_OUT(record) \
    { \
        Publish_##record(Name, \
            *new PUBLISH_WRITE<TYPEOF(record)>(Variable)); \
    }


/* For EPICS strings we need to treat things slightly differently as C++
 * doesn't allow assignment of arrays.  Fortunately it seems that simply
 * defining the following is enough. */

template<>
bool PUBLISH_READ<EPICS_STRING>::read(EPICS_STRING &Value)
{
    PRINTF("Read %s: %d\n", Name, (int)  Variable);
    CopyEpicsString(Variable, Value);
    return true;
}

template<>
bool PUBLISH_WRITE<EPICS_STRING>::init(EPICS_STRING &Result)
{
    CopyEpicsString(Variable, Result);
    return true;
}

template<>
bool PUBLISH_WRITE<EPICS_STRING>::write(EPICS_STRING Value)
{
    CopyEpicsString(Value, Variable);
    return true;
}



DEFINE_PUBLISH_VAR_IN(longin);
DEFINE_PUBLISH_VAR_OUT(longout);
DEFINE_PUBLISH_VAR_IN(ai);
DEFINE_PUBLISH_VAR_OUT(ao);
DEFINE_PUBLISH_VAR_IN(bi);
DEFINE_PUBLISH_VAR_OUT(bo);
DEFINE_PUBLISH_VAR_IN(stringin);
DEFINE_PUBLISH_VAR_OUT(stringout);
DEFINE_PUBLISH_VAR_IN(mbbi);
DEFINE_PUBLISH_VAR_OUT(mbbo);



/****************************************************************************/
/*                                                                          */
/*                     UPDATER and READBACK classes                         */
/*                                                                          */
/****************************************************************************/


template<class T>
    UPDATER<T>::UPDATER(T InitialValue)
{
    Value = InitialValue;
    iIntr = NULL;
}


template<class T>
    void UPDATER<T>::Write(T NewValue)
{
    Value = NewValue;
    if (iIntr != NULL)
        iIntr->IoIntr();
}


template<class T>
    bool UPDATER<T>::read(T &ValueRead)
{
    ValueRead = Value;
    return true;
}

template<class T>
    bool UPDATER<T>::EnableIoIntr(I_INTR & Intr)
{
    iIntr = &Intr;
    return true;
}


template class UPDATER<bool>;
template class UPDATER<int>;


template<class T>
    READBACK<T>::READBACK(T InitialValue, bool (*OnUpdate)(T)) :

    OnUpdate(OnUpdate),
    Writer(Value),
    Reader(*this, &READBACK<T>::UserUpdate, &READBACK<T>::Value)
{
    Value = InitialValue;
}



/* This is called when the system wants to change the value. */

template<class T>
    void READBACK<T>::Write(T NewValue)
{
    if (NewValue != Value)
    {
        Value = NewValue;
        Writer.Write(NewValue);
    }
}


/* This is called when the controlling PV is changed: we can regard this as
 * an operator change. */

template<class T>
    bool READBACK<T>::UserUpdate(T NewValue)
{
    if (NewValue == Value)
        return true;
    else
    {
        bool ok = OnUpdate(NewValue);
        if (ok)
            Value = NewValue;
        return ok;
    }
}


template class READBACK<bool>;
template class READBACK<int>;
