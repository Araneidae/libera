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

#include "drivers.h"
#include "publish.h"



/* Simple helper routine for building published names.  As we never have to
 * worry about end of lifetime, this is pretty easy. */
const char * Concat(const char * Prefix, const char * Suffix)
{
    char * Result = new char[strlen(Prefix) + strlen(Suffix) + 1];
    strcpy(Result, Prefix);
    strcat(Result, Suffix);
    return Result;
}


/****************************************************************************/
/*                                                                          */
/*                         Generic Publish by Name                          */
/*                                                                          */
/****************************************************************************/



/* A simple lookup table class. */

class LOOKUP
{
public:
    LOOKUP()
    {
        List = NULL;
    }

    /* Method to look up by name.  Returns NULL if not found. */
    I_RECORD * Find(const char * Name)
    {
        for (ENTRY * entry = List; entry != NULL; entry = entry->Next)
            if (strcmp(entry->Name, Name) == 0)
                return entry->Value;
        return NULL;
    }

    /* Inserts a new entry into the lookup table.  Note that the given string
     * is *NOT* copied, so the caller should ensure that it is persistent. */
    void Insert(const char * Name, I_RECORD * Value)
    {
        ENTRY * Entry = new ENTRY;
        Entry->Next = List;
        Entry->Name = Name;
        Entry->Value = Value;
        List = Entry;
    }

private:
    struct ENTRY
    {
        ENTRY * Next;
        const char * Name;
        I_RECORD * Value;
    };

    ENTRY * List;
};



//#define PRINTF(args...) printf(args)
#define PRINTF(args...)


/* This macro builds the appropriate Publish_<record> and Search_<record>
 * methods and initialises any associated static state. */
#define DEFINE_PUBLISH(record) \
    static LOOKUP Lookup_##record; \
    DECLARE_PUBLISH(record) \
    { \
        PRINTF("Publishing %s %s\n", Name, #record); \
        Lookup_##record.Insert(Name, &Record); \
    } \
    DECLARE_SEARCH(record) \
    { \
        return dynamic_cast<I_##record *>(Lookup_##record.Find(Name)); \
    }


DEFINE_PUBLISH(longin);
DEFINE_PUBLISH(longout);
DEFINE_PUBLISH(ai);
DEFINE_PUBLISH(ao);
DEFINE_PUBLISH(bi);
DEFINE_PUBLISH(bo);
DEFINE_PUBLISH(waveform);



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



DEFINE_PUBLISH_VAR_IN(longin);
DEFINE_PUBLISH_VAR_IN(ai);
DEFINE_PUBLISH_VAR_IN(bi);

DEFINE_PUBLISH_VAR_OUT(longout);
DEFINE_PUBLISH_VAR_OUT(ao);
DEFINE_PUBLISH_VAR_OUT(bo);
