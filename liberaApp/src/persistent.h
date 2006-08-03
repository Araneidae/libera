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

/* Interface to persistent state implementation. */


/* Each instance of the PERSISTENT class manages a persistent variable. */

class PERSISTENT_BASE
{
public:
    PERSISTENT_BASE();
    /* Specifies a name for the persistent state and attempts to read an
     * updated value from the state file.  True is returned iff a value was
     * read. */
    bool Initialise(const char *Name);

protected:
    virtual bool WriteValue(FILE * Output) = 0;
    virtual bool ReadValue(const char * String) = 0;
    virtual void BackupValue() = 0;
    virtual bool ValueChanged() = 0;
private:
    const char * Name;
    PERSISTENT_BASE * Next;
    friend bool WritePersistentState(FILE *File);
    friend bool CheckStateChanged();
};

template<class T>
class PERSISTENT : public PERSISTENT_BASE
{
public:
    /* Binds the Value to a persistence state.  All changes to this variable
     * will be tracked in the persistent state file. */
    PERSISTENT(T &Value);

private:
    bool WriteValue(FILE * Output);
    bool ReadValue(const char * String);
    void BackupValue();
    bool ValueChanged();
    T &Value;
    T OldValue;
};

typedef PERSISTENT<int>    PERSISTENT_INT;
typedef PERSISTENT<bool>   PERSISTENT_BOOL;
//typedef PERSISTENT<double> PERSISTENT_DOUBLE;



/* Calling this function is enough to establish persistence for the given
 * value. */
template<class T>
void Persistent(const char * Name, T &Value)
{
    PERSISTENT<T> & Persistence = * new PERSISTENT<T>(Value);
    Persistence.Initialise(Name);
}


bool InitialisePersistentState(const char * StateFileName);
void TerminatePersistentState();
