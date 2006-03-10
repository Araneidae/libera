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

/* Implemention of persistent state. */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>

#include "thread.h"
#include "hardware.h"
#include "persistent.h"


static const char * StateFileName = NULL;
static PERSISTENT_BASE * PersistentList = NULL;


/* Checks whether any persistent variables have changed since they were
 * loaded or last written. */

bool CheckStateChanged()
{
    for (PERSISTENT_BASE * Entry = PersistentList; Entry != NULL;
         Entry = Entry->Next)
        if (Entry->ValueChanged())
            return true;
    return false;
}


/* Writes all initialised persistent variables to the given state file. */

bool WritePersistentState(FILE *File)
{
    bool Ok = true;
    for (PERSISTENT_BASE * Entry = PersistentList; Ok && Entry != NULL;
         Entry = Entry->Next)
    {
        Ok = fprintf(File, "%s=", Entry->Name) > 0  &&
             Entry->WriteValue(File)  &&
             fprintf(File, "\n") == 1;
        Entry->BackupValue();
    }
    return Ok;
}



/* Performs a safe update of the persistent state file: writes to a backup
 * and renames it into place only if the state was written successfully. */

#define BACKUP ".backup"
static void WriteStateFile()
{
    /* Only actually write anything if:
     *  1. We have a file name to write to(!) and
     *  2. Anything has actually changed.
     * Ensuring that we only write if variables has changed is important if
     * the state file is hosted on the local flash file system. */
    if (StateFileName != NULL  &&  CheckStateChanged())
    {
        char BackupFileName[strlen(StateFileName) + strlen(BACKUP) + 1];
        strcpy(BackupFileName, StateFileName);
        strcat(BackupFileName, BACKUP);
        FILE * Backup = fopen(BackupFileName, "w");
        if (Backup == NULL)
            printf("Unable to write to state file \"%s\"\n", BackupFileName);
        else
        {
            char TimeBuffer[80];
            time_t Now = time(NULL);
            bool Ok =
                fprintf(Backup, "# Written: %s",
                    ctime_r(&Now, TimeBuffer)) > 0  &&
                WritePersistentState(Backup);
            fclose(Backup);
            if (Ok)
                TEST_(rename, BackupFileName, StateFileName);
        }
    }
}


PERSISTENT_BASE::PERSISTENT_BASE()
{
    Name = NULL;
}


bool PERSISTENT_BASE::Initialise(const char *SetName)
{
    bool Initialised = false;
    Name = SetName;

    /* Reading the initial value presents us with an interesting problem.
     * It is easier but quite inefficient to read and search the state file
     * separately for each persistent variable.  However, as there are very
     * few variables to read, this is really quite sufficient. */
    if (StateFileName != NULL)
    {
        FILE * Input = fopen(StateFileName, "r");
        if (Input != NULL)
        {
            int NameLength = strlen(Name);
            char Line[1024];
            while (fgets(Line, sizeof(Line), Input))
            {
                if (strncmp(Line, Name, NameLength) == 0  &&
                    Line[NameLength] == '=')
                {
                    /* Ok.  Found a match for our name.  Lets see if we can
                     * parse it!  Invoke the instance specific parser. */
                    int Length = strlen(Line);
                    if (Length > 0  &&  Line[Length - 1] == '\n')
                    {
                        Line[Length - 1] = '\0';
                        Initialised = ReadValue(Line + NameLength + 1);
                    }
                    if (!Initialised)
                        /* Odd.  The file must be malformed. */
                        printf("Malformed entry \"%s\" in state file \"%s\"\n",
                            Line, StateFileName);
                    break;
                }
            }
            fclose(Input);
        }
    }

    /* Mark the current value as saved: no need to write the state file just
     * yet! */
    BackupValue();

    /* Add this entry onto the list of persistent entities. */
    Next = PersistentList;
    PersistentList = this;
    
    return Initialised;
}


template<class T>
PERSISTENT<T>::PERSISTENT(T &Value) :
    Value(Value)
{
}


template<class T>
void PERSISTENT<T>::BackupValue()
{
    OldValue = Value;
}


template<class T>
bool PERSISTENT<T>::ValueChanged()
{
    return Value != OldValue;
}



/* Instance specific implementations. */

bool PERSISTENT<int>::WriteValue(FILE * Output)
{
    return fprintf(Output, "%d", Value) > 0;
}


bool PERSISTENT<int>::ReadValue(const char * String)
{
    char * end;
    int NewValue = strtol(String, &end, 10);
    if (String < end  &&  *end == '\0')
    {
        /* Good, a proper value. */
        Value = NewValue;
        return true;
    }
    else
        return false;
}

bool PERSISTENT<bool>::WriteValue(FILE * Output)
{
    return fprintf(Output, "%s", Value ? "yes" : "no") > 0;
}


bool PERSISTENT<bool>::ReadValue(const char * String)
{
    if (strcmp(String, "no") == 0)
        Value = false;
    else if (strcmp(String, "yes") == 0)
        Value = true;
    else
        return false;
    return true;
}

#if 0
/* Note that a double *cannot* be safely treated as an atomic value for
 * threaded operations.  This means that the code below isn't sound: we need
 * an atomic way to read Value.
 *
 * Fortunately we don't actually use persistent doubles at present. */
bool PERSISTENT<double>::WriteValue(FILE * Output)
{
    return fprintf(Output, "%g", Value) > 0;
}


bool PERSISTENT<double>::ReadValue(const char * String)
{
    char * end;
    double NewValue = strtod(String, &end);
    if (String < end  &&  *end == '\0')
    {
        Value = NewValue;
        return true;
    }
    else
        return false;
}
#endif


template PERSISTENT<int>;
template PERSISTENT<bool>;
//template PERSISTENT<double>;


/* We write the state file in a background time thread.  This has advantages
 * and disadvantages.  The advantage is that the state will be written out
 * within 10 seconds of being changed, so if the IOC crashes any updates to
 * persistent state are not lost.  The disadvantage is that we have to worry
 * about synchronisation issues. */

class TIMER_THREAD: public THREAD
{
private:
    void Thread()
    {
        pid = getpid();
        StartupOk();
        while (Running())
        {
            sleep(10);
            WriteStateFile();
        }
    }

    void OnTerminate()
    {
        /* Poke the terminate thread to kick it out of its sleep. */
        TEST_(kill, pid, SIGINT);
    }
    
    int pid;
};


static TIMER_THREAD * Timer = NULL;


bool InitialisePersistentState(const char * FileName)
{
    StateFileName = FileName;
    if (StateFileName == NULL)
        return true;
    else
    {
        Timer = new TIMER_THREAD;
        return Timer->StartThread();
    }
}


void TerminatePersistentState()
{
    if (Timer != NULL)
        Timer->Terminate();
}
