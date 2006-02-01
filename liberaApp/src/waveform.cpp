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


/* Waveform processing support.  Collects together common waveform processing
 * support. */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <db_access.h>

#include "drivers.h"
#include "hardware.h"
#include "convert.h"

#include "waveform.h"





/****************************************************************************/
/*                                                                          */
/*                       Simple I_waveform classes                          */
/*                                                                          */
/****************************************************************************/

/* A couple of helper classes implementing I_waveform interfaces. */


/* This implements a simple one dimensional waveform with a direct EPICS
 * implementation. */

SIMPLE_WAVEFORM::SIMPLE_WAVEFORM(size_t WaveformLength) :
    I_WAVEFORM(DBF_LONG),
    WaveformLength(WaveformLength),
    Waveform(new int[WaveformLength])
{
}


size_t SIMPLE_WAVEFORM::read(void *array, size_t length)
{
    if (length > WaveformLength)  length = WaveformLength;
    memcpy(array, Waveform, sizeof(int) * length);
    return length;
}



/* Implements EPICS access to a single column of a LIBERA_WAVEFORM. */

class READ_WAVEFORM : public I_WAVEFORM
{
public:
    READ_WAVEFORM(LIBERA_WAVEFORM & Waveform, int Index) :
        I_WAVEFORM(DBF_LONG),
        Waveform(Waveform),
        Index(Index)
    {
    }

    size_t read(void *array, size_t Length)
    {
        return Waveform.Read(Index, (int *) array, 0, Length);
    }
    

private:
    LIBERA_WAVEFORM & Waveform;
    const int Index;
};



/****************************************************************************/
/*                                                                          */
/*                          class LIBERA_WAVEFORM                           */
/*                                                                          */
/****************************************************************************/



LIBERA_WAVEFORM::LIBERA_WAVEFORM(size_t WaveformSize) :
    WaveformSize(WaveformSize),
    Data(*(LIBERA_DATA *) new char[LIBERA_DATA_SIZE(WaveformSize)])
{
    CurrentLength = WaveformSize;
    ActiveLength = 0;
}


void LIBERA_WAVEFORM::SetLength(size_t NewLength)
{
    if (NewLength > WaveformSize)  NewLength = WaveformSize;
    CurrentLength = NewLength;
}


void LIBERA_WAVEFORM::Capture(int Decimation)
{
    ActiveLength = ReadWaveform(Decimation, CurrentLength, Data);
    if (ActiveLength > CurrentLength)
    {   // We can lose this test soon enough!  Maybe need some asserts...
        printf("Whaa?!\n");     
        ActiveLength = CurrentLength;
    }
}


void LIBERA_WAVEFORM::CaptureFrom(LIBERA_WAVEFORM & Waveform, size_t Offset)
{
    /* Use as much of the other waveform as we can fit into our currently
     * selected length, also taking into account our desired offset into the
     * source. */
    if (Offset > Waveform.ActiveLength)
        Offset = Waveform.ActiveLength;
    ActiveLength = Waveform.ActiveLength - Offset;
    if (ActiveLength > CurrentLength)
        ActiveLength = CurrentLength;

    /* Copy the timestamp and our data area of interest: these are
     * necessarily separate operations, as Offset may be non zero. */
    Data.Timestamp = Waveform.Data.Timestamp;
    memcpy(&Data.Rows, &Waveform.Data.Rows[Offset],
        ActiveLength * sizeof(LIBERA_ROW));
}


size_t LIBERA_WAVEFORM::Read(
    int Index, int * Target, size_t Offset, size_t Length)
{
    /* Ensure we don't read beyond the waveform we have. */
    if (Offset > ActiveLength)
        Offset = ActiveLength;
    if (Offset + Length > ActiveLength)
        Length = ActiveLength - Offset;
    for (size_t i = 0; i < Length; i ++)
        Target[i] = Data.Rows[Offset + i][Index];
    return Length;
}


I_waveform & LIBERA_WAVEFORM::Waveform(int Index)
{
    return * new READ_WAVEFORM(*this, Index);
}


void LIBERA_WAVEFORM::Cordic(int Iterations)
{
    SinCosToABCD(Data.Rows, ActiveLength, Iterations);
}


void LIBERA_WAVEFORM::ABCDtoXYQS()
{
    ::ABCDtoXYQS(Data.Rows, ActiveLength);
}
