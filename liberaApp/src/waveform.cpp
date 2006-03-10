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
#include <stddef.h>

#include <dbFldTypes.h>

#include "drivers.h"
#include "hardware.h"
#include "convert.h"
#include "cordic.h"
#include "publish.h"

#include "waveform.h"




/****************************************************************************/
/*                                                                          */
/*                       Simple I_waveform classes                          */
/*                                                                          */
/****************************************************************************/


/* A simple one dimensional waveform with direct EPICS implementation.
 * Arrays of both ints and floats are supported, so we unify these two types
 * into a single template. */

template<class T>
SIMPLE_WAVEFORM<T>::SIMPLE_WAVEFORM(int TypeMark, size_t WaveformLength) :
    I_WAVEFORM(TypeMark),
    WaveformLength(WaveformLength),
    Waveform(new T[WaveformLength])
{
}


template<class T>
size_t SIMPLE_WAVEFORM<T>::read(void *array, size_t length)
{
    if (length > WaveformLength)  length = WaveformLength;
    memcpy(array, Waveform, sizeof(T) * length);
    return length;
}

INT_WAVEFORM::INT_WAVEFORM(size_t WaveformSize) :
    SIMPLE_WAVEFORM<int>(DBF_LONG, WaveformSize) { }

FLOAT_WAVEFORM::FLOAT_WAVEFORM(size_t WaveformSize) :
    SIMPLE_WAVEFORM<float>(DBF_FLOAT, WaveformSize) { }


template SIMPLE_WAVEFORM<int>;
template SIMPLE_WAVEFORM<float>;




/****************************************************************************/
/*                                                                          */
/*                         Generic Waveform Support                         */
/*                                                                          */
/****************************************************************************/


/* Implements EPICS access to a single column of a waveform.  Works by
 * remembering the waveforms block and which column is required and then
 * simply wraps the Read() method into an I_waveform read() method. */

template<class T>
class COLUMN_WAVEFORM : public I_WAVEFORM
{
public:
    COLUMN_WAVEFORM(const WAVEFORMS<T> & Waveforms, size_t Field) :
        I_WAVEFORM(DBF_LONG),
        Waveforms(Waveforms),
        Field(Field)
    {
    }

    size_t read(void *Array, size_t Length)
    {
        return Waveforms.Read(Field, (int*) Array, Length);
    }
    
private:
    const WAVEFORMS<T> & Waveforms;
    const size_t Field;
};



/* A WAVEFORMS class defines a block of row oriented waveforms: these are
 * typically processed row-by-row but read out in columns.  We support three
 * different types of row: raw IQ data (button data in quadrature), button
 * ABCD intensity values and decoded XYQS positions.
 *
 * Each waveforms block also maintains a desired length (which may be less
 * than the maximum waveform size, for example when capturing long
 * turn-by-turn waveforms) and also an "active" working length which records
 * how many points have actually been capture into this block. */


template<class T>
WAVEFORMS<T>::WAVEFORMS(size_t WaveformSize) :
    WaveformSize(WaveformSize),
    Data(new T[WaveformSize])
{
    CurrentLength = WaveformSize;
    ActiveLength = 0;
}


template<class T>
void WAVEFORMS<T>::SetLength(size_t NewLength)
{
    /* First ensure that the requested length is no longer than we actually
     * have room for. */
    if (NewLength > WaveformSize)
        NewLength = WaveformSize;
    CurrentLength = NewLength;
    /* Also truncate the active length to track the requested length. */
    if (ActiveLength > CurrentLength)
        ActiveLength = CurrentLength;
}


template<class T>
size_t WAVEFORMS<T>::Read(size_t Field, int * Target, size_t Length) const
{
    /* Adjust the length we'll return according to how much data we actually
     * have in hand. */
    Length = CaptureLength(0, Length);
    char * Source = ((char *) Data) + Field;
    for (size_t i = 0; i < Length; i ++)
    {
        Target[i] = *(int *) Source;
        Source += sizeof(T);
    }
    return Length;
}


template<class T>
void WAVEFORMS<T>::Write(size_t Field, const int * Source, size_t Length)
{
    /* Make sure we don't try to write more than we have room for. */
    if (Length > CurrentLength)
        Length = CurrentLength;
        
    char * Target = ((char *) Data) + Field;
    for (size_t i = 0; i < Length; i ++)
    {
        *(int*)Target = Source[i];
        Target += sizeof(T);
    }
    ActiveLength = Length;
}


template<class T>
size_t WAVEFORMS<T>::CaptureLength(size_t Offset, size_t Length) const
{
    /* Use as much of the other waveform as we can fit into our currently
     * selected length, also taking into account our desired offset into the
     * source. */
    if (Offset >= ActiveLength)
        return 0;
    else
    {
        if (Offset + Length > ActiveLength)
            Length = ActiveLength - Offset;
        return Length;
    }
}

template<class T>
void WAVEFORMS<T>::CaptureFrom(const WAVEFORMS<T> & Source, size_t Offset)
{
    ActiveLength = Source.CaptureLength(Offset, CurrentLength);
    memcpy(Data, Source.Data + Offset, ActiveLength * sizeof(*Data));
}


/* Helper routine for publishing a column of the waveforms block to EPICS.
 * Uses the COLUMN_WAVEFORM class to build the appropriate access method.
 * Works closely with the two macros below. */

template<class T>
void WAVEFORMS<T>::PublishColumn(
    const char * Prefix, const char * Name, size_t Field) const
{
    Publish_waveform(
        Concat(Prefix, Name),
        *new COLUMN_WAVEFORM<T>(*this, Field));
}

/* These two macros work together to publish a set of names in the form
 *      <Prefix>:<SubName><Name>
 * Note that these macros relie on PublishColumn taking a copy of the
 * PrefixString! */
#define PREPARE_PUBLISH(Prefix, SubName) \
    char PrefixString[100]; \
    snprintf(PrefixString, sizeof(PrefixString), "%s:%s", Prefix, SubName)

#define PUBLISH_COLUMN(Name, Field) \
    PublishColumn(PrefixString, Name, offsetof(typeof(*Data), Field))



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                           IQ Waveform Support                             */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void WAVEFORMS<IQ_ROW>::Publish(const char * Prefix, const char *SubName) const
{
    PREPARE_PUBLISH(Prefix, SubName);
    PUBLISH_COLUMN("AI", AI);
    PUBLISH_COLUMN("AQ", AQ);
    PUBLISH_COLUMN("BI", BI);
    PUBLISH_COLUMN("BQ", BQ);
    PUBLISH_COLUMN("CI", CI);
    PUBLISH_COLUMN("CQ", CQ);
    PUBLISH_COLUMN("DI", DI);
    PUBLISH_COLUMN("DQ", DQ);
}


void IQ_WAVEFORMS::Capture(int Decimation)
{
    ActiveLength = ReadWaveform(
        Decimation, CurrentLength, (LIBERA_ROW *) Data);
}

void IQ_WAVEFORMS::CapturePostmortem()
{
    ActiveLength = ReadPostmortem(CurrentLength, (LIBERA_ROW *) Data);
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                          ABCD Waveform Support                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void WAVEFORMS<ABCD_ROW>::Publish(
    const char * Prefix, const char *SubName) const
{
    PREPARE_PUBLISH(Prefix, SubName);
    PUBLISH_COLUMN("A", A);
    PUBLISH_COLUMN("B", B);
    PUBLISH_COLUMN("C", C);
    PUBLISH_COLUMN("D", D);
}

void ABCD_WAVEFORMS::CaptureCordic(const IQ_WAVEFORMS & Source)
{
    ActiveLength = Source.CaptureLength(0, CurrentLength);
    IQtoABCD(Source.Data, Data, ActiveLength);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                          XYQS Waveform Support                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void WAVEFORMS<XYQS_ROW>::Publish(
    const char * Prefix, const char *SubName) const
{
    PREPARE_PUBLISH(Prefix, SubName);
    PUBLISH_COLUMN("X", X);
    PUBLISH_COLUMN("Y", Y);
    PUBLISH_COLUMN("Q", Q);
    PUBLISH_COLUMN("S", S);
}

void XYQS_WAVEFORMS::CaptureConvert(const ABCD_WAVEFORMS &Source)
{
    ActiveLength = Source.CaptureLength(0, CurrentLength);
    ABCDtoXYQS(Source.Data, Data, ActiveLength);
}



/* Ensure that instances of all the templates we've just talked about
 * actually exist.  This approach also means that we don't have to copy all
 * the template definitions into the header file, which is good. */
template class WAVEFORMS<IQ_ROW>;
template class WAVEFORMS<ABCD_ROW>;
template class WAVEFORMS<XYQS_ROW>;




/****************************************************************************/
/*                                                                          */
/*                         Single Row Publishing                            */
/*                                                                          */
/****************************************************************************/

/* Slightly out of place here... */

void Publish_ABCD(const char * Prefix, ABCD_ROW &ABCD)
{
    Publish_longin(Concat(Prefix, ":A"), ABCD.A);
    Publish_longin(Concat(Prefix, ":B"), ABCD.B);
    Publish_longin(Concat(Prefix, ":C"), ABCD.C);
    Publish_longin(Concat(Prefix, ":D"), ABCD.D);
}

void Publish_XYQS(const char * Prefix, XYQSmm_ROW &XYQS)
{
    Publish_ai    (Concat(Prefix, ":X"), XYQS.X);
    Publish_ai    (Concat(Prefix, ":Y"), XYQS.Y);
    Publish_ai    (Concat(Prefix, ":Q"), XYQS.Q);
    Publish_longin(Concat(Prefix, ":S"), XYQS.S);
}
