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

#include <dbFldTypes.h>

#include "drivers.h"
#include "hardware.h"
#include "convert.h"
#include "cordic.h"

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


FLOAT_WAVEFORM::FLOAT_WAVEFORM(size_t WaveformLength) :
    I_WAVEFORM(DBF_FLOAT),
    WaveformLength(WaveformLength),
    Waveform(new float[WaveformLength])
{
}


size_t FLOAT_WAVEFORM::read(void *array, size_t length)
{
    if (length > WaveformLength)  length = WaveformLength;
    memcpy(array, Waveform, sizeof(float) * length);
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
    Data(new LIBERA_ROW[WaveformSize])
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

    /* Copy over the area of interest. */
    memcpy(Data, &Waveform.Data[Offset], ActiveLength * sizeof(LIBERA_ROW));
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
        Target[i] = Data[Offset + i][Index];
    return Length;
}


I_waveform & LIBERA_WAVEFORM::Waveform(int Index)
{
    return * new READ_WAVEFORM(*this, Index);
}


void LIBERA_WAVEFORM::Cordic()
{
    SinCosToABCD(Data, ActiveLength);
}


void LIBERA_WAVEFORM::ABCDtoXYQS()
{
    ::ABCDtoXYQS(Data, ActiveLength);
}



/****************************************************************************/
/*                                                                          */
/*                            class ADC_WAVEFORM                            */
/*                                                                          */
/****************************************************************************/

/* Access to raw and unfiltered sample rate (117MHz) data as captured by the
 * ADC. */


ADC_WAVEFORM::ADC_WAVEFORM()
{
    /* Initialise waveforms for all four buttons.  We maintain two waveforms
     * for each button: the raw 1024 samples as read from the ADC, and the
     * same data frequency shifted and resampled down to 256 points. */
    for (int i = 0; i < 4; i ++)
    {
        RawWaveforms[i] = new SIMPLE_WAVEFORM(ADC_LENGTH);
        Waveforms[i]    = new SIMPLE_WAVEFORM(ADC_LENGTH/4);
    }
}


/* Read a waveform from Libera.   The raw ADC data is read, sign extended
 * from 12 to 32 bits, and transposed into the four raw waveforms.
 *
 * The next stage of processing takes advantage of a couple of important
 * features of the data being sampled.  The input signal is RF (at
 * approximately 500MHz) and is undersampled (at approximately 117Mhz) so that
 * the centre frequency appears at close to 1/4 the sampling frequency.  To
 * make this possible, the signal is filtered through a narrow band
 * (approximately 10MHz bandwith) filter.
 *
 * Thus the intensity profile of the incoming train can be recovered by the
 * following steps:
 *  - mix with the centre frequency (producing a complex IQ waveform) to
 *    bring the carrier frequency close to DC
 *  - low pass filter the data
 *  - compute the absolute magnitude of the waveform.
 *
 * Furthermore, because the carrier frequency is so close to 1/4 sampling
 * frequency, mixing becomes a matter of multiplying successively by
 * exp(2*pi*i*n), in other words, by the sequence
 *      1,  i,  -1,  -i,  1,  ...
 * and if we then low pass filter by averaging four points together before
 * computing the magnitude, we can reduce the data stream
 *      x1, x2, x3, x4, x5, ...
 * to the stream
 *      |(x1-x3,x2-x4)|, |(x5-x7,x6-x8)|, ...
 */

bool ADC_WAVEFORM::Capture()
{
    ADC_DATA RawData;
    bool Ok = ReadAdcWaveform(RawData);
    if (Ok)
    {
        int * a[4];
        for (int j = 0; j < 4; j ++)
            a[j] = RawWaveforms[j]->Array();
        
        /* First sign extend each waveform point, extend to 32-bits and
         * transpose to a more useful orientation. */
        for (int i = 0; i < ADC_LENGTH; i ++)
            for (int j = 0; j < 4; j ++)
                a[j][i] = ((int) RawData.Rows[i][j] << 20) >> 20;
        
        /* Now reduce each set of four points down to one point.  This
         * removes the carrier frequency and recovers the underlying
         * intensity profile of each waveform. */
        for (int j = 0; j < 4; j ++)
        {
            int * p = RawWaveforms[j]->Array();
            int * q = Waveforms[j]->Array();
            for (int i = 0; i < ADC_LENGTH; i += 4)
            {
                int x1 = *p++, x2 = *p++, x3 = *p++, x4 = *p++;
                /* Scale the raw values so that they're compatible in
                 * magnitude with turn-by-turn filtered values.  This means
                 * that we can use the same scaling rules downstream.
                 *    A raw ADC value is +-2^11, and by combining pairs we
                 * get +-2^12.  Scaling by 2^18 gives us values in the range
                 * +-2^30, which will be comfortable.  After cordic this
                 * becomes 0..sqrt(2)*0.5822*2^30 or 0.823*2^30. */
                *q++ = CordicMagnitude((x1-x3)<<18, (x2-x4)<<18);
            }
        }
    }
    return Ok;
}
