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


/* Waveform processing support classes and routines. */


/* A very simple waveform with direct EPICS support. */
class SIMPLE_WAVEFORM : public I_WAVEFORM
{
public:
    SIMPLE_WAVEFORM(size_t WaveformSize);

    inline int * Array() { return Waveform; }

private:
    size_t read(void *array, size_t length);
    const size_t WaveformLength;
    int * Waveform;
};



class FLOAT_WAVEFORM : public I_WAVEFORM
{
public:
    FLOAT_WAVEFORM(size_t WaveformSize);

    inline float * Array() { return Waveform; }
    
private:
    size_t read(void *array, size_t length);
    const size_t WaveformLength;
    float * Waveform;
};





/* Support for waveforms captured from Libera. */

class LIBERA_WAVEFORM
{
public:
    LIBERA_WAVEFORM(size_t WaveformSize);

    /* Read a waveform from Libera at the requested decimation. */
    void Capture(int Decimation=1);

    /* Read a waveform by copying data from another waveform. */
    void CaptureFrom(LIBERA_WAVEFORM & Waveform, size_t Offset);

    /* Run a pass of Cordic over the data to reduce sin/cos pairs to absolute
     * button signal values. */
    void Cordic();

    /* Compute XYQS arrays from the raw button values. */
    void ABCDtoXYQS();

    /* Extracts one column from the interanl waveform into the given target
     * array.  Index selects the column, and may be one of (assuming the
     * appropriate functions Cordic and ABCDtoXYQS above have been called):
     *          0       Button A value
     *          1       Button B value
     *          2       Button C value
     *          3       Button D value
     *          4       X position
     *          5       Y position
     *          6       Skew value Q
     *          7       Total intensity S
     * Offset selects the offset into the waveform where reading will begin
     * and Length requests the number of rows to be read. 
     *     The number of rows actually read is returned: this may be less
     * than GetLength() returns. */
    size_t Read(int Index, int * Target, size_t Offset, size_t Length);

    /* Returns an I_waveform suitable for publishing to EPICS the selected
     * column of the waveform. */
    I_waveform & Waveform(int Index);

    /* Interrogate the length of this waveform. */
    size_t GetLength() { return CurrentLength; }

    /* This changes the active length of the waveform: all other operations
     * will then operate only on the initial segment of length NewLength. */
    void SetLength(size_t NewLength);

    /* Interrogate the working length: this is the number of rows
     * successfully captured. */
    size_t WorkingLength() { return ActiveLength; }
    
    
private:
    /* The maximum waveform size: space actually allocated. */
    const size_t WaveformSize;
    /* The requested current working length. */
    size_t CurrentLength;
    /* The length as actually captured by the most recent Capture[From].
     * This determines how much data is returned elsewhere. */
    size_t ActiveLength;
    LIBERA_ROW * Data;
};


/* Support for ADC rate waveform. */

class ADC_WAVEFORM
{
public:
    ADC_WAVEFORM();

    /* Read a waveform from Libera.  The raw data (1024 sample unprocessed
     * signed 12-bit * waveforms) are stored in the four RawWaveform()
     * waveforms (one for each button).
     * 
     * At the same time the four waveforms are frequency shifted, resampled
     * and Cordic reduced to produce four 256 sample Waveform() waveforms
     * containing the intensity envelope of the incoming signal. */
    bool Capture();

    /* Publishable interfaces to the captured raw ADC rate data and a reduced
     * form of the same data. */
    I_waveform & RawWaveform(int Index) { return * RawWaveforms[Index]; }
    I_waveform & Waveform(int Index)    { return * Waveforms[Index]; }

    /* Direct access to the underlying waveform data for waveforms. */
    int * RawArray(int Index)  { return RawWaveforms[Index]->Array(); }
    int * Array(int Index)     { return Waveforms[Index]->Array(); }
    
private:
    /* We internally maintain both the original raw (1024 point) waveform and
     * a version reduced by frequency shifting and resampling (256 point),
     * for each of the four buttons. */
    SIMPLE_WAVEFORM * RawWaveforms[4];
    SIMPLE_WAVEFORM * Waveforms[4];
};
