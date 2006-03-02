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
template<class T> class SIMPLE_WAVEFORM : public I_WAVEFORM
{
public:
    SIMPLE_WAVEFORM(int TypeMark, size_t WaveformSize);

    inline T * Array() { return Waveform; }

private:
    size_t read(void *array, size_t length);
    const size_t WaveformLength;
    T * Waveform;
};

class INT_WAVEFORM : public SIMPLE_WAVEFORM<int>
{
public:
    INT_WAVEFORM(size_t WaveformSize);
};

class FLOAT_WAVEFORM : public SIMPLE_WAVEFORM<float>
{
public:
    FLOAT_WAVEFORM(size_t WaveformSize);
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
    INT_WAVEFORM * RawWaveforms[4];
    INT_WAVEFORM * Waveforms[4];
};



template<class T> class WAVEFORMS
{
public:
    WAVEFORMS(size_t WaveformLength);

    /* Publishes all of the fields associated with this waveform to EPICS
     * using the given prefix. */
    void Publish(const char * Prefix);
    
    /* This changes the active length of the waveform: all other operations
     * will then operate only on the initial segment of length NewLength. */
    void SetLength(size_t NewLength);
    
    /* Interrogate the set length of this waveform: this is the desired
     * length as set through the EPICS interface. */
    size_t GetLength() { return CurrentLength; }

    /* Interrogate the working length: this is the number of rows
     * successfully captured. */
    size_t WorkingLength() { return ActiveLength; }
    /* A helper routine for the working length: returns the length of
     * waveform that actually fits at the requested offset, truncating
     * Length as appropriate so that Offset+Length<=ActiveLength. */
    size_t CaptureLength(size_t Offset, size_t Length);

    /* Reads a column from the block of waveforms, returns the number of
     * points actually read.  The Field must be the offset of the selected
     * field into the datatype T. */
    size_t Read(size_t Field, int * Target, size_t Length);

    /* Overwrites a single column in the waveform, setting the active length
     * to the number of points written. */
    void Write(size_t Field, const int * Source, size_t Length);
    

protected:
    void PublishColumn(
        const char * Prefix, const char * Name, size_t Field);

    
    /* The maximum waveform size: space actually allocated. */
    const size_t WaveformSize;
    /* The requested current working length. */
    size_t CurrentLength;
    /* The length as actually captured by the most recent Capture[From].
     * This determines how much data is returned elsewhere. */
    size_t ActiveLength;
    /* The waveform itself. */
    T * const Data;

    /* Some tiresome problems with C++ access management.  Anything that
     * looks across instances needs special helper declarations here. */
    friend class ABCD_WAVEFORMS;
    friend class XYQS_WAVEFORMS;
};


class IQ_WAVEFORMS : public WAVEFORMS<IQ_ROW>
{
public:
    IQ_WAVEFORMS(size_t Length) : WAVEFORMS<IQ_ROW>(Length) { }
    /* Capture the currently selected active length of waveform from the data
     * source. */
    void Capture(int Decimation);
    /* Capture the postmortem buffer. */
    void CapturePostmortem();
    /* Capture a copy of the selected waveform. */
    void CaptureFrom(IQ_WAVEFORMS & Source, size_t Offset);
};

class ABCD_WAVEFORMS : public WAVEFORMS<ABCD_ROW>
{
public:
    ABCD_WAVEFORMS(size_t Length) : WAVEFORMS<ABCD_ROW>(Length) { }
    
    /* Capture button values from given IQ waveform. */
    void CaptureCordic(IQ_WAVEFORMS & Source);
};

class XYQS_WAVEFORMS : public WAVEFORMS<XYQS_ROW>
{
public:
    XYQS_WAVEFORMS(size_t Length) : WAVEFORMS<XYQS_ROW>(Length) { }

    /* Publish short form names. */
    void PublishShort(const char * Prefix);
    /* Capture positions from button values. */
    void CaptureConvert(ABCD_WAVEFORMS &Source);
};


/* Misplaced publish routines. */
void Publish_ABCD(const char * Prefix, ABCD_ROW &ABCD);
void Publish_XYQS(const char * Prefix, XYQSmm_ROW &XYQS);
