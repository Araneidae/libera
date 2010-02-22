/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2005-2009  Michael Abbott, Diamond Light Source Ltd.
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
template<class T>
class SIMPLE_WAVEFORM : public I_WAVEFORM
{
public:
    SIMPLE_WAVEFORM(
        int TypeMark, size_t EpicsPointSize, size_t WaveformSize,
        T * ExternalWaveform = NULL);

    inline T * Array() { return Waveform; }

private:
    bool process(void *array, size_t max_length, size_t &new_length);
    const size_t EpicsPointSize;
    const size_t WaveformLength;
    T * const Waveform;
};

class INT_WAVEFORM : public SIMPLE_WAVEFORM<int>
{
public:
    INT_WAVEFORM(size_t WaveformSize, int * ExternalWaveform = NULL);
};

class FLOAT_WAVEFORM : public SIMPLE_WAVEFORM<float>
{
public:
    FLOAT_WAVEFORM(size_t WaveformSize, float * ExternalWaveform = NULL);
};

class UCHAR_WAVEFORM : public SIMPLE_WAVEFORM<unsigned char>
{
public:
    UCHAR_WAVEFORM(
        size_t WaveformSize, unsigned char * ExternalWaveform = NULL);
};

typedef INT_WAVEFORM        int_WAVEFORM;
typedef FLOAT_WAVEFORM      float_WAVEFORM;

#ifdef I
class COMPLEX_WAVEFORM : public SIMPLE_WAVEFORM<complex>
{
public:
    COMPLEX_WAVEFORM(size_t WaveformSize, complex * ExternalWaveform = NULL);
};
typedef COMPLEX_WAVEFORM    complex_WAVEFORM;
#endif



/* Wrappers for automatically building and publishing simple waveforms on top
 * of existing waveforms.  Note that Waveform must know its own size: if it's
 * a mere pointer, an explicit sizing cast is going to be needed! */
#define PublishSimpleWaveform(type, Name, Waveform) \
    ( { \
        type##_WAVEFORM &Wrapper = * new type##_WAVEFORM( \
            sizeof(Waveform) / sizeof(type), \
            (type *) Waveform); \
        Publish_waveform(Name, Wrapper); \
        Wrapper; \
    } )
            



/* Generic waveform set class.  This class is used to gather together several
 * waveforms into a single structure.  The following instances of this
 * template are defined:
 *      IQ_WAVEFORMS    Used for raw IQ data as read from Libera
 *      ABCD_WAVEFORMS  Used for button values, reduced from IQ via cordic
 *      XYQS_WAVEFORMS  Used for computed electron beam positions.
 */

template<class T>
class WAVEFORMS
{
public:
    /* Defines the maximum number of T rows in the waveform.  If FullSize is
     * set then the active length of the waveform is set to the full
     * waveform (the waveform is effectively assumed to start containing
     * data); if clear then the waveform is initialised with active length of
     * zero (no data actually in waveform). */
    WAVEFORMS(size_t WaveformLength, bool FullSize=false);

    /* Publishes all of the fields associated with this waveform to EPICS
     * using the given prefix. */
    void Publish(const char * Prefix, const char *SubName="WF") const;
    
    /* This changes the active length of the waveform: all other operations
     * will then operate only on the initial segment of length NewLength. */
    void SetLength(size_t NewLength);
    
    /* Interrogate the set length of this waveform: this is the desired
     * length as set through the EPICS interface. */
    size_t GetLength() const { return CurrentLength; }

    /* Interrogate the working length: this is the number of rows
     * successfully captured. */
    size_t WorkingLength() const { return ActiveLength; }
    /* A helper routine for the working length: returns the length of
     * waveform that actually fits at the requested offset, truncating
     * Length as appropriate so that Offset+Length<=ActiveLength. */
    size_t CaptureLength(size_t Offset, size_t Length) const;
    /* Returns the underlying maximum waveform size. */
    size_t MaxLength() const { return WaveformSize; };

    /* Reads a column from the block of waveforms, returns the number of
     * points actually read.  The Field must be the offset of the selected
     * field into the datatype T. */
    size_t Read(size_t Field, int * Target, size_t Length) const;

    /* Overwrites a single column in the waveform, setting the active length
     * to the number of points written. */
    void Write(size_t Field, const int * Source, size_t Length);
    
    /* Capture a waveform by copying from an existing instance of the same
     * waveform. */
    void CaptureFrom(const WAVEFORMS<T> & Source, size_t Offset);

    /* Reads the timestamp. */
    const LIBERA_TIMESTAMP & GetTimestamp() { return Timestamp; }

    /* Direct access to the raw data.  No more than GetLength() rows should
     * be written to this waveform, and no more than WorkingLength() can
     * sensibly be read. */
    T * Waveform() const { return Data; }

    
protected:
    void PublishColumn(
        const char * Prefix, const char * Name, size_t Field) const;


    /* The following invariant relates the three sizes below at all times:
     *
     *      0 <= ActiveLength <= CurrentLength <= WaveformSize
     *
     * CurrentLength records how long a waveform we will try to capture,
     * while active length records how much has successfully been captured. */
    
    /* The maximum waveform size: space actually allocated. */
    const size_t WaveformSize;
    /* The requested current working length. */
    size_t CurrentLength;
    /* The length as actually captured by the most recent Capture[From].
     * This determines how much data is returned elsewhere. */
    size_t ActiveLength;
    /* The waveform itself. */
    T * const Data;
    /* The timestamp of the waveform. */
    LIBERA_TIMESTAMP Timestamp;

    /* Some tiresome problems with C++ access management.  Anything that
     * looks across instances needs special helper declarations here. */
    friend class ABCD_WAVEFORMS;
    friend class XYQS_WAVEFORMS;
};


class IQ_WAVEFORMS : public WAVEFORMS<IQ_ROW>
{
public:
    IQ_WAVEFORMS(size_t Length, bool FullSize=false) :
        WAVEFORMS<IQ_ROW>(Length, FullSize) { }
    
    /* Capture the currently selected active length of waveform from the data
     * source.  Possible decimations are 1 or 64, as determined by the FPGA. */
    void Capture(int Decimation = 1, int Offset = 0);
    /* Capture the postmortem buffer. */
    void CapturePostmortem();
};

class ABCD_WAVEFORMS : public WAVEFORMS<ABCD_ROW>
{
public:
    ABCD_WAVEFORMS(size_t Length, bool FullSize=false) :
        WAVEFORMS<ABCD_ROW>(Length, FullSize) { }

    /* Capture button values from given IQ waveform. */
    void CaptureCordic(const IQ_WAVEFORMS & Source);
    /* Special case for naming RAW waveforms. */
    void PublishRaw(const char * Prefix) const;
};

class XYQS_WAVEFORMS : public WAVEFORMS<XYQS_ROW>
{
public:
    XYQS_WAVEFORMS(size_t Length, bool FullSize=false) :
        WAVEFORMS<XYQS_ROW>(Length, FullSize) { }

    /* Capture positions from button values. */
    void CaptureConvert(const ABCD_WAVEFORMS &Source);
};


/* Slightly misplaced publish routines. */
void Publish_ABCD(const char * Prefix, ABCD_ROW &ABCD);
void Publish_XYQS(const char * Prefix, XYQS_ROW &XYQS, const char * Suffix="");
