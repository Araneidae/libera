/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2010  Michael Abbott, Diamond Light Source Ltd.
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

/* Shared support for X/Y statistics. */

class WAVEFORM_TUNE
{
public:
    WAVEFORM_TUNE(XYQS_WAVEFORMS &Waveform, int Field, const char *Axis);

    void Update();

private:
    WAVEFORM_TUNE();
    
    const char * PvName(const char *Pv);
    

    XYQS_WAVEFORMS &Waveform;
    const int Field;
    const char *const Axis;

    int Frequency;
    int I, Q, Mag, Phase;
};



class WAVEFORM_STATS
{
public:
    WAVEFORM_STATS(XYQS_WAVEFORMS &Waveform, int Field, const char *Axis);

    void Update();
    
private:
    WAVEFORM_STATS();
    
    XYQS_WAVEFORMS &Waveform;
    const int Field;
    const int WaveformLength;

    int Mean, Std, Min, Max, Pp;
};
