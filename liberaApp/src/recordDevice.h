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


/* Record type definitions for the records used by Libera. */

/* It seems mad that I have to declare the structures here.  Why on earth
 * aren't they declared somewhere in an EPICS header?!  Unfortunately it seems
 * that the only place these structures are defined is in the corresponding
 * base/rec/<type>Record.c implementation file.  Grrr. */

#define COMMON_FIELDS(type) \
    long number; \
    long (*dev_report)(int); \
    long (*init)(int); \
    long (*init_record)(type##Record *); \
    long (*get_ioint_info)(int, dbCommon *, IOSCANPVT *)

struct longinDevice {
    COMMON_FIELDS(longin);
    long (*read_longin)(longinRecord *);
};

struct longoutDevice {
    COMMON_FIELDS(longout);
    long (*write_longout)(longoutRecord *);
};

struct aiDevice {
    COMMON_FIELDS(ai);
    long (*read_ai)(aiRecord *);
    long (*special_linconv)(aiRecord *, int);
};

struct aoDevice {
    COMMON_FIELDS(ao);
    long (*write_ao)(aoRecord *);
    long (*special_linconv)(aoRecord *, int);
};

struct biDevice {
    COMMON_FIELDS(bi);
    long (*read_bi)(biRecord *);
};

struct boDevice {
    COMMON_FIELDS(bo);
    long (*write_bo)(boRecord *);
};

struct stringinDevice {
    COMMON_FIELDS(stringin);
    long (*read_stringin)(stringinRecord *);
};

struct stringoutDevice {
    COMMON_FIELDS(stringout);
    long (*write_stringout)(stringoutRecord *);
};

struct waveformDevice {
    COMMON_FIELDS(waveform);
    long (*read_waveform)(waveformRecord *);
};

struct subArrayDevice {
    COMMON_FIELDS(subArray);
    long (*read_subArray)(subArrayRecord *);
};

struct mbbiDevice {
    COMMON_FIELDS(mbbi);
    long (*read_mbbi)(mbbiRecord *);
};

struct mbboDevice {
    COMMON_FIELDS(mbbo);
    long (*write_mbbo)(mbboRecord *);
};

#undef COMMON_FIELDS
