// $Id: ebpp.h,v 1.10 2006/01/06 22:45:13 ales Exp $

//! \file ebpp.h
//! Electron Beam Position Processor specific definitions.

#if !defined(_EBPP_H)
#define _EBPP_H

/** Libera EBPP Slow Acquisition (SA) sample. */
typedef struct {
	// Amplitudes
	int Va, Vb, Vc, Vd;
	// Sum Va + Vb + Vc + Vd
	int Sum;
	// Quadropole signal
	int Q;
	// Horizontal beam position
	int X;
	// Vertical beam position
	int Y;
	// Horiz. and vert. correction factors from the FA Application
	int Cx, Cy;
	// 6 values reserved for future use
	int reserved[6];
}
CSPI_SA_ATOM;

/** Libera EBPP Data on Demand (DD) raw sample. */
typedef struct {
	int cosVa, sinVa;
	int cosVb, sinVb;
	int cosVc, sinVc;
	int cosVd, sinVd;
}
CSPI_DD_RAWATOM;

/** Libera EBPP Data on Demand (DD) sample. */
typedef struct {
	// Amplitudes
	int Va, Vb, Vc, Vd;
	// Horiz. and vert. beam position
	int X, Y;
	// Quadropole signal
	int Q;
	// Sum Va + Vb + Vc + Vd
	int Sum;
}
CSPI_DD_ATOM;

typedef struct {
    short chD;
    short chC;
    short chB;
    short chA;
}
CSPI_ADC_ATOM;

//--------------------------------------------------------------------------

/** Private. Total number of attenuators. */
#define CSPI_MAXATTN 8

/** Environment parameters or attributes. */
struct tagCSPI_ENVPARAMS
{
	CSPI_ENVPARAMS_BASE;

	int Kx, Ky;
	int Xoffset, Yoffset, Qoffset;
	int Xlow, Xhigh, Ylow, Yhigh;
	int switches;
	int attn[CSPI_MAXATTN];
};

typedef enum
{
	CSPI_ENV_KX			= CUSTOM_ENV_BIT(0),
	CSPI_ENV_KY			= CUSTOM_ENV_BIT(1),
	CSPI_ENV_XOFFSET	= CUSTOM_ENV_BIT(2),
	CSPI_ENV_YOFFSET	= CUSTOM_ENV_BIT(3),
	CSPI_ENV_QOFFSET	= CUSTOM_ENV_BIT(4),
	CSPI_ENV_XLOW		= CUSTOM_ENV_BIT(5),
	CSPI_ENV_XHIGH		= CUSTOM_ENV_BIT(6),
	CSPI_ENV_YLOW		= CUSTOM_ENV_BIT(7),
	CSPI_ENV_YHIGH		= CUSTOM_ENV_BIT(8),
	CSPI_ENV_SWITCH		= CUSTOM_ENV_BIT(9),
	CSPI_ENV_ATTN		= CUSTOM_ENV_BIT(10),
}
CSPI_ENVFLAGS_EBPP;

//--------------------------------------------------------------------------

/** Derived from CSPI_CONPARAMS to handle DD specific
 *  parameters or attributes.
 */
typedef struct {
	/** Common connection parameters. */
	CSPI_CONPARAMS_BASE;

	/** Decimation factor. */
	size_t dec;
}
CSPI_CONPARAMS_DD;

//--------------------------------------------------------------------------

/** Bit flags corresponding to the CSPI_CONPARAMS_DD structure.
 *  See CSPI_CONPARAMS_DD structure for descriptions.
 */
typedef enum {
	CSPI_CON_DEC = CUSTOM_CON_BIT(0),
}
CSPI_CONFLAGS_DD;

#endif	// _EBPP_H
