// $Id: ebpp.h,v 1.31 2006/11/29 18:57:29 ales Exp $

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

/** DSC Compensation parameters. */
struct DSC_COMPPARAMS
{
        float ampl[16][4];
        float phase[16][4];
        int status;
};

/** Environment parameters or attributes. */
struct tagCSPI_ENVPARAMS
{
	CSPI_ENVPARAMS_BASE;

	int Kx, Ky;
	int Xoffset, Yoffset, Qoffset;

	int switches;	// Analog board switch mode. See CSPI_SWITCHMODE.
	int gain;		// Analog board gain (dBm).

	int agc;		// AGC mode. See CSPI_AGCMODE.
	int dsc;		// DSC mode. See CSPI_DSCMODE.

	// Interlock parameters.
	struct {
		// Interlock mode. See CSPI_INTERLOCKMODE.
		int mode;
		// Interlock limits.
		int Xlow, Xhigh, Ylow, Yhigh;
		// Interlock overflow limit (ADC count).
		int overflow_limit;
		// Interlock overflow duration (ADC clock periods).
		int overflow_dur;
		// Gain limit (dBm) for gain-dependant interlock.
		int gain_limit;
	} ilk;

};

/** Libera EBPP specific environment bitflags. */
typedef enum
{
	CSPI_ENV_KX			= CUSTOM_ENV_BIT(0),
	CSPI_ENV_KY			= CUSTOM_ENV_BIT(1),
	CSPI_ENV_XOFFSET	= CUSTOM_ENV_BIT(2),
	CSPI_ENV_YOFFSET	= CUSTOM_ENV_BIT(3),
	CSPI_ENV_QOFFSET	= CUSTOM_ENV_BIT(4),
	CSPI_ENV_SWITCH		= CUSTOM_ENV_BIT(5),
	CSPI_ENV_GAIN		= CUSTOM_ENV_BIT(6),
	CSPI_ENV_AGC		= CUSTOM_ENV_BIT(7),
	CSPI_ENV_DSC		= CUSTOM_ENV_BIT(8),
	CSPI_ENV_ILK		= CUSTOM_ENV_BIT(9),
}
CSPI_ENVFLAGS_EBPP;

/** Available switch modes. */
typedef enum {
	/** Enable switching. */
	CSPI_SWITCH_AUTO	= 0xff,
	/** Enable direct connection (no crossover). */
	CSPI_SWITCH_DIRECT	= 0x03,
        /** Minimal switch position value  */
        CSPI_SWITCH_MIN         = 0x00,
        /** Maximal switch position value  */
        CSPI_SWITCH_MAX         = 0x0f,
}
CSPI_SWITCHMODE;

/** Available AGC modes. */
typedef enum {
	/** Manual gain control. */
	CSPI_AGC_MANUAL = 0,
	/** Enable AGC. */
	CSPI_AGC_AUTO,
}
CSPI_AGCMODE;

/** Available DSC modes. */
typedef enum {
	/** Disable DSC. Keep current DSC coefficients. */
	CSPI_DSC_OFF = 0,
	/** Disable DSC. Apply unitiy DSC coefficients. */
	CSPI_DSC_UNITY,
	/** Enable signal conditioning with DSC daemon in AUTO mode. */	
	CSPI_DSC_AUTO,
	/** Save current DSC coefficients onto FLASH /opt/dsc/lastgood.dat. */
	CSPI_DSC_SAVE_LASTGOOD,
}
CSPI_DSCMODE;

/** Available interlock modes. */
typedef enum {
	/** Disable interlock. */
	CSPI_ILK_DISABLE = 0,
	/** Enable interlock. */
	CSPI_ILK_ENABLE = 1,
	/** Enable gain-dependant interlock. */
	CSPI_ILK_ENABLE_GAINDEP = 3,
}
CSPI_ILKMODE;


/** Event specific values for event CSPI_EVENT_INTERLOCK. */
typedef enum {
        /** IL: position X out of limit. */
        CSPI_INTERLOCK_X    = LIBERA_INTERLOCK_X,

        /** IL: position Y out of limit. */
        CSPI_INTERLOCK_Y    = LIBERA_INTERLOCK_Y,

        /** IL: Attenuators set higher than predefined value. */
        CSPI_INTERLOCK_ATTN = LIBERA_INTERLOCK_ATTN,

        /** IL: ADC Overflow  (filtered). */
        CSPI_INTERLOCK_ADCF = LIBERA_INTERLOCK_ADCF,

        /** IL: ADC Overflow  (not filtered). */
        CSPI_INTERLOCK_ADC = LIBERA_INTERLOCK_ADC,
} CSPI_ILKCAUSE;

//--------------------------------------------------------------------------

/** Derived from CSPI_CONPARAMS to handle EBPP specific
 *  parameters or attributes.
 */
typedef struct {
	/** Common connection parameters. */
	CSPI_CONPARAMS_BASE;

	/** DD decimation factor. */
	size_t dec;

	/** SA non-blocking mode. */
	size_t nonblock;
}
CSPI_CONPARAMS_EBPP;

//--------------------------------------------------------------------------

/** Bit flags corresponding to the CSPI_CONPARAMS_EBPP structure.
 *  See CSPI_CONPARAMS_EBPP structure for descriptions.
 */
typedef enum {
	CSPI_CON_DEC        = CUSTOM_CON_BIT(0),
	CSPI_CON_SANONBLOCK = CUSTOM_CON_BIT(1),
}
CSPI_CONFLAGS_EBPP;

/** Backward compatibility */
typedef CSPI_CONFLAGS_EBPP  CSPI_CONFLAGS_DD;
/** Backward compatibility */
typedef CSPI_CONPARAMS_EBPP  CSPI_CONPARAMS_DD;

#endif	// _EBPP_H
