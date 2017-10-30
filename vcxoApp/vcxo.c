/* $Id: vcxo.c 3994 2011-04-13 11:24:29Z hinko.kocevar $ */

/*
Silab Si57x VCXO frequency programmer.
Copyright (C) 2011 Instrumentation Technologies

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA
or visit http://www.gnu.org

TAB = 4 spaces.
*/

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include "i2c-dev.h"


#define VERSION					"0.2 (Apr 13, 2011)"

/* Verbose output, set to 1 to enable more output. */
static int verbose = 0;
#define VERBOSE(x...)	{ if (verbose) \
	{ fprintf (stderr, "%s:%d ", __FUNCTION__, __LINE__); \
	printf (x); } \
}

/* Error messages. */
#define ERROR(x...)		{ \
	fprintf (stderr, "%s:%d ERROR ", __FUNCTION__, __LINE__); \
	printf (x); \
}

/* I2C Si57x chip address. */
#define VCXO_I2C_ADDRESS		0x55

/* VCXO start-up output frequency after power-up is set to 100,00 MHz. */
#define STARTUP_FREQUENCY		100000000.0

#define LOW_DCO_FREQUENCY		(4.85 * 1e9)
#define HIGH_DCO_FREQUENCY		(5.67 * 1e9)

/* VCXO divider parameters: hs_div, n1 and RFREQ. */
struct dividerData {
	int hsdiv_reg;
	int n1div_reg;
	int hsdiv;
	int n1div;
	unsigned long long lrfreq;
	double dfdco;
};

/* HS divider register value <--> value mapping. */
static const int hsdiv_reg[6] = {0, 1, 2, 3, 5, 7};
static const int hsdiv_val[6] = {4, 5, 6, 7, 9, 11};

static int chip_fd = -1;

/*
 * See Si57x datasheet, page 18 for more information about the dividers.
 */
/* DCO high speed divider. */
static int hsdiv_reg2val (int reg) {
	int i;
	for (i=0; i<6; i++)
		if (hsdiv_reg[i] == reg)
			return hsdiv_val[i];
	return -1;
}

static int hsdiv_val2reg (int val) {
	int i;
	for (i=0; i<6; i++)
		if (hsdiv_val[i] == val)
			return hsdiv_reg[i];
	return -1;
}

/* CLKOUT output divider range. */
#define N1DIV_REGISTER_MIN		0
#define N1DIV_REGISTER_MAX		127
#define N1DIV_VALUE_MIN			1
#define N1DIV_VALUE_MAX			128

static int n1div_reg2val (int reg) {
	if (reg >= N1DIV_REGISTER_MIN && reg <= N1DIV_REGISTER_MAX)
		return (reg + 1);
	return -1;
}

static int n1div_val2reg (int val) {
	if (val >= N1DIV_VALUE_MIN && val <= N1DIV_VALUE_MAX)
		return (val - 1);
	return -1;
}

static int i2c_open(int i2c_devnr, int i2c_address) {
	char filename[20];

	snprintf(filename, 19, "/dev/i2c-%d", i2c_devnr);
	chip_fd = open(filename, O_RDWR);
	if (chip_fd < 0) {
		ERROR("open() error: %d, %s\n", errno, strerror(errno));
		return -1;
	}
	VERBOSE("opened path %s, fd = %d..\n", filename, chip_fd);

	/* Set I2C address. */
	VERBOSE("using I2C address 0x%X..\n", i2c_address);
	if (ioctl(chip_fd, I2C_SLAVE, i2c_address) < 0) {
		ERROR("ioctl() error: %d, %s\n", errno, strerror(errno));
		return -1;
	}

	return 0;
}

static void i2c_close(void) {
	if (chip_fd > 0) {
		VERBOSE("closing I2C, fd = %d..\n", chip_fd);
		close(chip_fd);
		chip_fd = -1;
	}
}

static int i2c_read(unsigned char reg) {
	int res;

	res = i2c_smbus_read_byte_data(chip_fd, reg);
	if (res < 0) {
		ERROR("i2c_smbus_read_byte_data() error: %d, %s\n", errno,
				strerror(errno));
	} else {
		/* res contains the read word */
		VERBOSE("reg %02X, data %02X\n", reg, (unsigned char) res & 0xFF);
	}

	return res;
}

static int i2c_write(unsigned char reg, unsigned char val) {
	int res;

	res = i2c_smbus_write_byte_data(chip_fd, reg, val);
	if (res < 0) {
		ERROR("i2c_smbus_write_byte_data() error: %d, %s\n", errno,
				strerror(errno));
	}

	return res;
}

static int i2c_read_silent(unsigned char reg) {
	return i2c_smbus_read_byte_data(chip_fd, reg);
}

/*
 * See Si57x datasheet pages 13 - 16 on how to access and program VCXO.
 */

/* VCXO presence check - read known register. */
static int check_vcxo (void) {
	int ret;

	/* If read byte succeeds, then the chip is there. */
	ret = i2c_read(7);

	if (ret < 0) {
		VERBOSE("reading register 7 chip.. FAILED!\n");
	} else if ((ret > 0) && (ret < 0xff)) {
		VERBOSE("reading register 7 chip.. SUCCESS!\n");
		ret = 0;
	}

	return ret;
}

/*
 * VCXO sets RFREQ default values (for 100.00 MHz) only IF we have not
 * programmed our own values, yet. This is true until VCXO is power cycled.
 *
 * To get default RFREQ values from (NVM) in registers, we need to issue
 * RECALL command via I2C - see Si57x datasheet page 20, register 135, bit 0.
 */
static int recall_nvm_vcxo (void) {

	int ret;

	VERBOSE("recalling NVM values ..\n");
	/* Reload NVM values to registers. I2C will not be disturbed. */
	ret = i2c_write(135, 1 << 0);

	int retry = 10;
	while (i2c_read_silent(7) < 0) {
		usleep(10000);
		if (--retry <= 0)
			return -1;
	}

	return 0;
}

static int extract_divider_data (unsigned long long ldata,
		struct dividerData *data) {

	/* Parse the retrieved data into structure, do some conversions. */
	data->hsdiv_reg = (ldata >> 45) & 7;
	data->n1div_reg = (ldata >> 38) & 0x3F;
	data->lrfreq = ldata & 0x3FFFFFFFFFULL;
	data->hsdiv = hsdiv_reg2val(data->hsdiv_reg);
	data->n1div = n1div_reg2val(data->n1div_reg);
	data->dfdco = 0;


	return 0;
}

static int read_rfreq (struct dividerData *data) {

	/*
	 * STEP 1
	 *
	 * Read start-up frequency configuration (RFREQ, HS_DIV, and N1) from the
	 * device after power-up or register reset.
	 *
	 * NOTE: Device returns 100.00 MHz configuration *ONLY* if not programmed.
	 * This is true after power-up or register reset.
	 */

	int r = 7;
	int v;
	int sh = 40;
	unsigned long long ldata = 0;

	/*
	 * Read 5 registers (7 - 12) that provide information for RFREQ, HS_DIV,
	 * and N1.
	 */
	while (r <= 12) {
		v = i2c_read(r);
		if (v < 0) {
			return v;
		}

		/* Construct the long long value. */
		ldata |= ((unsigned long long)v & 0xFF) << sh;
		r++;
		sh -= 8;
	}

	VERBOSE("read data LLONG: %llX\n", ldata);

	return extract_divider_data(ldata, data);
}

static int calc_new_rfreq (unsigned long long lfrequency,
		struct dividerData *old_div, struct dividerData *new_div) {
	/*
	 * STEP 2
	 *
	 * Calculate the actual nominal crystal frequency where f0 is the start-up
	 * output frequency:
	 *
	 * fxtal = ( f0 x HS_DIV x N1 ) / RFREQ
	 */

	double drfreq = 0;
	double dfxtal = 0;
	int new_hsdiv = 0;
	int new_n1div = 0;
	double new_ddco = 0;

	drfreq = (double) old_div->lrfreq / (1 << 28);
	dfxtal = (double) ( STARTUP_FREQUENCY * old_div->hsdiv * old_div->n1div )
			/ drfreq;
	VERBOSE("frfreq %f, fxtal %f\n", drfreq, dfxtal);

	/*
	 * STEP 3
	 *
	 * Choose the new output frequency (f1).
	 */

	VERBOSE("new frequency (f1) = %lld Hz (%f MHz)\n", lfrequency,
			lfrequency / 1e6);

	/*
	 * STEP 4
	 *
	 * Choose the output dividers for the new frequency configuration (HS_DIV
	 * and N1) by ensuring the DCO oscillation frequency (dfdco) is between
	 * 4.85 GHz and 5.67 GHz where dfdco = f1 x HS_DIV x N1. See the Divider
	 * Combinations tab for more options.
	 */

	VERBOSE("DCO limits: %f - %f GHz\n", LOW_DCO_FREQUENCY / 1e9,
			HIGH_DCO_FREQUENCY / 1e9);

	/* Initialize HS_DIV and N1 to lowest values. */
	new_hsdiv = 4;
	new_n1div = 1;
	memset(new_div, 0, sizeof(struct dividerData));

	VERBOSE("INITIAL : hs_div %d (reg %d), n1_div %d (reg %d), rfreq %llX\n",
			old_div->hsdiv, old_div->hsdiv_reg, old_div->n1div,
			old_div->n1div_reg, old_div->lrfreq);

	while (new_hsdiv < 12) {

		new_ddco = lfrequency * new_hsdiv * new_n1div;

		if (new_ddco > LOW_DCO_FREQUENCY && new_ddco < HIGH_DCO_FREQUENCY) {

			/* Try to get best guess! */
			if (new_div->dfdco == 0) {
				VERBOSE("SELECT  : hs_div %3d, n1_div %3d, DCO %f Hz (%f GHz)\n",
						new_hsdiv, new_n1div, new_ddco, new_ddco / 1e9);

				new_div->hsdiv = new_hsdiv;
				new_div->n1div = new_n1div;
				new_div->dfdco  = new_ddco;

			} else if (abs(new_hsdiv - old_div->hsdiv) <
					abs(new_div->hsdiv - old_div->hsdiv)) {
				VERBOSE("SELECT  : hs_div %3d, n1_div %3d, DCO %f Hz (%f GHz)\n",
						new_hsdiv, new_n1div, new_ddco, new_ddco / 1e9);

				new_div->hsdiv = new_hsdiv;
				new_div->n1div = new_n1div;
				new_div->dfdco  = new_ddco;

			} else {
				VERBOSE("GOOD    : hs_div %3d, n1_div %3d, DCO %f Hz (%f GHz)\n",
						new_hsdiv, new_n1div, new_ddco, new_ddco / 1e9);
			}
		}

		/* Increment N1 in steps of 2. */
		if (new_n1div == 1)
			new_n1div += 1;
		else
			new_n1div += 2;

		/* Increment HS_DIV and reset N1 to 1. */
		if (new_n1div > 128) {
			new_n1div = 1;

			new_hsdiv++;

			/* skip invalid values */
			if (new_hsdiv == 8 || new_hsdiv == 10)
				new_hsdiv++;
		}
	}

	/* Were new dividers found? */
	if (new_div->n1div) {

		/*
		 * STEP 5
		 *
		 * Calculate the new crystal frequency multiplication ratio (RFREQ) as
		 * RFREQ = fdco / fxtal
		 */

		drfreq = new_div->dfdco / dfxtal;
		drfreq = (double) drfreq * (1 << 28);
		new_div->lrfreq = (unsigned long long) drfreq;

		new_div->hsdiv_reg = hsdiv_val2reg(new_div->hsdiv);
		new_div->n1div_reg = n1div_val2reg(new_div->n1div);

		VERBOSE("FINAL   : hs_div %d (reg %d), n1_div %d (reg %d), rfreq %llX\n",
				new_div->hsdiv, new_div->hsdiv_reg, new_div->n1div,
				new_div->n1div_reg, new_div->lrfreq);

		return 0;
	}

	return -1;
}

static int write_rfreq(struct dividerData *data) {
	int val;
	int ret;

	/*
	 * STEP 6
	 *
	 * Freeze the DCO by setting Freeze DCO = 1 (bit 4 of register 137).
	 */

	ret = i2c_write(137, 1 << 4);
	if (ret < 0) {
		return ret;
	}

	/* Optionally freeze the M Control word. */
	ret = i2c_write(135, 1 << 5);
	if (ret < 0) {
		return ret;
	}

	/*
	 * STEP 7
	 *
	 * Write the new frequency configuration (RFREQ, HS_DIV, and N1).
	 */

	val = (data->hsdiv_reg << 5) | (data->n1div_reg >> 2);
	VERBOSE("Setting reg %d = %02X\n", 7, val);
	ret = i2c_write(7, val);
	if (ret < 0) {
		return ret;
	}

	val = ((data->n1div_reg & 0x03) << 6) | ((data->lrfreq >> 32) & 0x3F);
	VERBOSE("Setting reg %d = %02X\n", 8, val);
	ret = i2c_write(8, val);
	if (ret < 0) {
		return ret;
	}

	val = (data->lrfreq >> 24) & 0xFF;
	VERBOSE("Setting reg %d = %02X\n", 9, val);
	ret = i2c_write(9, val);
	if (ret < 0) {
		return ret;
	}

	val = (data->lrfreq >> 16) & 0xFF;
	VERBOSE("Setting reg %d = %02X\n", 10, val);
	ret = i2c_write(10, val);
	if (ret < 0) {
		return ret;
	}

	val = (data->lrfreq >> 8) & 0xFF;
	VERBOSE("Setting reg %d = %02X\n", 11, val);
	ret = i2c_write(11, val);
	if (ret < 0) {
		return ret;
	}

	val = data->lrfreq & 0xFF;
	VERBOSE("Setting reg %d = %02X\n", 12, val);
	ret = i2c_write(12, val);
	if (ret < 0) {
		return ret;
	}

	/*
	 * STEP 8
	 *
	 * Un-freeze the DCO by setting Freeze DCO = 0 and assert the NewFreq bit
	 * (bit 6 of register 135) within 10 ms.
	 */

	ret = i2c_write(137, 0 << 4);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_write(135, 1 << 6);
	if (ret < 0) {
		return ret;
	}

	/* Optionally un-freeze the M Control word, if frozen above. */
	ret = i2c_write(135, 0 << 5);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static void usage (char *app) {
	fprintf(stderr,
			"\nSilab Si57x VCXO RFREQ frequency programmer\n"
			"Version: %s\n"
			"License: GPLv2\n\n"
			"Usage: %s [-v] [-i i2caddr] [-c] frequency\n"
			"SWITCHES:\n"
			" -c            check chip presence at I2C address, and exit\n"
			" -i i2caddr    I2C address in HEX (default 0x%02X)\n"
			" -v            be verbose\n"
			"\nPARAMETERS:\n"
			" frequency     new VCXO frequency, in dHz\n"
			"\n", VERSION, app, VCXO_I2C_ADDRESS);
}

int main (int argc, char **argv)
{
	int c;
	int i2c_addr;
	struct dividerData old_data, new_data;
	unsigned long long new_freq;
	double new_freq_mhz;
	int check_only;
	int ret;

	/* Some defaults. */
	i2c_addr = VCXO_I2C_ADDRESS;
	check_only = 0;

	while ((c = getopt(argc, argv, "ci:v")) != -1) {
		switch (c) {
			case 'c':
				/* Check for I2C device. */
				check_only = 1;
				break;
			case 'i':
				/* Set new I2C address. */
				i2c_addr = strtoul(optarg, NULL, 16);
				break;
			case 'v':
				/* Be verbose! */
				verbose = 1;
				break;
			default:
				usage(argv[0]);
				exit(1);
		}
	}

	VERBOSE("argc %d, optind %d\n", argc, optind);
	if (check_only == 0 && optind >= argc) {
		usage(argv[0]);
		exit(1);
	}

	ret = i2c_open(0, i2c_addr);
	if (ret < 0)
		exit(1);

	/* Check if the chip is there - read a I2C register. */
	ret = check_vcxo();
	if (ret < 0) {
		ERROR("Silab Si57x VCXO NOT found!\n");
		goto out;
	}

	/* If only checking for the chip, exit here. */
	if (check_only) {
		goto out;
	}

	new_freq = strtoull(argv[optind], NULL, 10) / 10;
	new_freq_mhz = (double)(new_freq / 1e6);

	fprintf(stderr, "Silab Si57x VCXO frequency %lld Hz (%f MHz)\n",
			new_freq, new_freq_mhz);

	ret = recall_nvm_vcxo();
	if (ret < 0) {
		VERBOSE("END FAIL!\n");
		goto out;
	}

	ret = read_rfreq(&old_data);
	if (ret < 0) {
		VERBOSE("END FAIL!\n");
		goto out;
	}

	ret = calc_new_rfreq(new_freq, &old_data, &new_data);
	if (ret < 0) {
		VERBOSE("END FAIL!\n");
		goto out;
	}

	ret = write_rfreq(&new_data);
	if (ret < 0) {
		VERBOSE("END FAIL!\n");
		goto out;
	}

	ret = 0;
	VERBOSE("END OK!\n");

out:
	i2c_close();
	exit(ret);
}
