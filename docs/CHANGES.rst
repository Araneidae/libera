Change history of Libera EPICS Driver development
=================================================

.. This file is written in reStructuredText
.. default-role:: literal


2011/10/10 Version 2.05.7
-------------------------
Mostly very minor incremental changes, some changes to installation, a handful
of bug fixes, some minor extra functionality.

 - `synclibera` script removed from this distribution, maintaned elsewhere.
 - Add support for secondary interlock selection, provided by DLS FPGA for use
   in locations where it is necessary to switch under MPS control between two
   different interlock windows.
 - Correctly compute and display decimation factor when DDC factor is > 1.
 - On read error force health sensors to zero so that an alarm condition is
   signalled, rather than simply freezing the readings.
 - Fix long standing channel update bug: if an EPICS PV initialises to a
   non-zero value then writing it to zero wasn't generating an update event.
 - Add detailing logging of all caput events together with an optional
   blacklist.
 - Add support for FA payload selection when supported by FPGA.
 - Include installation of `/opt/lib` and `/opt/bin` directories, but *only*
   when targeting a DLS rootfs installation.
 - Integrate all three FA filters (notches and FIR) into EPICS control.
 - Golden Orbit PVs, `GOLDEN_{X,Y}_S` are now persistent.
 - Fix broken hardware probe in 2.05.3 installation which misfires on newer
   Libera Electron installations.
 - Include support for new programmable crystal controller.
 - Add normalised `SA:<button>N` PVs to help with checking for button drift
   during normal beam decay.


2010/03/03 Version 2.05.3
-------------------------
Add to calculations performed on FR and TT waveforms, one bug fix.

 - Support averaging FR waveforms to increase the noise to signal ratio for
   repetitive beam responses synchronised to the trigger.
 - Support tune response measurement PVs for both FR and TT waveforms, and add
   simple statistics measurement to TT as well as FR.
 - Update the documentation.
 - Fix bug in Epics and IOC Restart functions when low on memory.
 - Install separate Brilliance and Electron scaling factors for estimated power
   calculation.


2010/01/04 Version 2.05.2
-------------------------
Incremental changes, a little more monitoring, some bug fixes.

Major changes in this release:

 - Fix fairly long standing bug in calculation of `FR:STDY`.  I'm afraid this
   calculation has been broken ever since it was released!
 - Add option for open loop operation to health daemon.  Also involves renaming
   PV `SE:ENABLE_S` to `SE:HEALTHD_S`.
 - Added network statistics monitoring PVs for displaying network and channel
   access activity.
 - Default turn-by-turn window size has been increased to 128K samples to
   increase readout speed, and a corresponding new control `TT:DOREFRESH_S`
   allows for more efficient turn by turn capture.  Matlab functions `getxy` and
   `getiq` in `toolkit/matlab` have been updated accordingly.

Other minor changes:

 - References to "Fast Feedback" now renamed to "Communication Controller" in
   displays and documentation.
 - Updated build instructions and updated ntp build script.
 - DLS architecture now reported as `DLS` and correctly recognised.


2009/11/02 Version 2.05
-----------------------
Mostly incremental changes, but supports some new features.  Version number
stepped to match current Libera release.

The following major changes are in this release:

 - Support for simple (but rapid, 10dB/s) "automatic gain control".
 - Add support for internal postmortem triggers where supported by FPGA.
 - Add support for postmortem trigger offset where supported by driver.
 - Default release now based on EPICS 3.14.11

Some minor changes:

 - Allow 25 minutes for NTP daemon to synchronise before reporting NTP status in
   overall clock health: the standard health daemon can take this long.
 - A few relics have been tidied up.
 - The PV documentation is now more complete.
 - Timestamps on FT records are now more accurate.
 - Spike removal length parameter is now an mbbo selection: this may cause an
   incompatibility on upgrade; if so, simply reset spike removal length.


2009/07/17 Version 2.00.0
-------------------------
Substantial new features including full support for Libera 2.00 FPGA features.
This release works properly with 1.46, 1.60, 1.82 and 2.00 to 2.03 i-Tech
Libera releases.

The following are the major changes in this release:

 - Support multiple target architectures, including Libera 2.00 and new DLS
   target.
 - Automatically detects installation target and available system features,
   automatically makes approprate features available.
 - Support for new 2.00 functionality:

   1. "Mean Sum": computation of average beam intensity between successive
      triggers, useful for direct measurement of injection efficiency.
   2. Configuration of Spike Removal feature.
 - Support for attenuator offsets: help support mixture of Electron and
   Brilliance BPMs on the same machine.
 - Added detailed version identification PVs (useful for system diagnosis) and
   more detailed logging on IOC startup.  IOC logging now includes timestamps.
 - Add monitoring of NTP synchronisation status, included in clock health.
 - Integrate health daemon into EPICS driver, monitor both temperature sensors
   (if available), improve fan health monitoring.
 - Add support for decimated Turn by Turn capture.

The following minor changes and bug fixes are worth noting:

 - Fixed error in device link type which caused repeated broadcast requests for
   unobtainable PVs.
 - Brilliance detection works correctly on older systems.
 - Fix dependency on `DECIMATION` value by reading it directly from the FPGA.
   This helps integration on newer versions of Libera.
 - Automatically install `libstdc++.so.6` on 1.46 target if not present (a bit
   late for this now, sorry about that).
 - The gigabit ethernet controller is initialised if present (but is not
   otherwise configurable in this release).


2008/12/15 Version 1.46.4.2
---------------------------
Mostly minor changes since 1.46.4.

 - Add `CF:ATTEN_OFFSET_S` waveforms to correct for built in attenuator offsets.
 - Code now compiles without warnings with gcc 4.2.4 -- this is in preparation
   for migration to a newer environment.
 - Pick up the number of turns per switch position (`NTBT`) so that signal
   conditioning works correctly.
 - Other minor bug fixes.

There is also some preliminary support for Diamond specific FPGA features: these
are detected by looking for bit 23 set in the ITECH feature register (FPGA
register 0x1C) and bit 31 set in register 0x18.  At present the following
features are enabled:

 - Continuous MAXADC detection: used to generate `SA:MAXADC`
 - Slow (turn by turn) ADC overflow detection configuration

Note that if your Libera intermediate frequency (HARMONIC/DECIMATION mod 1) is
far away from 1/4 (values of 1/5 or 1/3 may be problematic) then some of the
first turn calculations may not work properly.  If you have this problem, let me
know.


2008/07/14 Version 1.46.4
-------------------------
Many changes in this release.

The following are important new features in this release:

 - Support for Libera Brilliance and Linux 2.6 installation.  This driver works
   with Libera 1.60 and 1.82 (though there has not been much testing on these
   platforms).


The following major changes will introduce incompatibilites:

 - The EPICS driver now completely replaces much of the preexising Libera driver
   functionality.  Two changes in particular will be visible:

   1. The script used to start, stop or restart the Libera system is now
      `/etc/init.d/libera-driver` rather than libera (which is renamed by
      installation to `old-libera`, and can safely be removed).
   2. CSPI applications (such as the libera application) will no longer work.
 - ADC readings reported by FT processing are now scaled to 16 bit values.  This
   is designed for compatibility with Libera Brilliance.

The following extra features are worth noting.  For more details see the README
file for details.

 - New beam position statistics have been added to `FR` data.
 - Add new `IL:IIRK_S` configuration setting to control a filter in the ADC
   overflow detection path.
 - Add new `IL:TEST_S` PV for testing the interlock.
 - Fix exceptionally long standing bug in Vertical mode calculation: if a
   non-zero origin is specified then all positions were returned as zero, due
   to a << vs - precedence error!
 - Add support for trigger offset to FR and TT waveforms.
 - Improve FT WF position calculations by using a proper 8 point filter for
   decimating raw ADC data down to display resolution.  This filter turns out
   to add a negligible amount of time to processing.

   Note that this filter is currently hard wired and relies on the intermediate
   frequency being close to 1/4 sample frequency.
 - Add `FT:WF` < `X`, `Y`, `Q`, `S`> and `FT:MAXS` pvs to provide an estimate of
   "intra-turn" beam position.
 - Change all screens from arial to helvetica fonts.  The Helvetica font has
   the virtue of having fixed pitch digits.
 - Ensure that the DONE PV updates with a current timestamp.  To ensure that its
   processing can be observed it is changed from a bo to a longout record.
 - Revisit the power and charge calculations.  The key scaling parameter `S_0`
   is now a configuration parameter for each affected mode (SA and FT).
 - Implement interlock holdoff when changing signal conditioning.  Also means
   that we can reduce the default interlock holdoff interval.
 - Change first turn ADC readouts to return full 16 bit values.  In
   pre-brilliance Libera we return zeros in the bottom four bits, in Brilliance
   we return the full 16 bit ADC reading.
 - Create new `SC` signal conditioning code to replace DSC daemon.


2007/09/27 Version 1.46.1
-------------------------
Support for Libera 1.46 driver.  Note that this version will not work correctly
with earlier versions of the driver.

The only change is to track an incompatible change in the driver interface.


2007/06/12 Version 1.40.2
-------------------------
More complete support for iTech Libera 1.40 and 1.42 driver.

This release has been developed and tested using gcc 3.4.5 with glibc 2.3.6.
Switching to this compiler and library has had two effects:

 - overall stability of Libera is improved; but at a cost:
 - turn by turn operation is much less stable.  See `PROBLEMS` for details.

Major changes:

 - Master BPM enable PV added.  This allows a BPM to be marked as "disabled" if
   it is returning unreliable positions.  The only direct effect on the BPM is
   to disable operation of the interlock.
 - Significant changes to "geometry" configuration.  The simple beam offset PVs
   X0 and Y0 have been replaced by three separate displacements:
   :`BBA_X,_Y`:     "Beam Based Alignment" offsets, persistent over reboots
   :`BCD_X,_Y`:     "Beam Current Dependent" offset, initialised to 0 on reboot
   :`GOLDEN_X,_Y`:  Special purpose local offsets (for bumps, etc), 0 on reboot
 - Control over DSC ("Digital Signal Conditioning") is now available through a
   new configuration PV.
 - Control over switch triggering, both selection of trigger source (internal
   or external) and trigger delay (when using external trigger).
 - Control over the clock is more refined and the machine clock tracking
   algorithm is now 'much' more stable.  The clock control PVs have moved to a
   separate PV group.
 - Machine clock synchronisation is now separated from system clock
   synchronisation, and in this release system clock synchronisation is not
   actually required.
 - The position interlocking now automatically switches off when the current is
   below a preset threshold, as well as switching on above a threshold.
 - Interlock reasons are now recorded.
 - Voltage monitoring now included in overall system health.
 - caRepeater now integrated into IOC, so now no need for separate EPICS
   installation: IOC is just a single application.

Minor changes:

 - MAXADC alarm limits changed to 1450 (-3dB) and 1700 (-1.6dB).
 - Core file writing can be supported by setting the `IOC_CORE` symbol in
   `/etc/sysconfig/epics_ioc`.  Set it to an NFS mounted directory.

There are significant changes to the installation process: read `INSTALL` for
detailed installation instructions.


2007/01/15 Version 1.40.0
-------------------------
Preliminary release to support iTech Libera 1.40 driver.

This is a stop-gap release to provide EPICS support for the 1.40 Libera driver:
this release does not support any new Libera functionality.  A more fully
featured release providing support for for 1.40 features will follow soon.

Significant changes:

 - Incorporate CSPI 1.40 support including updated CSPI files.
 - Add support for precision timestamps in EPICS for data types where this is
   available (modes `BN`, `FR`, `PM`, `TT`).

Minor changes:

 - Fix bash compatibility bug introduced in `/etc/init.d/epics` script in 0.6.3.
 - Version numbering now changed to track Libera driver version numbers.


2006/10/02 Version 0.6.3
------------------------
Minor changes from Version 0.6.1

Significant changes:

 - Add control over lmtd to provide support for "double detune": this is
   designed to reduce interference from revolution frequency sidebands.
 - New PVs to remotely restart EPICS and remotely reboot the IOC.
 - Improve the calculation of FT:CHARGE.  The new algorithm is virtually
   independent of the profile of the bunch train, though it is still not
   particularly accurate for low signal levels.
 - Add support for integrating IOC startup process into Libera startup script.

Minor changes:

 - Changed name of `FT:RAW` waveforms from <buttons> to <channel> to reflect the
   fact that these waveforms do not follow the buttons as the switch changes.
 - New `EPICSUP` field to record EPICS IOC up time.
 - Diamond specific fast feedback support is present in this release, but not
   compiled in in the default configuration.
 - Revisit interlock and attenuator handshaking to try avoiding dropping the
   interlock when changing the interlock.  This is still work in progress...
 - support files renamed to numeric to more accurately reflect their role.
 - Incorporate rather unfortunate patch to release 0.6.1!
 - Remove screen support from `/etc/init.d/epics` startup script.
 - The libera test application now builds in this environment.


2006/08/17 Version 0.6.1
------------------------
Works with Libera 1.21

Significant changes:

 - Automatic switching and DSC (digital signal conditioning) can now be switched
   on or off.  This should normally be on when accurate slow acquisition (and
   fast feedback) data is required, and should be off for first turn and turn by
   turn data.
 - The machine protection interlock can now be controlled and configured.
 - Slow acquisition mode now estimates beam current.
 - First turn now estimates charge in measured bunch train.
 - More detailed monitoring of Libera status, including accurate free memory,
   ram disk usage, cpu consumption.
 - IQ data available for decimated /64 waveforms.

Minor changes:

 - Added toolkit files.
 - Sensor PVs (temperature, fan speeds etcetera) reimplemented and renamed.
 - Descriptions added to all records.
 - Configuration readback PVs now removed.
 - Significant rearrangements of code, together with necessary changes to
   track substantial changes in CSPI interface.
 - ai/ao records now perform conversion to floating point in the EPICS layer.


2006/07/06 Version 0.5
----------------------
Works with Libera 1.00 iTech drivers

Note that the description of the installation process in `INSTALL` has changed:
this release includes scripts to run on Libera to automate part of this install
process.

Significant changes:

 - New `HEALTH` record which collects together alarm severities for temperature,
   fans and free memory.
 - New `TICK` record which records time since last processed trigger and records
   an alarm status after 1 and 10 seconds.
 - Redefine orientation of vertical mode: button/stripline B is now deemed to be
   in the negative X direction.  This makes the logic consistent with
   conventions at Diamond.
 - Postmortem support now enabled: this seems to work ok with the latest drivers
   from iTech.
 - New `libera-install-ioc` and `libera-install-epics` scripts.  These can be
   used to install the EPICS libraries and Libera EPICS driver on the IOC.  Note
   that the `INSTALL` instructions have changed to reflect this.
 - `/etc/init.d/epics stop` script now copes if the driver has locked up and
   forcibly kills it if necessary.
 - EDM control screens now included in this distribution.

Minor changes:

 - Temperature, fan and memory alarm limits defined.  Memory records `USED`
   and `CACHE` withdrawn (still looking for a good measure of free memory).
   Unused `TEMP2` record also deleted.
 - Change implementation of interlocking with EPICS layer, used to rule out the
   EPICS layer from any involvement with Libera lockups (it is clear now that
   these occur in the iTech driver).
 - Removed rather arbitrary limits on `TT`, `TW`, `FR` and `BN` IOC
   configuration parameters: if the installer insists on killing the IOC by
   setting unreasonable values, be my guest.


2006/03/13 Version 0.4
----------------------
Works with Libera 1.00

Major changes:

 - Uses Libera 1.00 driver, uses CSPI library interface, now cooperates with
   leventd!
 - Implements support for adc rate buffer and slow acquisition.
 - Unprocessed I/Q data now exposed through EPICS interface.
 - Separated out turn-by-turn processing into separate long capture and free
   running modes
 - Conversion routines now much more efficient (cordic in 135ns!)
 - Buffer and capture lengths can now be configured at startup time.
 - Persistent state now supported for basic configuration parameters.

The following incompatible changes with the previous release should be noted:

 - The `FT` (first turn) buffer is now sampled at 32ns intervals rather than at
   turn-by-turn rate.  The default sample window has also been changed
   accordingly.

 - The "free running" trigger mode for `TT` (turn-by-turn) capture is no longer
   supported, instead a separate `FR` (free running) mode is provided.

 - The file `/etc/sysconfig/epics_ioc` now contains important startup
   configuration information which must be provided before the IOC will start.

 - The loading of initial configuration has changed.  `IOC_STATE_PATH` should be
   defined appropriately and an initial configuration can be written through the
   EPICS interface.


2006/02/06 Version 0.2
----------------------
First published release, supporting Libera 0.92-2 driver only.
