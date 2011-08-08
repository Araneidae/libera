Proposal for DLS DSC Daemon
===========================
:Author: Michael Abbott <michael.abbott@diamond.ac.uk>

.. This file is written in reStructuredText
.. default-role:: literal
.. contents::
.. sectnum::


Introduction
------------

In the Libera EPICS driver developed at Diamond Light Source (DLS) the Signal
Conditioning process provides similar functionality to the Instrument
Technologies (I-Tech) Signal Conditioning Daemon, but has a completely
different implementation and somewhat different behaviour.  This document
describes a proposal to adapt the DLS signal conditioning process into a
separate DSC (Digitial Signal Conditioning) daemon so that it can be integrated
into the I-Tech framework.

There are the following issues in this conversion.

1.  Firstly, the DLS Signal Conditioning process is currently integrated into
    the main EPICS driver IOC.  The proposal here is to create a separate
    daemon, the DLS DSC Daemon, with a clearly defined communications interface,
    and to modify the DLS Libera EPICS driver to use this new interface.

    The existing functionality required by the DLS EPICS driver will drive the
    functionality of this interface.

2.  The standard interface to control software running on the I-Tech Libera
    software environment is through the I-Tech CSPI library.  It will be
    necessary to modify the CSPI library to use the newly defined DLS DSC daemon
    interface, and may be some impacts on the design of the DSC daemon to help
    with CSPI compatibility.

3.  The behaviour of the DLS DSC is rather different from the behaviour of the
    I-Tech DSC, and managing this is likely to raise some issues.  In particular
    there needs to be some extended discussion of the impact on the CSPI
    interface.

    It may be necessary to make significant incompatible changes in parts of the
    CSPI interface, which in turn will mean that existing client systems will
    need some rework.

In the rest of this document I'll discuss the scope of the changes and the
impact of the issues described above.


Scope of DLS DSC
----------------

The core purpose of the signal conditioning process is to manage four separate
two point phase and magnitude FIRs on the four ADC channels, used to compensate
for gain and phase changes as the button signals are multiplexed among the RF
amplifiers on the RF board.  This is done by periodically reading fairly short
bursts of turn by turn data, computing the appropriate corrections and applying
them.

This core functionality is intermingled with control over switching and control
over attenuation.  Control over switching is essential, as the entire purpose of
signal conditioning is to correct for switching artefacts, and it is necessary
for signal conditioning to interact with the attenuation control because
changing attenuators does have a (small) impact on the conditioning settings.

Attenuator control also needs to be integrated to some degree with interlock
control, as otherwise it is possible to lose the interlock when the attenuation
changes, as there is frequently an unavoidable transient position glitch when
attenuators are changed.  Thus the DLS EPICS driver briefly disables the
interlock when changing attenuation, and this will need to be part of the DLS
DSC functionality.

The I-Tech DSC daemon also provides auto-gain functionality which automatically
adjusts the attenuator settings to optimise the signal level.  The DLS EPICS
driver has similar functionality which would be integrated into the DLS DSC.  As
the DLS autogain relies on reading a MAX_ADC register provided by the FPGA which
cannot usefully be shared it will be necessary to incorporate access to this
reading into the DLS DSC daemon.

Finally, DLS signal conditioning makes available a number of EPICS PVs which
report the status of the conditioning activity.  This information will need to
be provided through the interface to the new DLS DSC daemon.


Abstract Interface to DLS DSC Daemon
------------------------------------
To support the functionality discussed above and to ensure proper integration
with existing DLS EPICS driver functionality the interface to the DLS DSC daemon
will need to provide the following functions.

Control Parameters
..................
To help with CSPI integration it is likely that most of the control parameters
listed here will be configurable on the command line, but certainly when being
controlled from the DLS EPICS driver these will be set through the daemon
interface.  Each listed parameter can also be read.

Attenuation: 0 to 62 dB
    The attenuation setting is set directly in dB.  On Libera Electron the
    setting can be any number between 0 to 62dB, distributed arbitrarily
    (evenly, in practice) between the two attenuators in each amplifier chain,
    while on Libera Brilliance the setting is any number from 0 to 31dB.

    There is an interaction between attenuation setting and the interlock --
    changing the attenuation can cause a transient position spike causing the
    interlock to drop.  This may require incorporation of interlock control into
    the DSC daemon if we can't find a better way to manage this.

Switch control: rotating or fixed
    Switch rotation can be switched on or off.  Turning rotating switches off
    will automatically disable signal conditioning.  The sequence of switches
    and number of switch positions (8 for Electron, 4 for Brilliance) is
    hard-wired in the DSC daemon.

Manual switch position: 0 to 15
    When switches are fixed the switch selection can be separately controlled.
    The precise permutation from button to channel depends on the RF board, and
    there is a readback for this permutation (listed below).

DSC control: unity gain, fixed gain, automatic
    This is the master control for signal conditioning with three possible
    settings:

    unity gain
        No gain compensation on channels
    fixed gain
        Channel gain compensation is programmed according to last successful DSC
        operation, but signal conditioning is not running
    automatic
        Signal conditioning runs and updates the channel gains as appropriate.
        Setting automatic will also force switch control to rotating.

Autogain enable: on or off
    If on then the attenuation setting will be automatically adjusted up or down
    to ensure that the MAX_ADC register reading lies within the configured
    autogain limits.

Autogain up and down limits: 0 to 100 %
    Autogain works up increasing or decreasing the attenuation when the MAX_ADC
    reading is outside these programmed limits, configured as a percentage of
    maximum ADC range.

MAX ADC update rate, autogain update rate
    These will need to be new parameters.  At present the MAX ADC update and
    autogain are paced by SA data, but this is not sensible for a separate DSC
    daemon, so this will be done using a timer.

DSC application IIR
    This is the filter coefficient for a one pole IIR used to smooth the
    application of channel compensation changes.

DSC update interval
    The rate of DSC computations is controlled here.

DSC deviation limit
    This is a threshold for controlling whether captured DSC data is adequate
    for signal conditioning computation.

Interlock control
    At the very least it will be necessary for interlock on/off control to be
    managed by the DSC daemon.  A closer study of the existing low level
    interlock interface will be needed to determine whether this can be
    separated from the detailed interlock settings.


Readbacks
.........

All of the parameters listed above can be read back from the daemon, and the
following further values are available.  Either single-shot or on-update reading
will have to be supported.

Switch permutation
    The mapping from button to ADC channel for the currently configured switch
    (only meaningful when manual switch control is enabled) can be read.

Signal deviation and status
    Measured signal deviation for last DSC update, together with an indication
    of whether this measurement was accepted for processing.

Button phases
    Phase differences among the four buttons as measured for the last DSC
    update, relative to the phase of button A.

Channel gains
    Angle, magnitude and variance measured for the four ADC channels.

Debug waveforms
    There are some large debug waveforms currently provided by the Libera
    driver.  Some will probably be discarded, but the IQ data used to measure
    the last compensation will continue to be available, as this provides
    occasionally very useful data.


Interaction with CSPI
---------------------

The set of "environment parameters" controlled by CSPI is rather long and needs
careful review.  Some will map directly to the DLS DSC, others will require some
translation.  It may be desirable to add the extra functionality provided by
the daemon.

The following `CSPI_ENVPARAMS` fields documented in `ebpp.h` (version 2.00) will
be affected in some way.

`switches`
    This is used to control the switching position.  This can map fairly
    directly into the DLS DSC daemon.

`gain`
    The model for gain is completely different.  CSPI talks in terms of dBm, in
    other words, target input power, whereas the DLS DSC works directly in dB
    attenuation.  A mapping can be faked, at the simplest by pretending `gain` =
    MAX_ATTEN - `atten`.

`agc`
    Maps pretty directly to autogain on or off control.

`dsc`
    Not all of the CSPI modes need be supported, the remaining modes map cleanly
    to the DLS daemon.

`ilk`
    The handling of interlock parameters remains an open issue in this note.
    Certainly `ilk.mode` will need to be managed by the DLS DSC daemon.

`max_adc`
    This will have to go via the DSC daemon rather than going directly to the
    driver.

Access to extra parameters can possibly be added as required.

Note that there is a fairly fundamental difference between the CSPI parameter
interface model and that used by the DLS DSC daemon: CSPI only supports reading
parameter and status information on demand ("single shot" readout), whereas the
daemon will support continuous updates.  This could be supported through CSPI by
defining a new data source, but I doubt this will be necessary.


Miscellaneous Issues
--------------------

A number of miscellaneous issues are gathered here.

*   How many clients will the daemon interface support?  There are two
    possibilities.

    1.  The simplest possibility is as currently implemented for existing DLS
        daemons, which is one client only at a time.  This can be implemented
        very easily with two dedicated pipes in `/tmp`.

    2.  A more complete option is to allow multiple clients to connect and
        subscribe to status updates from the DSC.  This can be done through a
        single Unix socket in `/tmp`, but requires a somewhat more complex
        broadcasting mechanism within the daemon to respond to each connection.

    Unfortunately with CSPI it is hard to know how many clients need to be
    supported, so it seems likely that option (2) will need to be implemented.
    This needs to be clarified.

*   Attenuation setting and interlock.  In the Libera EPICS driver whenever the
    attenuation settings are changed the machine protection interlock is briefly
    disabled to avoid accidentally dumping the beam if a transient position
    spike is generated.

    This functionality needs to be revisited and may need to be integrated into
    the DSC daemon.  Alas, if so, this increases the number of control
    parameters and the complexity of the daemon.

    However, this may simply be a matter of writing the interlock parameters
    through the DSC daemon.  Not hugely desirable, but feasible.  Alternatively
    it may be possible to just implement the interlock enable in the DSC daemon,
    this is the preferable solution.

*   Pacing of autogain and access to MAX_ADC.  The existing DLS EPICS driver
    uses SA (Slow Acquisition, 10Hz) updates to pace both autogain and MAX_ADC;
    this will probably need to be redesigned for the DLS DSC daemon, but we can
    also gain more control over the rate of autogain.

*   Clock synchronisation.  The DLS DSC process will not work if the sample
    clock is poorly synchronised.  We may want to think about including a
    reintegration of the DLS clock daemon.

*   Interfacing with hardware.  The Libera EPICS driver uses direct writes to
    hardware registers for a number of parameters.  It would be good to avoid
    this where possible in a shared daemon such as the DLS DSC daemon, so a
    review of the relevant affected parameters follows.

    Interlock settings
        A number of DLS specific interlock settings are written directly to
        register, including the secondary interlock settings and the ADC
        overflow limit register, both DLS exclusive features.

        Handling this correctly with interlock hold-off in the DSC daemon will
        require some careful inspection of the affected code.

    DSC control
        The entire DSC control interface is done through `/dev/libera.dsc` which
        is memory mapped to a subset of the FPGA registers.  It should be
        possible to move this entire interface to the DSC daemon unchanged.

        There are a handful of other control parameters which are only available
        through the `/dev/libera.dsc` device interface, and it may be
        appropriate to control these through the DSC daemon as well.

    MAX ADC
        This is currently read directly from the appropriate hardware register
        for the maximum degree of compatibility (there are two FPGA options, and
        not all driver version support this register).  We may need to settle on
        requiring a sufficiently recent driver and going via the driver.


Outstanding Questions
---------------------
The following questions need to be addressed.

1.  Single client or multiple client?  Will more than one simultaneous instance
    of the CSPI library be running on the target system?  If there is no clear
    answer then multiple clients will have to be supported.

2.  How will clients interface with the new DLS DSC daemon?  To what degree will
    the existing CSPI interface serve, and will clients want access to the
    extended functionality?  What will be the integration issues?

3.  What problems will this rework address?  Both ESRF and PETRA-3 are reporting
    problems with the I-Tech DSC daemon, we need to be confident that this
    rework will address these issues.

4.  Which versions of CSPI and I-Tech device driver are addressed by this work?

5.  How will the DLS DSC daemon be supported in the future?  Will it be
    integrated into the I-Tech distribution?  Similarly, should the DLS clock
    daemon and health daemons be considered for integration?
