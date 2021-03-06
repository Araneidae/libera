Building and Releasing Libera
=============================

.. This file is written in reStructuredText
.. default-role:: literal

This document describes the internal process used within the Diamond Controls
group to manage the development and deployment of the Libera EPICS driver, and
is linked to directly from the Controls "Application Development Environment"
(ADE) document.

The following ADE standard file system locations are used to manage Libera:

`$SVN_ROOT/diamond/trunk/ioc/Libera`
    Current development version of Libera EPICS driver.

`$SVN_ROOT/diamond/release/ioc/Libera`
    Versioned releases of Libera EPICS driver.  Note that since around version
    1.46 the Libere EPICS driver version numbers have tended to shadow the
    corresponding Instrumentation Technology releases, but with our own extra
    sub-version numbering.

`/dls_sw/prod/R3.14.9/ioc/Libera`, `/dls_sw/prod/R3.14.11/ioc/Libera`
    Currently all official releases of the Libera IOC are under these two
    directories.  Releases up to and including 2.00.0 are stored under
    `R3.14.9`, releases 2.05 and onwards under `R3.14.11`.

    The directory path to the current release,
    `/dls_sw/prod/R3.14.11/ioc/Libera/$VERSION`, where `$VERSION` identifies the
    currently install release (eg `2.05.3`), should be configured in
    `configure-ioc` against the key `DI-EBPM-gui-dir`.  This allows the
    `diagOpi` EDM screens to find the correct Libera screens.

`/dls_sw/cs-publish/libera`
    Publicly available released of Libera are made available through the
    external web site http://controls.diamond.ac.uk/downloads/libera/index.html.
    The content of this web site is managed through this directory, as
    documented below.

In the Libera source tree there is detailed documentation in the `docs`
directory, including the following:

`INSTALL.txt`
    Installation instructions for binary distributions downloaded from the
    external web site.

`BUILD.txt`
    Detailed build instructions, again designed for external users of the EPICS
    driver, but the build process is documented here.

`libera.`\{`txt`, `html`}
    Detailed operational documentation of the EPICS interface to Libera.

`CHANGES.`\{`txt`, `html`}
    Log of changes to the EPICS driver.  Designed to provide an overview of the
    changes so that external users can get an understanding of the differences
    between releases.

`process.`\{`txt`, `html`}
    This document.


Releasing the Libera EPICS driver
---------------------------------

To create a new release of the Libera EPICS driver the following process should
be followed:

1.  Check out the current sources from subversion.

2.  Modify the `VERSION` line of `install_d/CONFIG` during development.  The
    version number should be advanced, and the suffix `-dev` added until the
    final release can be made.

3.  Update the Libera EPICS driver, test it using the resources in the lab,
    possibly test it using test installations (described below) on one or more
    Liberas in the synchrotron, ensure the final version of the sources is
    commited to SVN.

4.  Ensure the documentation in `libera.txt` and `CHANGES.txt` is up to date,
    remove the `-dev` suffix from the `VERSION` string, and commit the final
    release to trunk.

5.  To create a production release the script `install_d/publish-release` should
    be used.  Give it one argument, the version string for the new release, and
    this script will automatically create a versioned release under subversion
    and build it under `/dls_sw/prod/R3.14.11/ioc/Libera`.

6.  The newly released driver can then be installed on all Liberas in the
    storage ring using the installation procedure documented below.

7.  Finally the IOC redirector needs to be reconfigured for the
    `DI-EBPM-gui-dir` key so that fresh launches of the Diagnostics screens will
    pick up the new Libera screens.  This is done by running the following
    command (where `$VERSION` is the new driver version string)::

        configure-ioc e -f DI-EBPM-gui-dir /dls_sw/prod/R3.14.11/ioc/Libera/$VERSION


Publishing the Libera EPICS driver
----------------------------------

After creating a new release of the Libera IOC it should be released to the
public web site.  This requires the following steps.

1.  Prepare a binary release by running the following command::

        install_d/distribute -b $VERSION /tmp

    This will create two files `libera-epics-$VERSION.tar.gz` and
    `libera-epics-$VERSION-install.tar.gz` in `/tmp`.  The first is the source
    distribution, the second the binary installation.

2.  Place the source distribution in `/dls_sw/cs-publish/libera/src/` and the
    binary install in `/dls_sw/cs-publish/libera/install/`.

3.  Run `make -C docs` in the current Libera sources and copy the generated
    files `CHANGES.html` and `libera.html` from `docs` to
    `/dls_sw/cs-publish/libera`.

4.  Edit the index file `/dls_sw/cs-publish/libera/index.html` to add a new row
    for the newly created release.

    The new release is now live.


Installing and Managing Libera
------------------------------

The processes for upgrading the Libera EPICS IOC in the lab and in the
synchrotron are somewhat different.  Typically only a single machine is managed
in the lab, though a script `lab-liberas` is provided for managing groups of lab
machines.  Upgrades on the synchrotron are a more delicate process, as a
guaranteed upgrade is mandatory!


Testing the Libera EPICS IOC in the Lab
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When a build from sources is complete, the command `install_d/install-ioc`
(without any arguments) will place a copy of the installation binaries (and all
associated files) in `/home/libera/nfs/testing/ioc`.

To connect to a properly configured lab machine the command (where `$NN` is a
two digit number, typically in the range `01` to `07`) ::

    ssh -x root@TS-DI-EBPM-$NN

will serve.  The root password is not documented here, but it is well known, and
ssh certificates can be set up to facilitate connection.  The `-x` option
prevents bogus error messages of the form `sh: /usr/X11R6/bin/xauth: not found`.

Once connected to a test machine, if the EPICS driver is already configured as a
test install, the newly built driver can be started by running the command
`/etc/init.d/epics restart` or `/etc/init.d/libera-driver restart` if a fuller
restart is required.

Alternatively, the interactive EPICS IOC shell can be started by running the
following commands::

    /etc/init.d/epics stop
    /mnt/nfs/testing/ioc/runioc

If the EPICS driver is not installed, if a production install is running, or if
locally installed files have changed, then the EPICS driver can be installed as
a test install by running the following command (the `-s` option also forces the
driver to start after installation is complete, `-h` can be used for a list of
options)::

    /mnt/nfs/testing/ioc/install_d/libera-install-ioc -tasw SR

The installation, whether test (`-t`) or final (`-f`), updates the configuration
file `/etc/libera/epics_ioc` (which can be found in `/etc/default` or
`/etc/sysconfig` on older versions of Libera), places the scripts `epics`,
`healthd`, `libera-driver` in `/etc/init.d` and manages autostart links in
`/etc/rc.d`, and a `-f` install places the EPICS driver and all its files in
`/opt/ioc`.


Installing the Libera EPICS driver on the Synchrotron
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Upgrading the Libera EPICS IOC on the synchrotron involves three steps: prepare
the files to be installed, ensure the NFS mounts are working correctly, install
the updated drivers.

The operations below use the command `liberas` which can be found in the
`toolkit` directory of the Libera source tree.

1.  Preparing the files to be installed

    Preparing the files to be installed is straightforward.  If a properly
    released production install is to be used then after a successful build all
    the required files will already be present in
    `/dls_sw/prod/R3.14.11/ioc/Libera/$VERSION`.  If on the other hand a test
    install is required then the command ::

        install_d/install-ioc /dls_sw/work/R3.14.11/ioc/Libera

    should be run in the build source directory tree.


2.  Ensuring the NFS mounts are working correctly

    This step is vital for a smoothly running install, as there is no guarantee
    that Libera can currently see the NFS mount points needed for the
    installation.  The following procedure will ensure that the upgrade will
    work smoothly.

    * First check that all liberas in the synchrotron are responding by running
      the command ::

        liberas '' hostname | wc -l

      At the time of writing this should return the number 205 after not too
      long a delay.  If not this must be investigated and sorted out before
      proceeding further.

    * Next check that the NFS mounts are present.  The following command will
      print out the name of all liberas where the `/dls_sw/prod` mount is
      missing::

        liberas -w '' '[ -d /mnt/prod/R3.14.11 ] || hostname'

      If this reports nothing (and returns to the prompt) then all is well and
      the installation can be done.  Otherwise, the NFS mounts must be refreshed
      on all affected Liberas by taking the next step.

      Typically the NFS mounts will be missing in a group of Liberas which were
      restarted while the NFS server was unavailable or unreachable, frequently
      because the network rack in that CIA wasn't yet turn on.

    * Recovering the NFS mounts.  This should be simply a matter of running the
      command ::

        /etc/init.d/mount-extra start

      on each affected Libera and re-running the mount test command above.

3.  Performing the upgrade

    This step is straightforward once the preparation above has been done.
    Simply run the following two commands, where `$INSTALL` is either
    `/mnt/prod/R3.14.11/ioc/Libera/$VERSION` or
    `/mnt/work/R3.14.11/ioc/Libera/ioc`::

        liberas -w 'LB|BR|BS' "$INSTALL/install_d/libera-install-ioc -fasw BR"
        liberas -w SR "$INSTALL/install_d/libera-install-ioc -fasw SR"

    Note that the injector (Linac to Booster, Booster, Booster to Storage) needs
    to be installed separately from the Storage ring as the Libera
    configurations differ.


Moving Liberas between the Lab and the Synchrotron Building
-----------------------------------------------------------

The DLS rootfs provides a simple script for moving the network address and
renaming the Libera.  This can be run from ssh before disconnecting from the old
network, or can be run over the serial port at any time.  To move a machine to
location `$device` run the command::

    /opt/bin/configure-network -w $device

The `-r` flag can be added to force an immediate network restart, otherwise the
original network settings will remain in force until the machine is restarted.
This command has other options which can be shown with the `-h` help option.


Moving Libera from Synchrotron Building to Lab
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To move a machine into the lab, assign it a test network number in the range 1
to 99, after first checking that the number is not already in use.  Then run the
`configure-network` script above with `$device=TS-DI-EBPM-$NN`, where `$NN`
should be the device number as a two digits.

The addresses `TS-DI-EBPM-01` to `TS-DI-EBPM-07` are assigned positions in the
test rack.


Moving Libera from Lab to Synchrotron Building
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Here it is very important to determine the state of the machine being moved from
the lab to the synchrotron building.  The following steps must be followed.

1.  First ensure that the machine to be moved is fit for service on the
    synchrotron.  Determining this is beyond the scope of this document, and it
    depends on the state of Diagnostics records and the history of the machine
    concerned.

2.  Next ensure that the machine is running an up to date version of the DLS
    rootfs distribution.  Running the command ::

        /mnt/nfs/upgrade-DLS/rootfs/upgrade-libera

    on the target machine should ensure this, but ideally this should already be
    determined by the records established at step (1).  Note that this command
    can be safely run on any version of Libera.

3.  Ensure that the version of the EPICS driver running on the target machine is
    the version required to be run on the machine, upgrading if appropriate.

4.  If at all possible capture a copy of the state file from the machine being
    replaced.  If a new Libera is being installed, or if the machine being
    replaced has failed completely, it is advisable to copy a state file from a
    machine which is as similar as possible: for example, when replacing a
    primary EBPM Libera copy the state file from another primary EBPM.

    The state file can be found at `/opt/state/$device.state` where `$device` is
    the IOC name of the machine.  This should be copied into the replacement
    machine.

5.  Switch the network address by running the `configure-network` script with
    the selected device.

6.  After replacing a Libera it should be disabled (by setting
    `$device:CF:ENABLED_S` to `"BPM disabled"` ) until beam based alignment
    (BBA) has been run.
