AMiRo-BLT is the bootloader and flashing toolchain for the base version of the
Autonomous Mini Robot (AMiRo) [1]. It is based on OpenBLT developed by Feaser
(see <http://feaser.com/en/openblt.php>).

Copyright (C) 2016..2018  Thomas Schöpping et al.
(a complete list of all authors is given below)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at
your option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

This research/work was supported by the Cluster of Excellence
Cognitive Interaction Technology 'CITEC' (EXC 277) at Bielefeld
University, which is funded by the German Research Foundation (DFG).

Authors:
 - Thomas Schöpping        <tschoepp[at]cit-ec.uni-bielefeld.de>
 - Stefan Herbrechtsmeier  <sherbrec[at]cit-ec.uni-bielefeld.de>
 - Marvin Barther

References:
 [1] S. Herbrechtsmeier, T. Korthals, T. Schopping and U. Rückert, "AMiRo: A
     modular & customizable open-source mini robot platform," 2016 20th
     International Conference on System Theory, Control and Computing (ICSTCC),
     Sinaia, 2016, pp. 687-692.



################################################################################
#                                                                              #
#        RRRRRRRR   EEEEEEEE     AAA     DDDDDDDD   MM     MM  EEEEEEEE        #
#        RR     RR  EE          AA AA    DD     DD  MMM   MMM  EE              #
#        RR     RR  EE         AA   AA   DD     DD  MMMM MMMM  EE              #
#        RRRRRRRR   EEEEEE    AA     AA  DD     DD  MM MMM MM  EEEEEE          #
#        RR   RR    EE        AAAAAAAAA  DD     DD  MM     MM  EE              #
#        RR    RR   EE        AA     AA  DD     DD  MM     MM  EE              #
#        RR     RR  EEEEEEEE  AA     AA  DDDDDDDD   MM     MM  EEEEEEEE        #
#                                                                              #
################################################################################

This file will help you to setup all required software on your system, compile
the source code, and use the tools and flash the bootloader software to the
AMiRo base modules.

================================================================================

CONTENTS:

  1  Required software
    1.1  Git
    1.2  GNU Make
    1.3  GCC
    1.4  stm32flash
    1.5  GCC ARM Embedded Toolchain
    1.6  CMake
  2  Recommended Software
  3  Compiling the source code
    3.1  host software
    3.2  target software

================================================================================



1 - REQUIRED SOFTWARE
---------------------

The only third party software required are stm32flash and GCC for ARM embedded
devices. While the latter is a compiler to build binaries that are compatible
with the microcontrollers (MCUs) of AMiRo, the former is used to write this data
to the hardware.


1.1 - Git
---------

Since all main- and subprojects are available as Git repositories, installing a
recent version of the tool is mandatory.


1.2 - GNU Make
--------------

GNU Make usually comes as preinstalled tool on Ubuntu based operating systems.
If your system is missing GNU Make, it is recommended to install it from the
standard repositories since no special requirements (e.g. features of a very
recent version) are required.


1.3 - GCC
---------

In order to build some required tools from source, GCC is required. It usually
comes as preinstalled tool on Ubuntu based operating systems. If your system is
missing GCC, it is recommended to install it from the standard repositories
since no special requirements (e.g. features of a very recent version) are
required.


1.4 - stm32flash
----------------

This tool is required to flash the bootloader binaries to the microcontrollers.
Since it is included in this project as a submodule, you can just run the setup
script in the root directory:
  >$ ./setup.sh
Follow the instructions to download the source code and compile the tool. The
resulting binary path is ./Host/Source/stm32flash/stm32flash. Other scripts that
require stm32flash will search for the binary at this location by default.

The setup script does not install the tool to your system path, though, since
this usually requires root permissions. However, stm32flash provides a Makefile
with installation capabilities. Just Follow the instructions given in the file
./Host/Source/stm32flash/INSTALL.


1.5 - GCC ARM Embedded Toolchain
--------------------------------

Various versions of the GCC for ARM embedded devices can be found at
<https://launchpad.net/gcc-arm-embedded>. For installation of the compiler
toolchain and managing of multiple versions, it is highly recommended to use the
provided setup script. Alternatively you can install the compiler manually by
following the instructions that can be found on the web page.

If you are running a 64-bit operating system, you may have to install several
32-bit libraries in order to make the compiler work. The required packages are
libc6, libstdc++6, and libncurses5. You can run the following shell commands to
install the according 32-bit versions of the packages:
  >$ sudo dpkg --add-architecture i386 && sudo apt-get update
  >$ sudo apt-get install libc6:i386 libstdc++6:i386 libncurses5:i386


1.6 - CMake
-----------

In order to build the SerialBoot host application, CMake version 2.8 or later is
required. If possible, it is recommended to instal it from the standard
repositories of your operating system.



2 - RECOMMENDED SOFTWARE
------------------------

AMiRo-BLT provides support for the QtCreator IDE. In order to setup according
projects, use the ./setup.sh script and follow the instructions. It will
automatically generate the required files and you can import the projects by
opening the .creator files with Qtcreator IDE.



3 - COMPILING THE SOURCE CODE
-----------------------------

The AMiRo-BLT project is separated into two major parts: target- and host-
related software. The former comprises the bootloaders for the three base
modules of the AMiRo. The latter is the SerialBoot tool, which can be used to
flash further binaries (e.g. a complex operating system) to the microcontrollers
without connecting to the module directly (data is passed through via CAN bus).
Since the programming connector of the lowermost AMiRo module is the only one
accessible when the robot is fully set up, this enables to update the firmware
even for other modules.


3.1 - host software
-------------------

The stm32flash tool is requried to flash bootloader binaries to the MCUs.
Instructions for builing the tool are given in chapter 1.4 of this file.

The SerialBoot tool can be built by using cmake. The according CMakeLists.txt
file can be found in the ./Host/Source/SerialBoot/ directory. To ensure
compatibility with other software (e.g. AMiRo-OS) it is higly recommended to use
the provided ./setup.sh script to build SerialBoot. In the end the binary path
should be ./Host/Source/SerialBoot/build/SerialBoot, which is the default for
any scripts and tools that use SerialBoot.


3.2 - target software
---------------------

In the ./Target/Demo/ directory there are three subfolders, one for each AMiRo
base module. Within each of these is an additional Boot/ folder
(./Target/Demo/<device>/Boot/), which contains a makefile. Each bootloader can
be compiled by executing 'make' in these directories.

In order to flash the bootloader to a microcontroller, you first have to set
full read and write permissions to the USB ports of your system. To do so, first
create a new file by executing the following command:
  >$ sudo touch /etc/udev/rules.d/50-usb-serial.rules
Open the file in a text editor of your choice (sudo required) and add the
following lines:

  # Future Technology Devices International Ltd. - TTL-232RG
  SUBSYSTEMS=="usb", ACTION=="add", KERNEL=="ttyUSB[0-9]*",
  ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666"

  # Future Technology Devices International Ltd. - FT231X
  SUBSYSTEMS=="usb", ACTION=="add", KERNEL=="ttyUSB[0-9]*",
  ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", MODE="0666"

Now connect the module you want to flash directly to your system (note that
indirect flashing is not possible for the bootloader itself) and run the command
  >$ make flash
If the procedure was not successful, the following hints might help:
  - Did your system apply the new udev rules?
    Reboot and try again!
  - Could the makefile execute the stm32flash tool?
    Reinitialize the submodule and try again!
  - Are the permissions for USB ports set correctly?
    Check the udev rules!
  - Are there any other applications using the serial connection?
    Close any other applications using the serial connection!
  - Is the AMiRo module connected to your system?
    Use the programming cable to connect the module to your system.
  - Is the AMiRo module powered up?
    Keep a charger plugged in during flashing.

ATTENTION:
Never flash a bootloader to the wrong module! Doing so might cause severe errors
and damage the robot.

================================================================================

