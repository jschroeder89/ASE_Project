AMiRo-LLD is a compilation of low-level hardware drivers for the base version of
the Autonomous Mini Robot (AMiRo) [1]. It provides directional interfaces for an
operating system to access the drivers and for the drivers to access the
communication infrastructure via the operating system.

Copyright (C) 2016..2018  Thomas Schöpping et al.
(a complete list of all authors is given below)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at
your option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

This research/work was supported by the Cluster of Excellence
Cognitive Interaction Technology 'CITEC' (EXC 277) at Bielefeld
University, which is funded by the German Research Foundation (DFG).

Authors:
 - Thomas Schöpping          <tschoepp[at]cit-ec.uni-bielefeld.de>
 - Marc Rothmann

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

AMiRo-LLD is a compilation of low-level hardware drivers, originally developed
for the Autonomous Mini Robot (AMiRo) [1]. It provides a modular design, so that
each driver can be (de)activated individually as required. Interface functions
allow for bidirectional comunication with an operating system. On the one hand
drivers access according hardware interfaces via defined interface functions
(which need to be implemented by the operating system) and any applications (or
the operating system itself) can take advantage of the drivers by their
individual interfaces. The abstraction layer of the hardware interfaces is
called "periphAL", which is defined by this project.

Although this compilation was originally designed to be used in combination with
the AMiRo operating system (AMiRo-OS; cf. https://opensource.cit-ec.de/projects/amiro-os/),
it is not limited to this use case. The included drivers may be used for any
purpose and contributions of further drivers, even if the according hardware is
not present on the AMiRo platform, are highly appreciated.

The files are structured as follows:
* include/
  Each driver defines exactly one include header in this directory. By
  convention these files are named by the form
    alld_<product_name>.h
  <product_name> is a placeholder for the exact name of the according hardware,
  or the product familiy, if the driver is compatible with all parts.
* source/
  Any source files are placed in this directory. Usually there is a single file
  for each driver. If multiple files are necessary, these should be placed in an
  accordingly named subfolder. Names of folders and files should comply to the
  aforementioned convention.
* templates/
  AMiRo-LLD requires an implementation of the defined interface and an
  configuration header to be accessible in the include paths at compile time.
  Template files for both can be found in this folder. It is recommended to
  place according implementations of these templated not in the AMiRo-LLD
  project, but the superproject which includes AMiRo-LLD.
* /
  The project root directory contains this file, a license.html file as well as
  a Makefile that allows to easily integrate the project. Furthermore, two
  interface headers are provided: amiro-lld.h and periphALtypes.h. These are
  entry points for any utilizing superproject, so it is not required (and not
  recommanded) to include each driver individually.

================================================================================

