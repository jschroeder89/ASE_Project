################################################################################
# AMiRo-BLT is the bootloader and flashing toolchain for the base version of   #
# the Autonomous Mini Robot (AMiRo). It is based on OpenBLT developed by       #
# Feaser (see <http://feaser.com/en/openblt.php>).                             #
# Copyright (C) 2016..2018  Thomas Schöpping et al.                            #
#                                                                              #
# This program is free software: you can redistribute it and/or modify         #
# it under the terms of the GNU General Public License as published by         #
# the Free Software Foundation, either version 3 of the License, or            #
# (at your option) any later version.                                          #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                 #
# GNU General Public License for more details.                                 #
#                                                                              #
# You should have received a copy of the GNU General Public License            #
# along with this program.  If not, see <http://www.gnu.org/licenses/>.        #
#                                                                              #
# This research/work was supported by the Cluster of Excellence Cognitive      #
# Interaction Technology 'CITEC' (EXC 277) at Bielefeld University, which is   #
# funded by the German Research Foundation (DFG).                              #
################################################################################



TARGET_PATH = $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
MODULES = $(patsubst $(TARGET_PATH)Modules/%/,%,$(sort $(dir $(wildcard $(TARGET_PATH)Modules/*/*))))

CLEANTRGS = $(MODULES:%=clean_%)
FLASHTRGS = $(MODULES:%=flash_%)

all: $(MODULES)

$(MODULES):
	$(MAKE) -C $(TARGET_PATH)Modules/$@/Boot/

clean: $(CLEANTRGS)

$(CLEANTRGS):
	$(MAKE) -C $(TARGET_PATH)Modules/$(@:clean_%=%)/Boot/ clean

$(FLASHTRGS): $(@:flash_%=%)
	$(MAKE) -C $(TARGET_PATH)Modules/$(@:flash_%=%)/Boot/ flash

