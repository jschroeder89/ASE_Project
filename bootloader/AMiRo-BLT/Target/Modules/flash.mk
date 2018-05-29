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



STM32FLASH := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))../../Host/Source/stm32flash/stm32flash
ifeq ($(OS),Windows_NT)
	STM32FLASH_PORT ?= COM5
else
	STM32FLASH_PORT ?= /dev/ttyUSB0
endif
BAUDRATE = 115200
BINARY = $(BIN_PATH)/$(PROJ_NAME).hex
GPIO_ENTER_SEQUENCE = "rts,-dtr,-rts"
GPIO_EXIT_SEQUENCE = "rts,dtr,-rts"

flash: $(BINARY)
	@echo +++ Flashing
	$(STM32FLASH) $(STM32FLASH_PORT) -i $(GPIO_ENTER_SEQUENCE):$(GPIO_EXIT_SEQUENCE) -R -b $(BAUDRATE) -w $(BINARY)

