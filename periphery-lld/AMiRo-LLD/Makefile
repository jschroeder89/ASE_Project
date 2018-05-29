################################################################################
# AMiRo-LLD is a compilation of low-level hardware drivers for the Autonomous  #
# Mini Robot (AMiRo) platform.                                                 #
# Copyright (C) 2016..2018  Thomas Schöpping et al.                            #
#                                                                              #
# This program is free software: you can redistribute it and/or modify         #
# it under the terms of the GNU Lesser General Public License as published by  #
# the Free Software Foundation, either version 3 of the License, or            #
# (at your option) any later version.                                          #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
# GNU Lesser General Public License for more details.                          #
#                                                                              #
# You should have received a copy of the GNU Lesser General Public License     #
# along with this program.  If not, see <http://www.gnu.org/licenses/>.        #
#                                                                              #
# This research/work was supported by the Cluster of Excellence Cognitive      #
# Interaction Technology 'CITEC' (EXC 277) at Bielefeld University, which is   #
# funded by the German Research Foundation (DFG).                              #
################################################################################



# absolute path to this directory
AMIROLLD_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

# include paths
AMIROLLD_INC = $(AMIROLLD_DIR) \
               $(AMIROLLD_DIR)include/

AMIROLLD_SRCDIR = $(AMIROLLD_DIR)source/

# C sources
AMIROLLD_CSRC = $(AMIROLLD_SRCDIR)alld_a3906.c \
                $(AMIROLLD_SRCDIR)alld_at24c01bn-sh-b.c \
                $(AMIROLLD_SRCDIR)alld_bq24103a.c \
                $(AMIROLLD_SRCDIR)alld_bq27500.c \
                $(AMIROLLD_SRCDIR)alld_hmc5883l.c \
                $(AMIROLLD_SRCDIR)alld_ina219.c \
                $(AMIROLLD_SRCDIR)alld_l3g4200d.c \
                $(AMIROLLD_SRCDIR)alld_led.c \
                $(AMIROLLD_SRCDIR)alld_lis331dlh.c \
                $(AMIROLLD_SRCDIR)alld_ltc4412.c \
                $(AMIROLLD_SRCDIR)alld_mpr121.c \
                $(AMIROLLD_SRCDIR)alld_pca9544a.c \
                $(AMIROLLD_SRCDIR)alld_pklcs1212e4001.c \
                $(AMIROLLD_SRCDIR)alld_tlc5947.c \
                $(AMIROLLD_SRCDIR)alld_tps2051bdbv.c \
                $(AMIROLLD_SRCDIR)alld_tps62113.c \
                $(AMIROLLD_SRCDIR)alld_vcnl4020.c

