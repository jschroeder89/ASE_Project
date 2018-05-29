################################################################################
# AMiRo-BLT is an bootloader and toolchain designed for the Autonomous Mini    #
# Robot (AMiRo) platform.                                                      #
# Copyright (C) 2016..2018  Thomas Schöpping et al.                            #
#                                                                              #
# This program is free software: you can redistribute it and/or modify         #
# it under the terms of the GNU General Public License as published by         #
# the Free Software Foundation, either version 3 of the License, or            #
# (at your option) any later version.                                          #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
# GNU General Public License for more details.                                 #
#                                                                              #
# You should have received a copy of the GNU General Public License            #
# along with this program.  If not, see <http://www.gnu.org/licenses/>.        #
#                                                                              #
# This research/work was supported by the Cluster of Excellence Cognitive      #
# Interaction Technology 'CITEC' (EXC 277) at Bielefeld University, which is   #
# funded by the German Research Foundation (DFG).                              #
################################################################################

#!/bin/bash

################################################################################
# GENERIC FUNCTIONS                                                            #
################################################################################

### print an error message #####################################################
# Prints a error <message> to standard output.
#If variable 'LOG_FILE' is specified, the message is also appended to the given file.
#
# usage:      printError <message>
# arguments:  <message>
#                 Message string to print.
# return:     n/a
#
function printError {
  local string="ERROR:   $1"
  # if a log file is specified
  if [ -n "$LOG_FILE" ]; then
    printf "[$(date '+%Y-%m-%d %H:%M:%S')] $string" >> $LOG_FILE
  fi
  printf "$(tput setaf 1)>>> $string$(tput sgr 0)" 1>&2
}

### print a warning message ####################################################
# Prints a warning <message> to standard output.
#If variable 'LOG_FILE' is specified, the message is also appended to the given file.
#
# usage:      printMessage <message>
# arguments:  <message>
#                 Message string to print.
# return:     n/a
#
function printWarning {
  local string="WARNING: $1"
  # if a log file is specified
  if [ -n "$LOG_FILE" ]; then
    printf "[$(date '+%Y-%m-%d %H:%M:%S')] $string" >> $LOG_FILE
  fi
  printf "$(tput setaf 3)>>> $string$(tput sgr 0)"
}

### print an information message ###############################################
# Prints an information <message> to standard output.
#If variable 'LOG_FILE' is specified, the message is also appended to the given file.
#
# usage:      printInfo <message>
# arguments:  <message>
#                 Message string to print.
# return:     n/a
#
function printInfo {
  local string="INFO:    $1"
  # if a log file is specified
  if [ -n "$LOG_FILE" ]; then
    printf "[$(date '+%Y-%m-%d %H:%M:%S')] $string" >> $LOG_FILE
  fi
  printf "$(tput setaf 2)>>> $string$(tput sgr 0)"
}

### print a message to file ####################################################
# Appends a <message> to a log file, specified by the variable 'LOG_FILE'.
#
# usage       printLog <message>
# arguments:  <message>
#                 Message string to print.
# return:     n/a
#
function printLog {
  local string="LOG:     $1"
  # if a log file is specified
  if [ -n "$LOG_FILE" ]; then
    printf "[$(date '+%Y-%m-%d %H:%M:%S')] $string" >> $LOG_FILE
  fi
}

### exit the script normally ###################################################
# Prints a delimiter and exits the script normally (returns 0).
#
# usage:      quitScript
# arguments:  n/a
# return:     0
#                 No error or warning occurred.
#
function quitScript {
  printInfo "exiting $(realpath ${BASH_SOURCE[0]})\n"
  printf "\n"
  printf "######################################################################\n"
  exit 0
}

### read a user input ##########################################################
# Reads a single character user input from a set up <options> and stores it in
# a given <return> variable.
#
# usage:      readUserInput <options> <return>
# arguments:  <options>
#                 String definiing the set of valid characters.
#                 If the string is empty, the user can input any character.
#             <return>
#                 Variable to store the selected character to.
# return:     n/a
#
function readUserInput {
  local input=""
  # read user input
  while [ -z $input ] || ( [ -n "$1" ] && [[ ! $input =~ ^[$1]$ ]] ); do
    read -p "your selection: " -n 1 -e input
    if [ -z $input ] || ( [ -n "$1" ] && [[ ! $input =~ ^[$1]$ ]] ); then
      printWarning "[$input] is no valid action\n"
    fi
  done
  printLog "[$input] has been selected\n"
  eval $2="$input"
}

### check whether argument is an option ########################################
# Checks a <string> whether it is an option.
# Options are defined to either start with '--' followed by any string, or
# to start with a single '-' followed by a single character, or
# to start with a single '-' followed by a single character, a '=' and any string.
# Examples: '--option', '--option=arg', '-o', '-o=arg', '--'
#
# usage:      parseIsOption <string>
# arguments:  <string>
#                 A string to check whether it is an option.
# return:     0
#                 <string> is an option.
#             -1
#                 <string> is not an option.
#
function parseIsOption {
  if [[ "$1" =~ ^-(.$|.=.*) ]] || [[ "$1" =~ ^--.* ]]; then
    return 0
  else
    return -1
  fi
}

### set the log file ###########################################################
# Sets a specified <infile> as log file and checks whether it already exists.
# If so, the log may either be appended to the file, its content can be cleared,
# or no log is generated at all.
# The resulting path is stored in <outvar>.
#
# usage:      setLogFile [--option=<option>] [--quiet] <infile> <outvar>
# arguments:  --option=<option>
#                 Select what to do if <file> already exists.
#                 Possible values are 'a', 'c', 'r' and 'n'.
#                 - a: append (starts with a separator)
#                 - c: continue (does not insert a seperator)
#                 - r: delete and restart
#                 - n: no log
#                 If no option is secified but <file> exists, an interactive selection is provided.
#             --quiet
#                 Suppress all messages.
#             <infile>
#                 Path of the wanted log file.
#             <outvar>
#                 Variable to store the path of the log file to.
# return:     0
#                 No error or warning occurred.
#             -1
#                 Error: invalid input
#
function setLogFile {
  local filepath=""
  local option=""
  local quiet=false

  # parse arguments
  local otherargs=()
  while [ $# -gt 0 ]; do
    if ( parseIsOption $1 ); then
      case "$1" in
        -o=*|--option=*)
          option=${1#*=}; shift 1;;
        -o*|--option*)
          option="$2"; shift 2;;
        -q|--quiet)
          quiet=true; shift 1;;
        *)
          printError "invalid option: $1\n"; shift 1;;
      esac
    else
      otherargs+=("$1")
      shift 1
    fi
  done
  filepath=$(realpath ${otherargs[0]})

  # if file already exists
  if [ -e $filepath ]; then
    # if no option was specified, ask what to do
    if [ -z "$option" ]; then
      printWarning "log file $filepath already esists\n"
      local userinput=""
      printf "Select what to do:\n"
      printf "  [A] - append log\n"
      printf "  [R] - restart log (delete existing file)\n"
      printf "  [N] - no log\n"
      readUserInput "AaRrNn" userinput
      option=${userinput,,}
    fi
    # evaluate option
    case "$option" in
      a|c)
        if [ $quiet = false ]; then
          printInfo "appending log to $filepath\n"
        fi
        if [ $option != c ]; then
          printf "\n" >> $filepath
          printf "######################################################################\n" >> $filepath
          printf "\n" >> $filepath
        fi
        ;;
      r)
        echo -n "" > $filepath
        if [ $quiet = false ]; then
          printInfo "content of $filepath wiped\n"
        fi
        ;;
      n)
        if [ $quiet = false ]; then
          printInfo "no log file will be generated\n"
        fi
        filepath=""
        ;;
      *) # sanity check (return error)
        printError "unexpected argument: $option\n"; return -1;;
    esac
  else
    if [ $quiet = false ]; then
      printInfo "log file set to $filepath\n"
    fi
  fi

  eval ${otherargs[1]}="$filepath"

  return 0
}

################################################################################
# SPECIFIC FUNCTIONS                                                           #
################################################################################

### print welcome text #########################################################
# Prints a welcome message to standard out.
#
# usage:      printWelcomeText
# arguments:  n/a
# return:     n/a
#
function printWelcomeText {
  printf "######################################################################\n"
  printf "#                                                                    #\n"
  printf "#              Welcome to the AMiRo-BLT compiler setup!              #\n"
  printf "#                                                                    #\n"
  printf "######################################################################\n"
  printf "#                                                                    #\n"
  printf "# Copyright (c) 2016..2018  Thomas Schöpping                         #\n"
  printf "#                                                                    #\n"
  printf "# This is free software; see the source for copying conditions.      #\n"
  printf "# There is NO warranty; not even for MERCHANTABILITY or FITNESS FOR  #\n"
  printf "# A PARTICULAR PURPOSE. The development of this software was         #\n"
  printf "# supported by the Excellence Cluster EXC 227 Cognitive Interaction  #\n"
  printf "# Technology. The Excellence Cluster EXC 227 is a grant of the       #\n"
  printf "# Deutsche Forschungsgemeinschaft (DFG) in the context of the German #\n"
  printf "# Excellence Initiative.                                             #\n"
  printf "#                                                                    #\n"
  printf "######################################################################\n"
}

### print help #################################################################
# Prints a help text to standard out.
#
# usage:      printHelp
# arguments:  n/a
# return:     n/a
#
function printHelp {
  printInfo "printing help text\n"
  printf "usage:    $(basename ${BASH_SOURCE[0]}) [-h|--help] [--GCC] [-q|--quit] [--log=<file>]\n"
  printf "\n"
  printf "options:  -h, --help\n"
  printf "              Print this help text.\n"
  printf "          --GCC\n"
  printf "              Enter GCC setup.\n"
  printf "          -q, --quit\n"
  printf "              Quit the script.\n"
  printf "          --log=<file>\n"
  printf "              Specify a log file.\n"
}

### enter GCC setup ############################################################
# Enter the arm-none-eabi-gcc setup.
#
# usage:      gccSetup
# arguments:  n/a
# return:     n/a
function gccSetup {
  printInfo "entering GCC setup...\n"
  printf "\n"
  if [ -z "$LOG_FILE" ]; then
    $(realpath $(dirname ${BASH_SOURCE[0]}))/GCC/gccsetup.sh --noinfo
  else
    $(realpath $(dirname ${BASH_SOURCE[0]}))/GCC/gccsetup.sh --LOG="$LOG_FILE" --noinfo
  fi
}

### main function of this script ###############################################
# The IDE setup lets the user select an IDE of choice.
# As of now, only QtCreator is supported.
#
# usage:      see function printHelp
# arguments:  see function printHelp
# return:     0
#                 No error or warning occurred.
#
function main {
  # print welcome/info text if not suppressed
  if [[ $@ != *"--noinfo"* ]]; then
    printWelcomeText
  else
    printf "######################################################################\n"
  fi
  printf "\n"

  # if --help or -h was specified, print the help text and exit
  if [[ $@ == *"--help"* || $@ == *"-h"* ]]; then
    printHelp
    printf "\n"
    quitScript
  fi

  # set log file if specified
  if [[ $@ == *"--log"* ]] || [[ $@ == *"--LOG"* ]]; then
    # get the parameter (file name)
    local cmdidx=1
    while [[ ! "${!cmdidx}" = "--log"* ]] && [[ ! "${!cmdidx}" = "--LOG"* ]]; do
      cmdidx=$[cmdidx + 1]
    done
    local cmd="${!cmdidx}"
    local logfile=""
    if [[ "$cmd" = "--log="* ]] || [[ "$cmd" = "--LOG="* ]]; then
      logfile=${cmd#*=}
    else
      local filenameidx=$((cmdidx + 1))
      logfile="${!filenameidx}"
    fi
    # optionally force silent appending
    if [[ "$cmd" = "--LOG"* ]]; then
      setLogFile --option=c --quiet "$logfile" LOG_FILE
    else
      setLogFile "$logfile" LOG_FILE
      printf "\n"
    fi
  fi
  # log script name
  printLog "this is $(realpath ${BASH_SOURCE[0]})\n"

  # parse arguments
  local otherargs=()
  while [ $# -gt 0 ]; do
    if ( parseIsOption $1 ); then
      case "$1" in
        -h|--help) # already handled; ignore
          shift 1;;
        --GCC)
          gccSetup; printf "\n"; shift 1;;
        -q|--quit)
          quitScript; printf "\n"; shift 1;;
        --log=*|--LOG=*) # already handled; ignore
          shift 1;;
        --log|--LOG) # already handled; ignore
          shift 2;;
        --noinfo) # already handled; ignore
          shift 1;;
        *)
          printError "invalid option: $1\n"; shift 1;;
      esac
    else
      otherargs+=("$1")
      shift 1
    fi
  done

  # interactive menu
  while ( true ); do
    # main menu info prompt and selection
    printInfo "AMiRo-BLT compiler setup main menu\n"
    printf "Please select one of the following actions:\n"
    printf "  [G] - enter GCC setup\n"
    printf "  [Q] - quit this setup\n"
    local userinput=""
    readUserInput "GgQq" userinput
    printf "\n"

    # evaluate user selection
    case "$userinput" in
      G|g)
        gccSetup; printf "\n";;
      Q|q)
        quitScript;;
      *) # sanity check (exit with error)
        printError "unexpected argument: $userinput\n"; printf "\n";;
    esac
  done

  exit 0
}

################################################################################
# SCRIPT ENTRY POINT                                                           #
################################################################################

main "$@"
