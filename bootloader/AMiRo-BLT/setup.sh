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
  printf "#                  Welcome to the AMiRo-BLT setup!                   #\n"
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
  printf "usage:    $(basename ${BASH_SOURCE[0]}) [-h|--help] [-f|--stm32flash] [-s|--SerialBoot] [-e|--IDE] [-c|--compiler] [-q|--quit] [--log=<file>]\n"
  printf "\n"
  printf "options:  -h, --help\n"
  printf "              Print this help text.\n"
  printf "          -f, --stm32flash\n"
  printf "              Run st,32flash tool setup.\n"
  printf "          -s, --SerialBoot\n"
  printf "              Run SerialBoot tool setup.\n"
  printf "          -e, --IDE\n"
  printf "              Enter IDE setup.\n"
  printf "          -c, --compiler\n"
  printf "              Enter compiler setup.\n"
  printf "          -q, --quit\n"
  printf "              Quit the script.\n"
  printf "          --log=<file>\n"
  printf "              Specify a log file.\n"
}

### stm32flash tool setup ######################################################
# Fetches the source code for the stm32flash tool and builds the binary.
# If the tool was already initialized, it can be wiped and rebuilt.
#
# usage:      stm32flashSetup
# arguments:  n/a
# return:     0
#                 No error or warning occurred.
#             1
#                 Warning: Setup aborted by user.
#             -1
#                 Error: Unexpected user input.
#
function stm32flashSetup {
  local amirobltdir=$(dirname $(realpath ${BASH_SOURCE[0]}))
  local stm32flashdir=${amirobltdir}/Host/Source/stm32flash
  local userdir=$(pwd)

  # if the stm32flash folder is not empty
  if [ ! -z "$(ls -A $stm32flashdir)" ]; then
    printWarning "$stm32flashdir is not empty. Delete and reinitialize? [y/n]\n"
    local userinput=""
    readUserInput "YyNn" userinput
    case "$userinput" in
      Y|y)
        printInfo "wiping ${stm32flashdir}...\n"
        # checkout base commit and delete all local branches
        cd $amirobltdir
        git submodule update --force --checkout $stm32flashdir | tee -a $LOG_FILE
        cd $stm32flashdir
        local git_branches=($(git for-each-ref --format="%(refname)"))
        for branch in $git_branches ; do
          if [[ $branch = *"heads/"* ]]; then
            git branch -D ${branch##*/} | tee -a $LOG_FILE
          fi
        done
        cd $amirobltdir
        # deinit stm32flash submodule and delete any remaining files
        git submodule deinit -f $stm32flashdir 2>&1 | tee -a $LOG_FILE
        rm -rf $stm32flashdir/*
        cd $userdir
        ;;
      N|n)
        printWarning "stm32flash setup aborted by user\n"
        return 1
        ;;
      *) # sanity check (return error)
        printError "unexpected input: $userinput\n";
        return -1
        ;;
    esac
  fi

  # initialize submodule
  printInfo "initializing stm32flash submodule...\n"
  cd $amirobltdir
  git submodule update --init $stm32flashdir 2>&1 | tee -a $LOG_FILE
  while [ ${PIPESTATUS[0]} -ne 0 ]; do
    printWarning "initialitaion failed. Retry? [y/n]\n"
    local userinput=""
    readUserInput "YyNn" userinput
    case "$userinput" in
      Y|y)
        git submodule update --init $stm32flashdir 2>&1 | tee -a $LOG_FILE;;
      N|n)
        printWarning "stm32flash initialization aborted by user\n"
        cd $userdir
        return 1
        ;;
      *) # sanity check (return error)
        printError "unexpected input: $userinput\n"; return -1;;
    esac
  done
  cd $userdir

  # build the stm32flash tool
  printInfo "compiling stm32flash\n"
  cd $stm32flashdir
  make 2>&1 | tee -a $LOG_FILE
  cd $userdir

  return 0
}

### SerialBoot tool setup ######################################################
# Builds the SerialBoot tool.
# If the tool was built before, it can be deleted and rebuilt.
#
# usage:      serialBootSetup
# arguments:  n/a
# return:     0
#                 No errort or warning occurred.
#             1
#                 Warning: Setup aborted by user.
#             -1
#                 Error: Unexpected user input.
#
function serialBootSetup {
  local amirobltdir=$(dirname $(realpath ${BASH_SOURCE[0]}))
  local serialbootdir=${amirobltdir}/Host/Source/SerialBoot
  local userdir=$(pwd)

  # if a build folder already exists
  if [ -d "${serialbootdir}/build/" ]; then
    printWarning "SerialBoot has been built before. Delete and rebuild? [y/n]\n"
    local userinput=""
    readUserInput "YyNn" userinput
    case "$userinput" in
      Y|y)
        printInfo "deleting ${serialbootdir}build/...\n"
        rm -rf "${serialbootdir}build/"
        ;;
      N|n)
        printWarning "SerialBoot setup aborted by user\n"
        return 1
        ;;
      *) # sanity check (return error)
        printError "unexpected input: $userinput\n";
        return -1
        ;;
    esac
  fi

  # build SerialBoot
  printInfo "compiling SerialBoot...\n"
  mkdir ${serialbootdir}/build
  cd ${serialbootdir}/build
  cmake .. 2>&1 | tee -a $LOG_FILE
  make 2>&1 | tee -a $LOG_FILE
  cd $userdir

  return 0
}

### IDE setup ##################################################################
# Enter the IDE setup.
#
# usage:      ideSetup
# arguments:  n/a
# return:     n/a
#
function ideSetup {
  printInfo "entering IDE setup\n"
  printf "\n"
  if [ -z "$LOG_FILE" ]; then
    $(dirname $(realpath ${BASH_SOURCE[0]}))/tools/ide/idesetup.sh --noinfo
  else
    $(dirname $(realpath ${BASH_SOURCE[0]}))/tools/ide/idesetup.sh --LOG="$LOG_FILE" --noinfo
  fi
}

### compiler setup #############################################################
# Enter the compiler setup.
#
# usage:      compilerSetup
# arguments:  n/a
# return:     n/a
#
function compilerSetup {
  printInfo "entering IDE setup\n"
  printf "\n"
  if [ -z "$LOG_FILE" ]; then
    $(dirname $(realpath ${BASH_SOURCE[0]}))/tools/compiler/compilersetup.sh --noinfo
  else
    $(dirname $(realpath ${BASH_SOURCE[0]}))/tools/compiler/compilersetup.sh --LOG="$LOG_FILE" --noinfo
  fi
}

### main function of this script ###############################################
# A setup to initialize dependencies, setup IDE projects and configure the
# compiler setup.
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
        -f|--stm32flash)
          stm32flashSetup; printf "\n"; shift 1;;
        -s|--SerialBoot)
          serialBootSetup; printf "\n"; shift 1;;
        -e|--IDE)
          ideSetup; printf "\n"; shift 1;;
        -c|--compiler)
          compilerSetup; printf "\n"; shift 1;;
        -q|--quit)
          quitScript; shift 1;;
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
    printInfo "AMiRo-BLT setup main menu\n"
    printf "Please select one of the following actions:\n"
    printf "  [F] - get and build stm32flash tool\n"
    printf "  [S] - build SerialBoot tool\n"
    printf "  [E] - IDE project setup\n"
    printf "  [C] - enter compiler setup\n"
    printf "  [Q] - quit this setup\n"
    local userinput=""
    readUserInput "FfSsEeCcQq" userinput
    printf "\n"

    # evaluate user selection
    case "$userinput" in
      F|f)
        stm32flashSetup; printf "\n";;
      S|s)
        serialBootSetup; printf "\n";;
      E|e)
        ideSetup; printf "\n";;
      C|c)
        compilerSetup; printf "\n";;
      Q|q)
        quitScript;;
      *) # sanity check (exit with error)
        printError "unexpected argument: $userinput\n";;
    esac
  done

  exit 0
}

################################################################################
# SCRIPT ENTRY POINT                                                           #
################################################################################

main "$@"

