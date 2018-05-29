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
  printf "#                  Welcome to the QtCreator setup!                   #\n"
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
  printf "usage:    $(basename ${BASH_SOURCE[0]}) [-h|--help] [-c|--clean] [-w|--wipe] [--LightRing] [--PowerManagement] [--DiWheelDrive] [-a|--all] [-q|--quit] [--log=<file>]\n"
  printf "\n"
  printf "options:  -h, --help\n"
  printf "              Print this help text.\n"
  printf "          -c, --clean\n"
  printf "              Delete project files.\n"
  printf "          -w, --wipe\n"
  printf "              Delete project and .user files.\n"
  printf "          --LightRing\n"
  printf "              Create project for the LightRing module.\n"
  printf "          --PowerManagement\n"
  printf "              Create project for the PowerManagement module.\n"
  printf "          --DiWheelDrive\n"
  printf "              Create project for the DiWheelDrive module.\n"
  printf "          -a, --all\n"
  printf "              Create projects for all modules.\n"
  printf "          -q, --quit\n"
  printf "              Quit the script.\n"
  printf "          --log=<file>\n"
  printf "              Specify a log file.\n"
}

### read directory where to create/delete projects #############################
# Read the directory where to create/delete project files from user.
#
# usage:      getProjectDir <pathvar>
# arguments:  <pathvar>
#                 Variable to store the selected path to.
# return:     n/a
#
function getProjectDir {
  printLog "reading path for project files from user...\n"
  local amirobltdir=$(realpath $(dirname $(realpath ${BASH_SOURCE[0]}))/../../../Target/)
  local input=""
  read -p "Path where to create/delete project files: " -i $amirobltdir -e input
  printLog "user selected path $(realpath $input)\n"
  eval $1="$(realpath $input)"
}

### retrieves the ARM-NONE-EABI-GCC include directory ##########################
# Retrieves the include directory of the currently set arm-none-eabi-gcc.
#
# usage:      retrieveGccIncludeDir <path>
# arguments:  <path>
#                 Variable to store the path to.
# return:    0
#                 No error or warning occurred.
#            -1
#                 Error: Command 'arm-none-eabi-gcc' not found.
#
function retrieveGccIncludeDir {
  # retrieve binary path or link
  local binpath=$(which arm-none-eabi-gcc)
  if [ -z "$binpath" ]; then
    printError "command 'arm-none-eabi-gcc' not found\n"
    return -1
  else 

    # traverse any links
    while [ -L "$binpath" ]; do
      binpath=$(readlink $binpath)
    done
    printInfo "gcc-arm-none-eabi detected: $binpath\n"

    # return include path
    eval $1=$(realpath $(dirname ${binpath})/../arm-none-eabi/include/)

    return 0
  fi
}

### delete project files #######################################################
# Deletes all project files and optionally .user files, too.
#
# usage:      deleteProjects [-p|--path=<path>] [-o|--out=<var>] [-w|-wipe]
# arguments:  -p, --path <path>
#                 Path where to delete the project files.
#             -o, --out <var>
#                 Variable to store the path to.
#             -w, --wipe
#                 Delete .user files as well.
# return:
#  -  0: no error
#  -  1: warning: function aborted by user
#  - -1: error: unexpected user input
function deleteProjects {
  local projectdir=""
  local outvar=""
  local wipe=false

  # parse arguments
  local otherargs=()
  while [ $# -gt 0 ]; do
    if ( parseIsOption $1 ); then
      case "$1" in
        -p=*|--path=*)
          projectdir=$(realpath "${1#*=}"); shift 1;;
        -p|--path)
          projectdir=$(realpath "$2"); shift 2;;
        -o=*|--out=*)
          outvar=${1#*=}; shift 1;;
        -o|--out)
          outvar=$2; shift 2;;
        -w|--wipe)
          wipe=true; shift 1;;
        *)
          printError "invalid option: $1\n"; shift 1;;
      esac
    else
      otherargs+=("$1")
      shift 1
    fi
  done

  # print message
  if [ $wipe != true ]; then
    printInfo "deleting all QtCreator project files (*.includes, *.files, *.config, and *.creator)\n"
  else
    printInfo "deleting all QtCreator project files (*.includes, *.files, *.config, *.creator, and *.user)\n"
  fi

  # read project directory if required
  if [ -z "$projectdir" ]; then
    getProjectDir projectdir
  fi

  # remove all project files
  rm ${projectdir}/LightRing.includes 2>&1 | tee -a $LOG_FILE
  rm ${projectdir}/PowerManagement.includes 2>&1 | tee -a $LOG_FILE
  rm ${projectdir}/DiWheelDrive.includes 2>&1 | tee -a $LOG_FILE

  rm ${projectdir}/LightRing.files 2>&1 | tee -a $LOG_FILE
  rm ${projectdir}/PowerManagement.files 2>&1 | tee -a $LOG_FILE
  rm ${projectdir}/DiWheelDrive.files 2>&1 | tee -a $LOG_FILE

  rm ${projectdir}/LightRing.config 2>&1 | tee -a $LOG_FILE
  rm ${projectdir}/PowerManagement.config 2>&1 | tee -a $LOG_FILE
  rm ${projectdir}/DiWheelDrive.config 2>&1 | tee -a $LOG_FILE

  rm ${projectdir}/LightRing.creator 2>&1 | tee -a $LOG_FILE
  rm ${projectdir}/PowerManagement.creator 2>&1 | tee -a $LOG_FILE
  rm ${projectdir}/DiWheelDrive.creator 2>&1 | tee -a $LOG_FILE

  if [ $wipe == true ]; then
    rm ${projectdir}/LightRing.creator.user 2>&1 | tee -a $LOG_FILE
    rm ${projectdir}/PowerManagement.creator.user 2>&1 | tee -a $LOG_FILE
    rm ${projectdir}/DiWheelDrive.creator.user 2>&1 | tee -a $LOG_FILE
  fi

  # store the path to the output variable, if required
  if [ ! -z "$outvar" ]; then
    eval $outvar="$projectdir"
  fi

  return 0
}

### create LightRing project files #############################################
# Create project files for the LightRing module.
#
# usage:      createLightRingProject [-p|--path=<path>] [--gcc=<path>] [-o|--out=<var>] [--gccout=<var>]
# arguments:  -p, --path <path>
#                 Path where to create the project files.
#             --gcc=<path>
#                 Path to the GCC include directory.
#             -o, --out <var>
#                 Variable to store the path to.
#             --gccout=<var>
#                 Variable to store the path to the GCC include directory to.
# return:     0
#                 No error or warning occurred.
#
function createLightRingProject {
  local userdir=$(pwd)
  local projectdir=""
  local gccincludedir=""
  local outvar=""
  local gccoutvar=""

  # parse arguments
  local otherargs=()
  while [ $# -gt 0 ]; do
    if ( parseIsOption $1 ); then
      case "$1" in
        -p=*|--path=*)
          projectdir=$(realpath "${1#*=}"); shift 1;;
        -p|--path)
          projectdir=$(realpath "$2"); shift 2;;
        --gcc=*)
          gccincludedir=$(realpath "${1#*=}"); shift 1;;
        --gcc)
          gccincludedir=$(realpath "$2"); shift 2;;
        -o=*|--out=*)
          outvar=${1#*=}; shift 1;;
        -o|--out)
          outvar=$2; shift 2;;
        --gccout=*)
          gccoutvar=$(realpath "${1#*=}"); shift 1;;
        --gccout)
          gccoutvar=$(realpath "$2"); shift 2;;
        *)
          printError "invalid option: $1\n"; shift 1;;
      esac
    else
      otherargs+=("$1")
      shift 1
    fi
  done

  # print message
  printInfo "creating QtCreator project files for the LightRing module...\n"

  # read absolute project directory if required
  if [ -z "$projectdir" ]; then
    getProjectDir projectdir
  fi

  # retrieve absolute GCC include dir
  if [ -z "$gccincludedir" ]; then
    retrieveGccIncludeDir gccincludedir
  fi

  # move to project directory
  cd $projectdir

  # create project files
  # generate a file that contains all subdirectories as includes (but ignore hidden and documentation directories)
  find $gccincludedir -type d > ${projectdir}/LightRing.includes
  find $(realpath --relative-base=$projectdir $(dirname ${BASH_SOURCE[0]})/../../../Target/Source) -type d | grep -v "ARMCM4_STM32" >> ${projectdir}/LightRing.includes
  find $(realpath --relative-base=$projectdir $(dirname ${BASH_SOURCE[0]})/../../../Target/Modules/LightRing_1-0/Boot) -type d | grep -v "bin\|cmd\|ethernetlib\|fatfs\|uip\|obj" >> ${projectdir}/LightRing.includes
  # generate a file that specifies all files
  echo -n "" > ${projectdir}/LightRing.files
  for path in `cat ${projectdir}/LightRing.includes`; do
    find $path -maxdepth 1 -type f \( ! -iname ".*" \) | grep -v "/arm-none-eabi/" | grep -E ".*(\.h|\.c|\.x)$" >> ${projectdir}/LightRing.files
  done
  # generate a default project configuration file if none exists so far
  if [ ! -f ${projectdir}/LightRing.config ]; then
    echo -e "// Add predefined macros for your project here. For example:" > ${projectdir}/LightRing.config
    echo -e "// #define YOUR_CONFIGURATION belongs here" >> ${projectdir}/LightRing.config
    echo -e "" >> ${projectdir}/LightRing.config
  fi
  # generate a default .creator file if none exists so far
  if [ ! -f ${projectdir}/LightRing.creator ]; then
    echo -e "[general]" > ${projectdir}/LightRing.creator
    echo -e "" >> ${projectdir}/LightRing.creator
  fi

  # go back to user directory
  cd $userdir

  # fill the output variables
  if [ ! -z "$outvar" ]; then
    eval $outvar="$projectdir"
  fi
  if [ ! -z "$gccoutvar" ]; then
    eval $gccoutvar="$gccincludedir"
  fi

  return 0
}

### create PowerManagement project files #######################################
# Create project files for the PowerManagement module.
#
# usage:      createPowerManagementProject [-p|--path=<path>] [--gcc=<path>] [-o|--out=<var>] [--gccout=<var>]
# arguments:  -p, --path <path>
#                 Path where to create the project files.
#             --gcc=<path>
#                 Path to the GCC include directory.
#             -o, --out <var>
#                 Variable to store the path to.
#             --gccout=<var>
#                 Variable to store the path to the GCC include directory to.
# return:     0
#                 No error or warning occurred.
#
function createPowerManagementProject {
  local userdir=$(pwd)
  local projectdir=""
  local gccincludedir=""
  local outvar=""
  local gccoutvar=""

  # parse arguments
  local otherargs=()
  while [ $# -gt 0 ]; do
    if ( parseIsOption $1 ); then
      case "$1" in
        -p=*|--path=*)
          projectdir=$(realpath "${1#*=}"); shift 1;;
        -p|--path)
          projectdir=$(realpath "$2"); shift 2;;
        --gcc=*)
          gccincludedir=$(realpath "${1#*=}"); shift 1;;
        --gcc)
          gccincludedir=$(realpath "$2"); shift 2;;
        -o=*|--out=*)
          outvar=${1#*=}; shift 1;;
        -o|--out)
          outvar=$2; shift 2;;
        --gccout=*)
          gccoutvar=$(realpath "${1#*=}"); shift 1;;
        --gccout)
          gccoutvar=$(realpath "$2"); shift 2;;
        *)
          printError "invalid option: $1\n"; shift 1;;
      esac
    else
      otherargs+=("$1")
      shift 1
    fi
  done

  # print message
  printInfo "creating QtCreator project files for the PowerManagement module...\n"

  # read absolute project directory if required
  if [ -z "$projectdir" ]; then
    getProjectDir projectdir
  fi

  # retrieve absolute GCC include dir
  if [ -z "$gccincludedir" ]; then
    retrieveGccIncludeDir gccincludedir
  fi

  # move to project directory
  cd $projectdir

  # create project files
  # generate a file that contains all subdirectories as includes (but ignore hidden and documentation directories)
  find $gccincludedir -type d > ${projectdir}/PowerManagement.includes
  find $(realpath --relative-base=$projectdir $(dirname ${BASH_SOURCE[0]})/../../../Target/Source) -type d | grep -v "ARMCM3_STM32" >> ${projectdir}/PowerManagement.includes
  find $(realpath --relative-base=$projectdir $(dirname ${BASH_SOURCE[0]})/../../../Target/Modules/PowerManagement_1-1/Boot) -type d | grep -v "bin\|cmd\|ethernetlib\|fatfs\|uip\|obj" >> ${projectdir}/PowerManagement.includes
  # generate a file that specifies all files
  echo -n "" > ${projectdir}/PowerManagement.files
  for path in `cat ${projectdir}/PowerManagement.includes`; do
    find $path -maxdepth 1 -type f \( ! -iname ".*" \) | grep -v "/arm-none-eabi/" | grep -E ".*(\.h|\.c|\.x)$" >> ${projectdir}/PowerManagement.files
  done
  # generate a default project configuration file if none exists so far
  if [ ! -f ${projectdir}/PowerManagement.config ]; then
    echo -e "// Add predefined macros for your project here. For example:" > ${projectdir}/PowerManagement.config
    echo -e "// #define YOUR_CONFIGURATION belongs here" >> ${projectdir}/PowerManagement.config
    echo -e "" >> ${projectdir}/PowerManagement.config
  fi
  # generate a default .creator file if none exists so far
  if [ ! -f ${projectdir}/PowerManagement.creator ]; then
    echo -e "[general]" > ${projectdir}/PowerManagement.creator
    echo -e "" >> ${projectdir}/PowerManagement.creator
  fi

  # go back to user directory
  cd $userdir


  # fill the output variables
  if [ ! -z "$outvar" ]; then
    eval $outvar="$projectdir"
  fi
  if [ ! -z "$gccoutvar" ]; then
    eval $gccoutvar="$gccincludedir"
  fi

  return 0
}

### create DiWheelDrive project files ##########################################
# Create project files for the DiWheelDrive module.
#
# usage:      createDiWheelDriveProject [-p|--path=<path>] [--gcc=<path>] [-o|--out=<var>] [--gccout=<var>]
# arguments:  -p, --path <path>
#                 Path where to create the project files.
#             --gcc=<path>
#                 Path to the GCC include directory.
#             -o, --out <var>
#                 Variable to store the path to.
#             --gccout=<var>
#                 Variable to store the path to the GCC include directory to.
# return:     0
#                 No error or warning occurred.
#
function createDiWheelDriveProject {
  local userdir=$(pwd)
  local projectdir=""
  local gccincludedir=""
  local outvar=""
  local gccoutvar=""

  # parse arguments
  local otherargs=()
  while [ $# -gt 0 ]; do
    if ( parseIsOption $1 ); then
      case "$1" in
        -p=*|--path=*)
          projectdir=$(realpath "${1#*=}"); shift 1;;
        -p|--path)
          projectdir=$(realpath "$2"); shift 2;;
        --gcc=*)
          gccincludedir=$(realpath "${1#*=}"); shift 1;;
        --gcc)
          gccincludedir=$(realpath "$2"); shift 2;;
        -o=*|--out=*)
          outvar=${1#*=}; shift 1;;
        -o|--out)
          outvar=$2; shift 2;;
        --gccout=*)
          gccoutvar=$(realpath "${1#*=}"); shift 1;;
        --gccout)
          gccoutvar=$(realpath "$2"); shift 2;;
        *)
          printError "invalid option: $1\n"; shift 1;;
      esac
    else
      otherargs+=("$1")
      shift 1
    fi
  done

  # print message
  printInfo "creating QtCreator project files for the DiWheelDrive module...\n"

  # read absolute project directory if required
  if [ -z "$projectdir" ]; then
    getProjectDir projectdir
  fi

  # retrieve absolute GCC include dir
  if [ -z "$gccincludedir" ]; then
    retrieveGccIncludeDir gccincludedir
  fi

  # move to project directory
  cd $projectdir


  # create project files
  # generate a file that contains all subdirectories as includes (but ignore hidden and documentation directories)
  find $gccincludedir -type d > ${projectdir}/DiWheelDrive.includes
  find $(realpath --relative-base=$projectdir $(dirname ${BASH_SOURCE[0]})/../../../Target/Source) -type d | grep -v "ARMCM4_STM32" >> ${projectdir}/DiWheelDrive.includes
  find $(realpath --relative-base=$projectdir $(dirname ${BASH_SOURCE[0]})/../../../Target/Modules/DiWheelDrive_1-1/Boot) -type d | grep -v "bin\|cmd\|ethernetlib\|fatfs\|uip\|obj" >> ${projectdir}/DiWheelDrive.includes
  # generate a file that specifies all files
  echo -n "" > ${projectdir}/DiWheelDrive.files
  for path in `cat ${projectdir}/DiWheelDrive.includes`; do
    find $path -maxdepth 1 -type f \( ! -iname ".*" \) | grep -v "/arm-none-eabi/" | grep -E ".*(\.h|\.c|\.x)$" >> ${projectdir}/DiWheelDrive.files
  done
  # generate a default project configuration file if none exists so far
  if [ ! -f ${projectdir}/DiWheelDrive.config ]; then
    echo -e "// Add predefined macros for your project here. For example:" > ${projectdir}/DiWheelDrive.config
    echo -e "// #define YOUR_CONFIGURATION belongs here" >> ${projectdir}/DiWheelDrive.config
    echo -e "" >> ${projectdir}/DiWheelDrive.config
  fi
  # generate a default .creator file if none exists so far
  if [ ! -f ${projectdir}/DiWheelDrive.creator ]; then
    echo -e "[general]" > ${projectdir}/DiWheelDrive.creator
    echo -e "" >> ${projectdir}/DiWheelDrive.creator
  fi

  # go back to user directory
  cd $userdir


  # fill the output variables
  if [ ! -z "$outvar" ]; then
    eval $outvar="$projectdir"
  fi
  if [ ! -z "$gccoutvar" ]; then
    eval $gccoutvar="$gccincludedir"
  fi

  return 0
}

### create project files for al modules ########################################
# Create project files for all modules.
#
# usage:      createAllProjects
# arguments:  n/a
# return:     0
#                 No error or warning occurred.
#
function createAllProjects {
  # print message
  printInfo "creating QtCreator project files for the DiWheelDrive module...\n"

  # read project directory
  local projectdir=""
  getProjectDir projectdir
  printInfo "files will be created in $projectdir\n"

  # retrieve gcc-arm-none-eabi include dir
  retrieveGccIncludeDir gccincludedir

  # create projects
  createLightRingProject --path="$projectdir" --gcc="$gccincludedir"
  createPowerManagementProject --path="$projectdir" --gcc="$gccincludedir"
  createDiWheelDriveProject --path="$projectdir" --gcc="$gccincludedir"

  return 0
}

### main function of this script ###############################################
# Creates, deletes and wipes QtCreator project files for the three AMiRo base modules.
#
# usage:      see function printHelp
# arguments:  see function printHelp
# return:     0
#                 No error or warning ocurred.
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
        -c|--clean)
          deleteProjects; printf "\n"; shift 1;;
        -w|--wipe)
          deleteProjects --wipe; printf "\n"; shift 1;;
        --LightRing)
          createLightRingProject; printf "\n"; shift 1;;
        --PowerManagement)
          createPowerManagementProject; printf "\n"; shift 1;;
        --DiWheelDrive)
          createDiWheelDriveProject; printf "\n"; shift 1;;
        -a|--all)
          createAllProjects; printf "\n"; shift 1;;
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
    printInfo "QtCreator setup main menu\n"
    printf "Please select one of the following actions:\n"
    printf "  [C] - clean project files\n"
    printf "  [W] - wipe project and .user files\n"
    printf "  [L] - create a project for the LightRing module\n"
    printf "  [P] - create a project for the PowerManagement module\n"
    printf "  [D] - create a project for the DiWheelDrive module\n"
    printf "  [A] - create a project for all modules\n"
    printf "  [Q] - quit this setup\n"
    local userinput=""
    readUserInput "CcWwLlPpDdAaQq" userinput
    printf "\n"

    # evaluate user selection
    case "$userinput" in
      C|c)
        deleteProjects; printf "\n";;
      W|w)
        deleteProjects --wipe; printf "\n";;
      L|l)
        createLightRingProject; printf "\n";;
      P|p)
        createPowerManagementProject; printf "\n";;
      D|d)
        createDiWheelDriveProject; printf "\n";;
      A|a)
        createAllProjects; printf "\n";;
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

