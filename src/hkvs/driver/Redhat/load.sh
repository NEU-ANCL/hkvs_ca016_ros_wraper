#!/bin/sh

################################################################################
#
# load.sh
# for HikGEVFilter.ko load
#
################################################################################

# Variables
HOST_VERSION=`uname -r`
HOST_ARCH=`uname -m | sed -e 's/i.86/i686/' -e 's/^armv.*/arm/'`

#Display the help for this script
DisplayHelp()
{
    echo ""
    echo "NAME"
    echo "    load.sh - Load the Hik For Ethernet driver module "
    echo "              HikGEVFilter.ko"
    echo ""
    echo "SYNOPSIS"
    echo "    bash load.sh [--help]"
    echo ""
    echo "DESCRIPTION"
    echo "    Load the Hik For Ethernet module and configure the system to"
    echo "    be ready to use"
    echo "    This script can only used by root or sudoer"
    echo "    --help             Display this help"
    echo ""
}

#Print out the error and exit 
# $1 Error message
# $2 Exit code
ErrorOut()
{
	echo ""
	echo "Error: $1"
	echo ""
	exit $2
}

# Parse the input arguments
for i in $*
do
    case $i in        
        --help)
            DisplayHelp
            exit 0
        ;;
        *)
        # unknown option
        DisplayHelp
        exit 1
        ;;
    esac
done

# Check required priviledge
if [ `whoami` != root ]; then
	ErrorOut "This script can only be run by root user or sudoer" 1
fi

# Do not re-load if not needed
HIKIMAGEFILTER_LOADED=`lsmod | grep -o HikGEVFilter`
if [ "$HIKIMAGEFILTER_LOADED" = "HikGEVFilter" ];then
	echo "already have HikGEVFilter"
	exit 0
fi

# Sanity check
if [ "x86_64" != "$HOST_ARCH" ]; then
    ErrorOut "*** The module HikGEVFilter.ko can only be load with a kernel x86_64 ***" 1
fi

# Load the module
echo "Loading Hik GigEVision Image Filter For Ethernet for x86_64 ..."
/sbin/insmod ./HikGEVFilter.ko $* || exit 1

chmod 0777 /dev/HikGEVFilter

echo "end..."


