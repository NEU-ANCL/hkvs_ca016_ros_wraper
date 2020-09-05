#!/bin/sh

################################################################################
#
# unload.sh
# for HikGEVFilter.ko unload
#
################################################################################

# Display the help for this script
DisplayHelp()
{
    echo ""
    echo "NAME"
    echo "    unload.sh - Unload the Hik For Ethernet driver module "
    echo "                HikGEVFilter.ko"
    echo ""
    echo "SYNOPSIS"
    echo "    bash unload.sh [--help]"
    echo ""
    echo "DESCRIPTION"
    echo "    Unload the eBUS Universal Pro For Ethernet module and remove the configure"
	echo "    from the system to be ready to use"
    echo "    This script can only used by root or sudoer"
    echo "    --help             Display this help"
    echo ""
}

# Print out the error and exit 
#  $1 Error message
#  $2 Exit code
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

# Ensure the module is in memory
HIKIMAGEFILTER_LOADED=`lsmod | grep -o HikGEVFilter`
if [ "$HIKIMAGEFILTER_LOADED" != "HikGEVFilter" ];then
	echo "already have no HikGEVFilter"
	exit 0
fi

# Unload the module
echo "Unloading Hik GigEVision Image Filter For Ethernet..."
/sbin/rmmod HikGEVFilter.ko $* || exit 1

