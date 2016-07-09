#!/bin/bash
set -e
set -u

# This script needs to be run as root, but 
# also needs to set permissions to allow a non-root
# user to access the TAP devices. Instead of hardcoding
# the non-root user here, it uses the owner of this script
# file.
USER=`stat -c %U "$0"`

# Create tun device
DEV1=`tunctl -b -u $USER`
DEV2=`tunctl -b -u $USER`
echo "Created $DEV1 and $DEV2"

chown root.$USER /dev/net/tun

ifconfig $DEV1 -multicast arp mtu 1440 10.0.0.1 netmask 255.255.255.0
ifconfig $DEV2 -multicast arp mtu 1440 10.1.0.2 netmask 255.255.255.0

# add a trap to remove the tun device
#trap "trap - EXIT; tunctl -d $DEV; exit" INT TERM EXIT

#MAC=`ip link show splat0 | grep link/ether | awk '{print $2}'`

#./tap $DEV 11:22:33:44:55:66 10.0.0.2

#./netport
