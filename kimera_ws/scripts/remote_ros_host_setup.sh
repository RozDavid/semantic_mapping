#!/bin/bash
# This scripts needs to be "sourced" and not executed in order to take effect
# source remoteHostNetwork.sh
#
# In order to simplify communication between several ros nodes zeroconf/avahi is used:
# Install like described here http://wiki.ubuntuusers.de/Avahi

function int-ip { /sbin/ifconfig $1 | grep "inet addr" | awk -F: '{print $2}' | awk '{print $1}'; }

#Configure this if required
NETWORK_DEVICE="wlan0"
ROS_PORT=11311

#ROS_HOSTNAME=`int-ip $NETWORK_DEVICE`

#this host in AVAHI represetation
export ROS_HOSTNAME="$HOSTNAME.local"
export ROS_MASTER_URI="http://$ROS_HOSTNAME:$ROS_PORT"

echo "ROS_HOSTNAME=$ROS_HOSTNAME"
echo "ROS_MASTER_URI=$ROS_MASTER_URI"

