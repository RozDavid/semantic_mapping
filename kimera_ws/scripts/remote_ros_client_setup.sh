#!/bin/bash
# This scripts needs to be "sourced" and not executed in order to take effect
# source remoteClientNetwork.sh
#
# In order to simplify communication between several ros nodes zeroconf/avahi is used:
# Install like described here http://wiki.ubuntuusers.de/Avahi

if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters. Provide hostname of remote machine as parameter."
    echo "Example: source remoteClientNetwork.sh jetson-nano"
else

    REMOTE_HOST=$1

    # This scripts needs to be "sourced" and not executed in order to take effect
    # source remoteNetwork.sh

    #configure this
    #export ROS_HOSTNAME="192.168.2.165"
    ROS_PORT=11311
    #this host in AVAHI represetation
    export ROS_HOSTNAME="$HOSTNAME.local"
    
    export ROS_MASTER_URI="http://$REMOTE_HOST.local:$ROS_PORT"

    echo "ROS_HOSTNAME:$ROS_HOSTNAME"
    echo "ROS_MASTER_URI:$ROS_MASTER_URI"
   

fi
