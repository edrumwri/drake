#!/bin/bash
# This bash script uses the xacro tools in ROS to generate SDFs from the "*.sdf.xacro" files that share a directory with this script.

# Generates an sdf with name "${2}/chopstick_${1}.sdf" from a file with signature "${2}/chopstick_${1}.sdf.xacro".
function generate_chopstick_sdf() 
{
    local SIDE="${1}"
    local DIR="${2}"
    rosrun xacro xacro ${DIR}/chopstick_${SIDE}.sdf.xacro \
        | sed '/robot/d' > ${DIR}/chopstick_${SIDE}.sdf
}

source /opt/ros/melodic/setup.bash

DIR=$(realpath $(dirname ${0}))
generate_chopstick_sdf "left" ${DIR}
generate_chopstick_sdf "right" ${DIR}
