#!/bin/bash

#####################################################################
# rcl_logging_syslog rcl logging implementation via rsyslog/syslog.
#
# This script builds and tests it within ros distro public docker images.
#
# To avoid updating and modifying the files under `.github/workflows`,
# this scripts should be adjusted workflow process accordingly.
# And `.github/workflows` just calls this script in the workflow pipeline.
# This allows us to maintain the workflow process easier for contributors.
#
#####################################################################

########################
# Function Definitions #
########################

function mark {
    export $1=`pwd`;
}

function exit_trap() {
    # shellcheck disable=SC2317  # Don't warn about unreachable commands in this function
    if [ $? != 0 ]; then
        echo "Command [$BASH_COMMAND] is failed"
        exit 1
    fi
}

function install_prerequisites () {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: update and install dependent packages."
    apt update && apt upgrade -y
    apt install -y git
    # TODO(@fujitatomoya) probably we should use rosdep to install dependencies to build
    apt install -y ros-${ROS_DISTRO}-mimick-vendor ros-${ROS_DISTRO}-performance-test-fixture ros-${ROS_DISTRO}-test-msgs
    #apt install -y ros-${ROS_DISTRO}-desktop --no-install-recommends
    cd $there
}

function setup_colcon_env () {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: set up colcon build environment."
    mkdir -p ${COLCON_WORKSPACE}/src
    # fetch rcl repository in local file system to build with rcl_logging_syslog.
    # currently no dynamic loading is supported with rcl_logging, so we need to build rcl together.
    # see more details for https://github.com/ros2/rcl/issues/1178
    git clone -b ${ROS_DISTRO} https://github.com/ros2/rcl.git ${COLCON_WORKSPACE}/src/rcl
    # move rcl_logging_syslog directory to colcon workspace
    cd ${COLCON_WORKSPACE}
    cp -rf $there ${COLCON_WORKSPACE}/src
}

function setup_rsyslog () {
    # this is required basically only for container environment.
    # for security reason, container does not have system service or daemon processes.
    # just for the test with rsyslog, it installs rsyslog and start the rsyslog daemon.
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: install and setup rsyslogd."
    apt update && apt upgrade -y
    apt install -y rsyslog
    # permission setting for rsyslog file sink for the test.
    mkdir -p /var/log/ros
    chown syslog:adm /var/log/ros
    # copy the rsyslogd configuration file for the test
    cd ${COLCON_WORKSPACE}
    cp ./src/rcl_logging_syslog/config/ros2-test.conf /etc/rsyslog.d/
    rsyslogd -N1
    # start the rsyslogd daemon in the background process.
    /usr/sbin/rsyslogd -n -iNONE &
}

function build_colcon_package () {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: build rcl and rcl_logging_syslog packages."
    source /opt/ros/${ROS_DISTRO}/setup.bash
    cd ${COLCON_WORKSPACE}
    export RCL_LOGGING_IMPLEMENTATION=rcl_logging_syslog
    colcon build --symlink-install --cmake-clean-cache --packages-select rcl_logging_syslog rcl
}

function test_colcon_package () {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: test rcl_logging_syslog packages."
    # source local workspace projects
    cd ${COLCON_WORKSPACE}
    source ./install/local_setup.bash
    # initiate the colcon test
    colcon test --event-handlers console_direct+ --packages-select rcl_logging_syslog
}

########
# Main #
########

export DEBIAN_FRONTEND=noninteractive
export COLCON_WORKSPACE=/tmp/colcon_ws

# mark the working space root directory, so that we can come back anytime with `cd $there`
mark there

# set the trap on error
trap exit_trap ERR

# call install functions in sequence
install_prerequisites
setup_colcon_env
setup_rsyslog
build_colcon_package
test_colcon_package

exit 0
