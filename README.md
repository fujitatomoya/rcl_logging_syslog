# rcl_logging_syslog 🚢🚀🚂

[rcl_logging_syslog](https://github.com/fujitatomoya/rcl_logging_syslog) is alternative logging backend implementation that can be used for [ROS 2](https://github.com/ros2) application via [rcl_logging_interface](https://github.com/ros2/rcl_logging/tree/rolling/rcl_logging_interface).

[rcl_logging_syslog](https://github.com/fujitatomoya/rcl_logging_syslog) uses [SYSLOG(3)](https://man7.org/linux/man-pages/man3/syslog.3.html) to send the log data to [rsyslog](https://www.rsyslog.com/) a.k.a **rocket-fast system for log processing** 🚀.

The main objective is that **Enabling ROS 2 logging system with Cloud-Native Log Management and Observability**.

<img src="./doc/images/architecture_overview.png" width="700">

## Motivation

The logging data is critical especially for entire system observability and status, so that application can alert the administrator or even give the feedback to the system with adjusting the parameter.
This importance rises once it comes to robotics and robot application, especially distributed system such as [ROS 2](https://github.com/ros2) or edge computing because we must be able to specify what went wrong in the 1st place with these logging data.

[rsyslog](https://www.rsyslog.com/) is available in default Ubuntu distribution managed by system service, performative, and many configuration supported including log data pipeline.
So that user can choose the logging configuration depending on the application requirement and use case, sometimes **file system sink**, sometimes **forwarding to remote rsyslogd**, or even **[FluentBit](https://github.com/fluent/fluent-bit)**.

[FluentBit](https://github.com/fluent/fluent-bit) is a **Fast Log Processor and Forwarder** part of Graduated [Fluentd](https://www.fluentd.org/) Ecosystem and a [CNCF](https://www.cncf.io/) sub-project.

## Demonstration

See how it works 🔥

https://github.com/user-attachments/assets/bdb05bf7-92b2-4b9a-8f20-3d3b803a7a86

## Supported [ROS Distribution](https://docs.ros.org/en/rolling/Releases.html)

| Distribution      | Supported | Note |
| :---------------- | :-------- | :--- |
| Rolling Ridley    |    ✅     | Development / Mainstream Branch |
| Jazzy Jalisco     |    ⛔     | W.I.P (No distro specific dependency, should work.) |
| Iron Irwini       |    ⛔     | W.I.P (No distro specific dependency, should work.) |
| Humble Hawksbill  |    ⛔     | W.I.P (No distro specific dependency, should work.) |

## Installation

### Prerequisites

- [rsyslog](https://www.rsyslog.com/) installation

[rcl_logging_syslog](https://github.com/fujitatomoya/rcl_logging_syslog) requires `rsyslog` package, which Ubuntu should have already in default.
But if you are using container, the situation is bit different from host system since there is no system services or `rsyslogd` is running by default.
In the case of container, we need to install `rsyslog` packages in the container root file system.

> [!NOTE]
> We can enable the container with host system privileges but that is NOT recommended, especially for security.

```bash
### Install rsyslog package
apt install rsyslog
```

- [rsyslog](https://www.rsyslog.com/) service check

If you are running the application on host system, check the `rsyslog` system service.

```bash
systemctl status rsyslog
```

If you are running the application on container environment, start the `rsyslogd` daemon process.

```bash
/usr/sbin/rsyslogd -n -iNONE
```

- [fluent-bit](https://github.com/fluent/fluent-bit) installation

if you want to use `fluent-bit` to forward the logging data from `rsyslogd`, please follow [Fluent Bit Installation Manual](https://docs.fluentbit.io/manual/installation/linux/ubuntu) to install it.

```bash
### Check fluent-bit package is installed
dpkg -l fluent-bit
Desired=Unknown/Install/Remove/Purge/Hold
| Status=Not/Inst/Conf-files/Unpacked/halF-conf/Half-inst/trig-aWait/Trig-pend
|/ Err?=(none)/Reinst-required (Status,Err: uppercase=bad)
||/ Name           Version      Architecture Description
+++-==============-============-============-=================================
ii  fluent-bit     3.1.6        amd64        Fast data collector for Linux
```

### Build

Currently we need to build the source code if we want to use alternative logging backend with [ROS 2](https://github.com/ros2).
See more details for https://github.com/ros2/rcl/issues/1178.

Please follow [ROS 2 Official Development / Installation](https://docs.ros.org/en/rolling/Installation/Alternatives/Ubuntu-Development-Setup.html) to build the `rcl_logging_syslog` package below.

```bash
### Clone the repository under the workspace
cd <YOUR_WORKSPACE>/src
git clone https://github.com/fujitatomoya/rcl_logging_syslog.git

### Build rcl_logging_syslog
export RCL_LOGGING_IMPLEMENTATION=rcl_logging_syslog
colcon build --symlink-install --cmake-clean-cache --packages-select rcl_logging_syslog rcl
```

### Configuration

[SYSLOG(3)](https://man7.org/linux/man-pages/man3/syslog.3.html) is really simple that does not have much interfaces to control on application side, it just writes the log data on `rsyslog` Unix Domain Socket.
So we need to configure `rsyslog` how it manages the log message with `/etc/rsyslog.conf`, for example file system sink and forward the message to `fluent-bit`.

At this moment, `rcl_logging_syslog` uses syslog facility only `local1`.
More facilities should be supported in the future via environmental variables at `rcl_logging_syslog` initialization, so that user application can use appropriate facility configured by `rsyslog`.

See more details for https://www.rsyslog.com/doc/index.html.

#### `rsyslog`

Add the following configuration to `/etc/rsyslog.conf`, replace `<FLUENTBIT_IP>` with your own IP address where `fluent-bit` runs.

```bash
#
# ROS Setting
#

$template TemplateName1,"%timereported% %hostname% %syslogfacility-text%.%syslogseverity-text%: %syslogtag%%msg:::sp-if-no-1s\
t-sp%%msg:::drop-last-lf%\n"

# TODO(@fujitatomoya): template property does not work in container.
#$template TemplateName2,"/var/log/ros/%hostname%/%programname%_%procid%_%$now%.log"
$template TemplateName2,"/var/log/ros/%hostname%/pid_%procid%-%$now%.log"

# Log locally with specified templated file name.
local1.* ?TemplateName2;TemplateName1
# For remote server, it also needs to set those templates
local1.* @@<FLUENTBIT_IP>:5140
```

Check the configuration file to see if there is no error,

```bash
rsyslogd -N1
rsyslogd: version 8.2312.0, config validation run (level 1), master config /etc/rsyslog.conf
rsyslogd: End of config validation run. Bye.
```

then, restart the `rsyslog` system service, or simply restart `rsyslog` daemon when using container.

```bash
### via systemd (not container environment)
sudo systemctl restart rsyslog
systemctl status rsyslog

### container environment
/usr/sbin/rsyslogd -n -iNONE
```

#### [fluent-bit](https://github.com/fluent/fluent-bit)

With above configuration, `rsyslogd` also sends log data to TCP port 5140 that `fluent-bit` listens with the following configuration.
Replace `<FLUENTBIT_IP>` with your own IP address in `fluent-bit.conf`.

```bash
### Create working directory
mkdir fluentbit_ws; cd fluentbit_ws

### Copy the original files to the local directory
cp -rf /etc/fluent-bit/fluent-bit.conf fluent-bit.conf
cp -rf /etc/fluent-bit/parsers.conf parsers.conf

### Modify the configuration as below
cat fluent-bit.conf
[SERVICE]
    Flush        1
    Parsers_File parsers.conf

[INPUT]
    Name     syslog
    Parser   syslog-rfc3164
    Listen   <FLUENTBIT_IP>
    Port     5140
    Mode     tcp

[OUTPUT]
    Name     stdout
    Match    *

# Start fluent-bit
/opt/fluent-bit/bin/fluent-bit --config=./fluent-bit.conf
```

## Usage

Logging messages from ROS 2 application will be sent to `rsyslogd` and forwarded to `fluent-bit` if that is configured.

### Examples

The followings are example output for file system and [fluent-bit](https://github.com/fluent/fluent-bit).

Start ROS 2 demo application:

```bash
ros2 run demo_nodes_cpp talker
[WARN] [1724453269.434832744] []: syslog logging backend doesn't support external configuration, configure rsyslogd.conf instead of /root/.ros/log log directory
[INFO] [1724453270.445092186] [talker]: Publishing: 'Hello World: 1'
[INFO] [1724453271.444981999] [talker]: Publishing: 'Hello World: 2'
[INFO] [1724453272.445056310] [talker]: Publishing: 'Hello World: 3'
...
```

Logging files under file system:

```bash
find /var/log/ros/
/var/log/ros/
/var/log/ros/tomoyafujita
/var/log/ros/tomoyafujita/pid_2594-2024-08-23.log

cat /var/log/ros/tomoyafujita/pid_2594-2024-08-23.log
Aug 23 16:02:05 tomoyafujita local1.info: `▒▒J[2594]: 4]: [INFO] [1724454125.014384556] [talker]: Publishing: 'Hello World: 1'
Aug 23 16:02:06 tomoyafujita local1.info: `▒▒J[2594]: 4]: [INFO] [1724454126.014415486] [talker]: Publishing: 'Hello World: 2'
Aug 23 16:02:07 tomoyafujita local1.info: `▒▒J[2594]: 4]: [INFO] [1724454127.014399261] [talker]: Publishing: 'Hello World: 3'
Aug 23 16:02:07 tomoyafujita local1.info: `▒▒J[2594]: 4]: [INFO] [1724454127.648797600] [rclcpp]: signal_handler(signum=2)
```

> [!WARNING]
> Some properties do not work container environment, need to address these issues.

FluentBit output:

```bash
/opt/fluent-bit/bin/fluent-bit --config=./fluent-bit.conf
Fluent Bit v3.1.6
* Copyright (C) 2015-2024 The Fluent Bit Authors
* Fluent Bit is a CNCF sub-project under the umbrella of Fluentd
* https://fluentbit.io

______ _                  _    ______ _ _           _____  __
|  ___| |                | |   | ___ (_) |         |____ |/  |
| |_  | |_   _  ___ _ __ | |_  | |_/ /_| |_  __   __   / /`| |
|  _| | | | | |/ _ \ '_ \| __| | ___ \ | __| \ \ / /   \ \ | |
| |   | | |_| |  __/ | | | |_  | |_/ / | |_   \ V /.___/ /_| |_
\_|   |_|\__,_|\___|_| |_|\__| \____/|_|\__|   \_/ \____(_)___/

[2024/08/23 16:02:00] [ info] [fluent bit] version=3.1.6, commit=, pid=1686882
[2024/08/23 16:02:00] [ info] [storage] ver=1.5.2, type=memory, sync=normal, checksum=off, max_chunks_up=128
[2024/08/23 16:02:00] [ info] [cmetrics] version=0.9.4
[2024/08/23 16:02:00] [ info] [ctraces ] version=0.5.5
[2024/08/23 16:02:00] [ info] [input:syslog:syslog.0] initializing
[2024/08/23 16:02:00] [ info] [input:syslog:syslog.0] storage_strategy='memory' (memory only)
[2024/08/23 16:02:00] [ info] [in_syslog] TCP server binding 43.135.146.89:5140
[2024/08/23 16:02:00] [ info] [sp] stream processor started
[2024/08/23 16:02:00] [ info] [output:stdout:stdout.0] worker #0 started
[0] syslog.0: [[1724428925.000000000, {}], {"pri"=>"142", "time"=>"Aug 23 16:02:05", "host"=>"tomoyafujita", "message"=>"4]: [INFO] [1724454125.014384556] [talker]: Publishing: 'Hello World: 1'"}]
[0] syslog.0: [[1724428926.000000000, {}], {"pri"=>"142", "time"=>"Aug 23 16:02:06", "host"=>"tomoyafujita", "message"=>"4]: [INFO] [1724454126.014415486] [talker]: Publishing: 'Hello World: 2'"}]
[0] syslog.0: [[1724428927.000000000, {}], {"pri"=>"142", "time"=>"Aug 23 16:02:07", "host"=>"tomoyafujita", "message"=>"4]: [INFO] [1724454127.014399261] [talker]: Publishing: 'Hello World: 3'"}]
[0] syslog.0: [[1724428927.000000000, {}], {"pri"=>"142", "time"=>"Aug 23 16:02:07", "host"=>"tomoyafujita", "message"=>"4]: [INFO] [1724454127.648797600] [rclcpp]: signal_handler(signum=2)"}]
...
```

## Reference

- https://www.rsyslog.com/doc/index.html
- https://docs.fluentbit.io/manual
