# How to forward the ros2 log data to [Fluentd](https://www.fluentd.org/) / [Loki](https://grafana.com/oss/loki/) / [Grafana](https://grafana.com/oss/grafana/)

## Overview

```text
                   +------------------------------------------------+
                   |                                                |
                   |   +---------+                                  |
+--------+ 514/udp |   |         |                                  |
| client +------------->         |                                  |
+--------+         |   |         |                                  |
                   |   | rsyslog |                                  |
+--------+ 514/udp |   |         |                                  |
| client +------------->         |                                  |
+--------+         |   |         |                                  |
                   |   +----+----+                                  |
                   |        |                                       |
                   |        | 5514/tcp                              |
                   |        |                                   http/3000
                   |   +----v-----+  +----------+  +------------+   |  +-----------+
                   |   | fluentd  +-->   loki   +-->  grafana   +------>  Browser  |
                   |   +----------+  +----------+  +------------+   |  +-----------+
                   |                   http/3100                    |
                   +------------------------------------------------+
```

## Installation

- [Loki](https://grafana.com/oss/loki/) / [Grafana](https://grafana.com/oss/grafana/)

  Reference: https://grafana.com/docs/grafana/latest/setup-grafana/installation/debian/

```bash
sudo apt-get install grafana loki
```

- [Fluentd](https://www.fluentd.org/)

  Reference: https://docs.fluentd.org/installation/install-by-deb

```bash
### This is an example to install Ubuntu Focal and Fluentd LTS
curl -fsSL https://toolbelt.treasuredata.com/sh/install-ubuntu-focal-fluent-package5-lts.sh | sh
```

  To send the data from Fluentd to Loki, it requires [fluent-plugin-grafana-loki](https://grafana.com/docs/loki/latest/send-data/fluentd/) provided by Grafana Lab.

```bash
fluent-gem install fluent-plugin-grafana-loki
```

## Setup / Configuration

- grafana-server

  Apply the following change to use your machine IP address.

```patch
--- /etc/grafana/grafana.ini	2024-09-05 14:53:08.578366205 -0700
+++ /etc/grafana/grafana.ini.org	2024-09-05 14:52:50.286486517 -0700
@@ -35,7 +35,7 @@
 ;min_tls_version = ""

 # The ip address to bind to, empty will bind to all interfaces
-;http_addr = <YOUR_IP_ADDRESS>
+;http_addr =

 # The http port to use
 ;http_port = 3000
```

  then, restart the grafana-server system service.

```bash
sudo systemctl restart grafana-server
sudo systemctl status grafana-server
```

  you can now access http://<YOUR_IP_ADDRESS>:3000/ with `admin`/`admin`. (1st time, be requested to change password.)

- loki

  Apply the following change to use your machine IP address.

```patch
--- /etc/loki/config.yml	2024-09-05 15:07:28.938148908 -0700
+++ /etc/loki/config.yml.org	2024-09-05 15:07:12.466242279 -0700
@@ -5,7 +5,7 @@ server:
   grpc_listen_port: 9096

 common:
-  instance_addr: <YOUR_IP_ADDRESS>
+  instance_addr: 127.0.0.1
   path_prefix: /tmp/loki
   storage:
     filesystem:
```

  then, restart the grafana-server system service.

```bash
sudo systemctl restart loki
sudo systemctl status loki
```

- rsyslog

  Add the following configuration to `/etc/rsyslog.d/ros2-logging.conf`.

```ros2-logging.conf
...
# Fluentd Setting
$ModLoad imudp
$UDPServerRun 514

$ActionQueueType LinkedList
$ActionQueueFileName srvrfwd
$ActionResumeRetryCount -1
$ActionQueueSaveOnShutdown on

local1.* @@<YOUR_IP_ADDRESS>:5514;RSYSLOG_SyslogProtocol23Format
local2.* @@<YOUR_IP_ADDRESS>:5514;RSYSLOG_SyslogProtocol23Format
local3.* @@<YOUR_IP_ADDRESS>:5514;RSYSLOG_SyslogProtocol23Format
```

  and restart the rsyslog.

```bash
### via systemd (not container environment)
sudo systemctl restart rsyslog
systemctl status rsyslog

### container environment
/usr/sbin/rsyslogd -n -iNONE
```

- fluentd

  Create the following simple configuration to send the syslog data to Loki listening port. (`<YOUR_IP_ADDRESS>` needs to be replaced as well.)

```ros2-fluentd.conf
<source>
  @type syslog
  @id syslog_in
  port 5514
  bind <YOUR_IP_ADDRESS>
  severity_key level
  facility_key facility
  tag syslog
  <transport tcp>
  </transport>
  <parse>
    message_format rfc5424
  </parse>
</source>

<match syslog.**>
  @type loki
  @id syslog_out
  url "http://<YOUR_IP_ADDRESS>:3100"
  extra_labels {"app":"ros2"}
  <label>
    pid
    host
    level
    facility
  </label>
</match>
```

  and then, start fluentd with the following commands.

```bash
### Stop system service for now
sudo systemctl stop fluentd

### Start fluentd
fluentd -c ros2-fluentd.conf
```

## Usage

Run the ROS 2 demo application.

```bash
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
ros2 run demo_nodes_cpp add_two_ints_server
while true; do ros2 run demo_nodes_cpp add_two_ints_client; sleep 1; done
```

you can now access http://<YOUR_IP_ADDRESS>:3000/ to explore the log data with Grafana.

https://github.com/user-attachments/assets/4a1aae42-5c55-4f31-9198-8c7c246244ca

## Reference

- https://docs.fluentd.org/
- https://grafana.com/docs/loki/latest/
- https://grafana.com/docs/grafana/latest/
