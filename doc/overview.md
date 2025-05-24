---
marp: true
theme: uncover # gaia, uncover, default
header: "__ROS 2 logging rsyslog / FluentBit__"
footer: "[fujitatomoya@github](https://github.com/fujitatomoya)"
_backgroundColor: white
page_number: true
---

# [rcl_logging_syslog](https://github.com/fujitatomoya/rcl_logging_syslog)

![bg right:35% width:300px](./images/QR.png)

- ROS 2 rcl logging implementation built on [syslog(3)](https://man7.org/linux/man-pages/man3/syslog.3.html).
- Connects with [rsyslog](https://www.rsyslog.com/) and [FluentBit](https://fluentbit.io/).

<!---
Comment Here
--->

---

![bg 70%](https://images.squarespace-cdn.com/content/v1/606d378755a86f589aa297b7/1653397531343-6M4IQ4JWDQV1SQ8W17UN/HumbleHawksbill_TransparentBG-NoROS.png)
![bg 70%](https://images.squarespace-cdn.com/content/v1/606d378755a86f589aa297b7/ebf9b1d5-45b7-4a73-8f48-dc5d3f4fc8fc/JazzyJalisco_Final.png?format=1500w)
![bg 70%](https://us1.discourse-cdn.com/flex022/uploads/ros/optimized/3X/1/a/1aeb03b4cdf23885ce9cf7e778c8d7bbb8fb5fe0_2_750x750.png)
![bg 70%](https://images.squarespace-cdn.com/content/v1/606d378755a86f589aa297b7/1628726028642-TVRVRIQL914IVYWV8MG9/rolling.png)

<!---
Note that rolling is officially verified, but other distros should work without any change.
rcl_logging_syslog is only dependent on legacy syslog(3C) interface, so no difference at all.
--->

---

# Objectives

- Configure log behavior without code change.
- Reasonable Performance.
- Log data pipeline and forward capability support.
- Enabling ROS 2 logging system with Cloud-Native Log Management and Observability.

<!---
Comment Here
--->

---

![bg 70%](./images/architecture_overview.png)

<!---
This logging design is just one of the example can be supported by the architecture.
rsyslog and FluentBit enables user to support any log data pipeline with security TLS.
Even more, this architecture can take advantage of Cloud-Native services and tools.
--->

---

# [Demo FluentBit](https://github.com/user-attachments/assets/bdb05bf7-92b2-4b9a-8f20-3d3b803a7a86)

<video controls="controls" width="1000" src="https://github.com/user-attachments/assets/bdb05bf7-92b2-4b9a-8f20-3d3b803a7a86">

<!---
Comment Here
--->

---

# [Demo Fluentd/Loki/Grafana](https://github.com/user-attachments/assets/4a1aae42-5c55-4f31-9198-8c7c246244ca)

<video controls="controls" width="1100" src="https://github.com/user-attachments/assets/4a1aae42-5c55-4f31-9198-8c7c246244ca">

<!---
Comment Here
--->

---

# [rsyslog](https://www.rsyslog.com/)

## a.k.a rocket-fast system for log processing 🚀🚀🚀

[rsyslog](https://www.rsyslog.com/) is available in default Ubuntu distribution managed by system service, performative, and many configuration supported including log data pipeline.
So that user can choose the logging configuration depending on the application requirement and use case, sometimes file system sink, sometimes forwarding to remote rsyslogd, or even [FluentBit](https://github.com/fluent/fluent-bit).

<!---
SYSLOG(3) is really simple that does not have much interfaces to control on application side, it just writes the log data on rsyslog Unix Domain Socket.
--->

---

# [FluentBit](https://github.com/fluent/fluent-bit)

![bg right:70% fit](https://imagedelivery.net/xZXo0QFi-1_4Zimer-T0XQ/cd90e2c5-86b3-42be-5a24-3cc25fb96000/orig)

---

- Lightweight and Efficient: suitable for environments with limited computational power.
- High Performance: capable of handling high-volume data streams with minimal latency. It leverages asynchronous I/O and efficient data processing techniques to ensure optimal performance.
- Flexibility: supports a wide range of data sources and destinations.
- Configurability: offers a flexible configuration language that allows you to customize its behavior to fit your specific needs.
- Extensibility: highly extensible through plugins including custom ones.
- Scalability: easily scaled horizontally to handle increasing data volumes by deploying multiple instances.
- Reliability: features like fault tolerance and retry mechanisms to ensure data reliability.

<!---
Comment Here
--->

---

## Issues and PRs always welcome 🚀

https://github.com/fujitatomoya/rcl_logging_syslog

![bg left:35% width:300px](./images/QR.png)

<!---
Comment Here
--->
