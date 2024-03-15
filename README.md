# servo-monitoring
This is the final product and improved version of the prototype [basic-sub-prom](https://github.com/gerrynicho/basic-sub-prom) that will be used for my ICHIRO's final presentation task

## Table of Contents
- [servo-monitoring](#servo-monitoring)
  - [Table of Contents](#table-of-contents)
  - [Requirements](#requirements)
  - [Usage](#usage)
  - [Documentation](#documentation)
  - [Default Settings](#default-settings)

## Requirements
the required dependencies for the repository can be installed here
- [Tachimawari](https://github.com/ichiro-its/tachimawari/tree/feature/current-monitoring)
- [rclcpp Iron for Ubuntu](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)
- [Prometheus Client Library](https://github.com/jupp0r/prometheus-cpp)
- [Grafana](https://grafana.com/docs/grafana/latest/setup-grafana/installation/debian/)

After all of the required installation is finished, edit the prometheus.yml file (located in the Prometheus Client Library directory) and change accordingly. The [prometheus.yml.example](https://github.com/gerrynicho/servo_monitoring/blob/689e30d8aa3bc7c1d2f6a7a94e8914be77801b61/prometheus.yml.example) file shows how to include the scrape target into Prometheus

## Usage
1. Run `systemctl start grafana-server` in terminal to start local Grafana Client
<br/><br/>
2. Access to the Prometheus Client directory and run prometheus.exe 
<br/><br/>
3. While the prometheus client is still running in the background, run this package using `ros2 run servo_monitoring jalan` 
<br/><br/>
4. Access Grafana from the Grafana port (default access port to Grafana `[host address]:3000`) and log in with the email/username and password being `admin`
<br/><br/>
5. Create a new dashboard with prometheus being the data source (the prometheus server url will be `[host address]:9090`) and set the scrape interval accordingly to the publisher timer callback
<br/><br/>
6. Select the metrics that will be monitored
<br/><br/>
7. If there is an issue displaying the data, make sure the "instant" key is set to true in the JSON query inspector
<br/><br/>
8. Resize the graph to your liking and make sure the auto refresh interval is turned on (the auto refresh interval option is located in the top right of the dashboard)

## Documentation
this repository relies on one class that being the [Exporter](https://github.com/gerrynicho/servo_monitoring/blob/689e30d8aa3bc7c1d2f6a7a94e8914be77801b61/include/servo_monitoring/exporter.hpp#L22) class that functions as the tachimawari subscriber as well as formatter for the Prometheus database.

## Default Settings
| Attributes | Value |
| ---------- | ----- |
| Scrape Port | :6969 |
| Metric name | consuming_current_joints_on_robot|
| Topic subscriber | joint/consuming_current_joints |
| Package Executable | `ros2 run servo_monitoring jalan`|

