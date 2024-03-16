# servo_monitoring
This is the final product and improved version of the prototype [basic-sub-prom](https://github.com/gerrynicho/basic-sub-prom) that will be used for my ICHIRO's final presentation task. The purpose of this repository is to monitor DYNAMIXEL servo's consuming current that is used in ICHIRO's humanoid robots.

# Table of Contents
- [servo\_monitoring](#servo_monitoring)
- [Table of Contents](#table-of-contents)
- [Requirements](#requirements)
- [Usage](#usage)
- [Documentation](#documentation)
    - [Initialize The Class](#initialize-the-class)
    - [Vector Of Gauge](#vector-of-gauge)
    - [Spinning The Node](#spinning-the-node)
- [Default Settings](#default-settings)

# Requirements
the required dependencies for the repository can be installed here
- [Tachimawari](https://github.com/ichiro-its/tachimawari/tree/feature/current-monitoring)
  - [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/download/)
  - [kansei_interfaces](https://github.com/ichiro-its/kansei_interfaces)
  - [keisan](https://github.com/ichiro-its/keisan)
  - [tachimawari_interfaces](https://github.com/ichiro-its/tachimawari_interfaces)
- [rclcpp Iron for Ubuntu](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)
- [Prometheus Client Library](https://github.com/jupp0r/prometheus-cpp)
- [Grafana](https://grafana.com/docs/grafana/latest/setup-grafana/installation/debian/)

After all of the required installation is finished, edit the prometheus.yml file (located in the Prometheus Client Library directory) and change accordingly. The [prometheus.yml.example](https://github.com/gerrynicho/servo_monitoring/blob/689e30d8aa3bc7c1d2f6a7a94e8914be77801b61/prometheus.yml.example) file shows how to include the scrape target into Prometheus

# Usage
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

# Documentation
this repository relies on one class that being the [Exporter](https://github.com/gerrynicho/servo_monitoring/blob/689e30d8aa3bc7c1d2f6a7a94e8914be77801b61/include/servo_monitoring/exporter.hpp#L22) class that functions as the tachimawari subscriber as well as formatter for the Prometheus database.

### Initialize The Class
When first making an instance of the class, we immediately declare the port for the scrape target (that being 127.0.0.1:6969), the registry to contain gauge family and the gauge family where the each gauge for each servo's data is located.
```cpp
Exporter::Exporter(rclcpp::Node::SharedPtr node) 
    : Node("current_subscriber"), // the name of the node
      node(node), // transform the exporter class into a ros2 subscriber node
      exposer("127.0.0.1:6969"), // declare the scrape target
      registry(std::make_shared<prometheus::Registry>()), 
      // declare the registry that will house the gauge family
      GaugeBuilder(prometheus::BuildGauge()
                   .Name("consuming_current_joints_on_robot")
                   .Help("Joints that will be monitored")
                   .Register(*registry)) 
      // declare the gauge family that will be linked into the previously declared registry
```
### Vector Of Gauge
Due to how prometheus-cpp works, we cannot directly declare a vector of gauge. Instead, we can use std::refence_wrapper\<prometheus::Gauge> that wraps the refencence prometheus::Gauge into an assignable object 
```cpp
using Gauge = std::reference_wrapper<prometheus::Gauge>;
std::vector<Gauge> joints;
```
<br/>
In the constructor of the class, the vector of gauges needs to be declared to be located in the gauge family

```cpp
joints.reserve(20); // reserve  20 indices because we need to monitor 20 servos
    for(int i = 0; i < 20; i++) {
        joints[i] = GaugeBuilder.Add({{"id", std::to_string(i+1)}}); 
        // declare the gauge to the gauge family with their own identifying id label
    }
```

### Spinning The Node
Every action that the node makes is after the node received the message that was published by tachimawari, meaning that the node will do nothing if tachimawari has not published any message.

When the node received a message, it calls on the function setGauge(incoming_message.joints) with incoming_message.joints being the consuming current data that tachimawari published

```cpp

void Exporter::setGauge(const std::vector<ConsumingCurrentJoint> & new_joints) const
{
    for(int i = 0; i < 20; i++) {
        joints[i].get().Set(new_joints[i].position); 
        // remember that joints is a vector of std::reference_wrapper<prometheus::Gauge>
        // to access the prometheus::gauge itself we need to use joints[i].get()
    }
}
```

After we set the gauge to the according data, the registry will expose the gauge to 127.0.0.1:6969 which the prometheus client will then scraped.



# Default Settings
| Attributes | Value |
| ---------- | ----- |
| Scrape Port | :6969 |
| Metric name | consuming_current_joints_on_robot|
| Topic subscriber | joint/consuming_current_joints |
| Package Executable | `ros2 run servo_monitoring jalan`|

