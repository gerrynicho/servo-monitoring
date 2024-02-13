#include <prometheus/exposer.h>
#include <prometheus/gauge.h>
#include <prometheus/registry.h>

#include "tachimawari_interfaces/msg/current_joints.hpp"
#include "rclcpp/rclcpp.hpp"

#include "servo-monitoring/exporter.cpp"

#include <array>
#include <chrono>   
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
using CurrentJoints = tachimawari_interfaces::msg::CurrentJoints;
using std::placeholders::_1;

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("exporter");
    auto prometheus_exporter = std::make_shared<exporter>(node);
    prometheus_exporter->exposer.RegisterCollectable
        (prometheus_exporter->get_registry());
    rclcpp::spin(node);
    rclcpp::shutdown();
}
