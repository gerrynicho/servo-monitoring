#include "tachimawari_interfaces/msg/current_joints.hpp"
#include "rclcpp/rclcpp.hpp"

#include "servo-monitoring/exporter.hpp"

#include <chrono>   
#include <memory>

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("exporter");
    auto prometheus_exporter = std::make_shared<exporter>(node);
    prometheus_exporter->exposer.RegisterCollectable
        (prometheus_exporter->get_registry());
    rclcpp::spin(node);
    rclcpp::shutdown();
}
