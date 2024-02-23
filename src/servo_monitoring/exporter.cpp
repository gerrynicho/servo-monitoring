#include "servo_monitoring/exporter.hpp"

#include <prometheus/exposer.h>
#include <prometheus/gauge.h>
#include <prometheus/registry.h>

#include <array>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tachimawari_interfaces/msg/current_joints.hpp"
using ConsumingCurrentJoints = tachimawari_interfaces::msg::CurrentJoints;
using ConsumingCurrentJoint = tachimawari_interfaces::msg::Joint;
using std::placeholders::_1;

void Exporter::setGauge(const std::vector<ConsumingCurrentJoint> & new_joints) const
{
    for(int i = 0; i < 20; i++) {
        joints[i].get().Set(new_joints[i].position);
    }
}

void Exporter::topic_callback(const ConsumingCurrentJoints & incoming_message) const
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Got msg");
    auto & new_joints = incoming_message.joints;
    setGauge(new_joints);
}

Exporter::Exporter(rclcpp::Node::SharedPtr node) 
    : Node("current_subscriber"),
      node(node),
      exposer("127.0.0.1:6969"),
      registry(std::make_shared<prometheus::Registry>()),
      GaugeBuilder(prometheus::BuildGauge()
                   .Name("consuming_current_joints_on_robot")
                   .Help("Joints that will be monitored")
                   .Register(*registry))
{
    joints.reserve(20);
    for(int i = 0; i < 20; i++) {
        joints[i] = GaugeBuilder.Add({{"id", std::to_string(i+1)}});
    }
    subscription = node->create_subscription<ConsumingCurrentJoints>
                ("joint/consuming_current_joints", 
                 10,
                 std::bind(&Exporter::topic_callback, this, _1));
}


std::shared_ptr<prometheus::Registry> Exporter::get_registry()
{
    return registry;
}