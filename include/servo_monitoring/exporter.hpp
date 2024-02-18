#ifndef SERVO__MONITORING__EXPORTER_HPP_
#define SERVO__MONITORING__EXPORTER_HPP_

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
using Gauge = std::reference_wrapper<prometheus::Gauge>;

class Exporter : public rclcpp::Node
{
public:
    Exporter(rclcpp::Node::SharedPtr node);
    prometheus::Exposer exposer;
    std::shared_ptr<prometheus::Registry> get_registry();
    rclcpp::Node::SharedPtr node;
private:
    void topic_callback(const ConsumingCurrentJoints & incoming_message) const;
    void setGauge(const std::vector<ConsumingCurrentJoint> & new_joints) const;
    rclcpp::Subscription<ConsumingCurrentJoints>::SharedPtr subscription;
    std::shared_ptr<prometheus::Registry> registry;
    prometheus::Family<prometheus::Gauge>& GaugeBuilder;
    std::vector<Gauge> joints;
};

#endif