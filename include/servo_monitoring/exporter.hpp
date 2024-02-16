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
    prometheus::Gauge& joint_1;
    prometheus::Gauge& joint_2;
    prometheus::Gauge& joint_3;
    prometheus::Gauge& joint_4;
    prometheus::Gauge& joint_5;
    prometheus::Gauge& joint_6;
    prometheus::Gauge& joint_7;
    prometheus::Gauge& joint_8;
    prometheus::Gauge& joint_9;
    prometheus::Gauge& joint_10;
    prometheus::Gauge& joint_11;
    prometheus::Gauge& joint_12;
    prometheus::Gauge& joint_13;
    prometheus::Gauge& joint_14;
    prometheus::Gauge& joint_15;
    prometheus::Gauge& joint_16;
    prometheus::Gauge& joint_17;
    prometheus::Gauge& joint_18;
    prometheus::Gauge& joint_19;
    prometheus::Gauge& joint_20;
};

#endif