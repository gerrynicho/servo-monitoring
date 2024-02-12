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

class exporter : public rclcpp::Node
{
public:
    exporter();
    prometheus::Exposer exposer;
    std::shared_ptr<prometheus::Registry> get_registry();
private:
    void topic_callback(const ConsumingCurrentJoints & incoming_message) const;
    rclcpp::Node::SharedPtr node;
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