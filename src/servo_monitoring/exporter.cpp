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
    joint_1.Set(new_joints[0].position);
    joint_2.Set(new_joints[1].position);
    joint_3.Set(new_joints[2].position);
    joint_4.Set(new_joints[3].position);
    joint_5.Set(new_joints[4].position);
    joint_6.Set(new_joints[5].position);
    joint_7.Set(new_joints[6].position);
    joint_8.Set(new_joints[7].position);
    joint_9.Set(new_joints[8].position);
    joint_10.Set(new_joints[9].position);
    joint_11.Set(new_joints[10].position);
    joint_12.Set(new_joints[11].position);
    joint_13.Set(new_joints[12].position);
    joint_14.Set(new_joints[13].position);
    joint_15.Set(new_joints[14].position);
    joint_16.Set(new_joints[15].position);
    joint_17.Set(new_joints[16].position);
    joint_18.Set(new_joints[17].position);
    joint_19.Set(new_joints[18].position);
    joint_20.Set(new_joints[19].position);
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
                   .Name("consuming_current_joints")
                   .Help("Joints that will be monitored")
                   .Register(*registry)),
      joint_1(GaugeBuilder.Add({{"id", "1"}})),
      joint_2(GaugeBuilder.Add({{"id", "2"}})),
      joint_3(GaugeBuilder.Add({{"id", "3"}})),
      joint_4(GaugeBuilder.Add({{"id", "4"}})),
      joint_5(GaugeBuilder.Add({{"id", "5"}})),
      joint_6(GaugeBuilder.Add({{"id", "6"}})),
      joint_7(GaugeBuilder.Add({{"id", "7"}})),
      joint_8(GaugeBuilder.Add({{"id", "8"}})),
      joint_9(GaugeBuilder.Add({{"id", "9"}})),
      joint_10(GaugeBuilder.Add({{"id", "10"}})),
      joint_11(GaugeBuilder.Add({{"id", "11"}})),
      joint_12(GaugeBuilder.Add({{"id", "12"}})),
      joint_13(GaugeBuilder.Add({{"id", "13"}})),
      joint_14(GaugeBuilder.Add({{"id", "14"}})),
      joint_15(GaugeBuilder.Add({{"id", "15"}})),
      joint_16(GaugeBuilder.Add({{"id", "16"}})),
      joint_17(GaugeBuilder.Add({{"id", "17"}})),
      joint_18(GaugeBuilder.Add({{"id", "18"}})),
      joint_19(GaugeBuilder.Add({{"id", "19"}})),
      joint_20(GaugeBuilder.Add({{"id", "20"}}))
{
    subscription = node->create_subscription<ConsumingCurrentJoints>
                ("joint/consuming_current_joints", 
                 10,
                 std::bind(&Exporter::topic_callback, this, _1));
}


std::shared_ptr<prometheus::Registry> Exporter::get_registry()
{
    return registry;
}