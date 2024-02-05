#include <prometheus/exposer.h>
#include <prometheus/gauge.h>
#include <prometheus/counter.h>
#include <prometheus/registry.h>

#include <array>
#include <chrono>   
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "tachimawari_interfaces/msg/currentjoints.hpp"

using std::placeholders::_1;

prometheus::Exposer exposer{"127.0.0.1:6000"}; //localhost
auto registry = std::make_shared<prometheus::Registry>();
auto& Joint = prometheus::BuildGauge()
                .Name("current_joint")
                .Help("Joints that will be monitored")
                .Register(*registry);
auto& joint_1 = Joint.Add({{"id", "1"}});
auto& joint_2 = Joint.Add({{"id", "2"}});

class JointSubscriber : public rclcpp::Node
{
public:
    JointSubscriber()
    : Node("joint/current_joints")
    {
        subscription_ = this->create_subscription<tachimawari_interfaces::msg::CurrentJoints>(
            "Joint", 10, std::bind(&JointSubscriber::topic_callback, this, _1));
    }
private:
    void topic_callback(const tachimawari_interfaces::msg::CurrentJoints & msg) const
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Got msg");
        joint_1.Set(msg[0].position);
        joint_2.Set(msg[1].position);
        joint_3.Set(msg[1].position);
        joint_4.Set(msg[1].position);
        joint_5.Set(msg[1].position);
        joint_6.Set(msg[1].position);
        joint_7.Set(msg[1].position);
        joint_8.Set(msg[1].position);
        joint_9.Set(msg[1].position);
        joint_10.Set(msg[9].position);
        joint_11.Set(msg[10].position);
        
    }
    rclcpp::Subscription<tachimawari_interfaces::msg::Joint>::SharedPtr subscription_;
};

int main(int argc, char * argv[]){
    exposer.RegisterCollectable(registry);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointSubscriber>());
    rclcpp::shutdown();
}