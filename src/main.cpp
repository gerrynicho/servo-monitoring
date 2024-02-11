#include <prometheus/exposer.h>
#include <prometheus/gauge.h>
#include <prometheus/registry.h>

#include <array>
#include <chrono>   
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "tachimawari_interfaces/msg/current_joints.hpp"
using CurrentJoints = tachimawari_interfaces::msg::CurrentJoints;

using std::placeholders::_1;

prometheus::Exposer exposer{"127.0.0.1:5001"}; //localhost
std::shared_ptr<prometheus::Registry> registry = 
    std::make_shared<prometheus::Registry>();
prometheus::Family<prometheus::Gauge>& Joints = prometheus::BuildGauge()
                .Name("current_joints")
                .Help("Joints that will be monitored")
                .Register(*registry);
prometheus::Gauge& joint_1 = Joints.Add({{"id", "1"}});
prometheus::Gauge& joint_2 = Joints.Add({{"id", "2"}});
prometheus::Gauge& joint_3 = Joints.Add({{"id", "3"}});
prometheus::Gauge& joint_4 = Joints.Add({{"id", "4"}});
prometheus::Gauge& joint_5 = Joints.Add({{"id", "5"}});
prometheus::Gauge& joint_6 = Joints.Add({{"id", "6"}});
prometheus::Gauge& joint_7 = Joints.Add({{"id", "7"}});
prometheus::Gauge& joint_8 = Joints.Add({{"id", "8"}});
prometheus::Gauge& joint_9 = Joints.Add({{"id", "9"}});
prometheus::Gauge& joint_10 = Joints.Add({{"id", "10"}});
prometheus::Gauge& joint_11 = Joints.Add({{"id", "11"}});
prometheus::Gauge& joint_12 = Joints.Add({{"id", "12"}});
prometheus::Gauge& joint_13 = Joints.Add({{"id", "13"}});
prometheus::Gauge& joint_14 = Joints.Add({{"id", "14"}});
prometheus::Gauge& joint_15 = Joints.Add({{"id", "15"}});
prometheus::Gauge& joint_16 = Joints.Add({{"id", "16"}});
prometheus::Gauge& joint_17 = Joints.Add({{"id", "17"}});
prometheus::Gauge& joint_18 = Joints.Add({{"id", "18"}});
prometheus::Gauge& joint_19 = Joints.Add({{"id", "19"}});
prometheus::Gauge& joint_20 = Joints.Add({{"id", "20"}});

class joint_subscriber : public rclcpp::Node
{
public:
    joint_subscriber()
    : Node("joint_subscriber")
    {
        subscription = this->create_subscription<CurrentJoints>(
            "joint/current_joints", 10, std::bind(&joint_subscriber::topic_callback, this, _1));
    }
private:
    void topic_callback(const CurrentJoints & current_joints_msg) const
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Got msg");
        auto & msg = current_joints_msg.joints;
        // for(auto x:msg){
        //     RCLCPP_INFO_STREAM(this->get_logger(), x.position);
        // }
        joint_1.Set(msg[0].position);
        joint_2.Set(msg[1].position);
        joint_3.Set(msg[2].position);
        joint_4.Set(msg[3].position);
        joint_5.Set(msg[4].position);
        joint_6.Set(msg[5].position);
        joint_7.Set(msg[6].position);
        joint_8.Set(msg[7].position);
        joint_9.Set(msg[8].position);
        joint_10.Set(msg[9].position);
        joint_11.Set(msg[10].position);
        joint_12.Set(msg[11].position);
        joint_13.Set(msg[12].position);
        joint_14.Set(msg[13].position);
        joint_15.Set(msg[14].position);
        joint_16.Set(msg[15].position);
        joint_17.Set(msg[16].position);
        joint_18.Set(msg[17].position);
        joint_19.Set(msg[18].position);
        joint_20.Set(msg[19].position);
    }
    rclcpp::Subscription<CurrentJoints>::SharedPtr subscription;
};

int main(int argc, char * argv[]){
    exposer.RegisterCollectable(registry);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<joint_subscriber>());
    rclcpp::shutdown();
}