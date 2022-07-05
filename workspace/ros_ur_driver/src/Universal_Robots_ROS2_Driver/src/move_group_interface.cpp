#include <memory>
#include <string> 
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
using std::placeholders::_1;
using namespace std;

class MinimalSubscriber
{
private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(node->get_logger(), "I heard: '%s'", msg->data.c_str());

        auto const target_pose = [](string str) {
            geometry_msgs::msg::Pose target_pose;
            string::size_type sz;
            target_pose.position.x = stod(str,&sz);
            str = str.substr(sz);
            target_pose.position.y = stod(str,&sz);
            str = str.substr(sz);
            target_pose.position.z = stod(str,&sz);
            str = str.substr(sz);
            target_pose.orientation.x = stod(str,&sz);
            str = str.substr(sz);
            target_pose.orientation.y = stod(str,&sz);
            str = str.substr(sz);
            target_pose.orientation.z = stod(str,&sz);
            str = str.substr(sz);
            target_pose.orientation.w = stod(str,&sz);
            return target_pose;
        }(msg->data);
        
        this->move_group_interface->setPoseTarget(target_pose);
        RCLCPP_INFO(node->get_logger(), "Moving to: '%.2lf %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf'",
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z,
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w);

        // Create a plan to that target pose
        auto const [success, plan] = [&] {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(this->move_group_interface->plan(msg));
            return std::make_pair(ok, msg);
        }();

        // Execute the plan
        if (success) {
            this->move_group_interface->execute(plan);
        } else {
            RCLCPP_ERROR(node->get_logger(), "Planing failed!");
        }
    }
public:
    shared_ptr<rclcpp::Node> node;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    moveit::planning_interface::MoveGroupInterface *move_group_interface;
    MinimalSubscriber()
    {
        node = std::make_shared<rclcpp::Node>("moveit2_control", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
        subscription_ = node->create_subscription<std_msgs::msg::String>("moveit2_control_target_pose", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
        move_group_interface = new moveit::planning_interface::MoveGroupInterface(node, "ur_manipulator");
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto ms = std::make_shared<MinimalSubscriber>();
    rclcpp::spin(ms->node);
    rclcpp::shutdown();
    return 0;
}