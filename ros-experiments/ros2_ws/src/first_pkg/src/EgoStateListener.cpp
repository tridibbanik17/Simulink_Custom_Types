#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

// Subscribes to the /ego_odom topic that the MATLAB sensor-fusion script
// publishes (nav_msgs/Odometry) and prints the ego vehicle's current
// position and velocity.
class EgoStateListener : public rclcpp::Node
{
public:
    
    // EgoStateListener() constructor takes no inputs and return nothing
    // It names the node ego_state_listener and creates a subscription to /ego_odom topic.
    EgoStateListener() : Node("ego_state_listener")
    {
        // create_subscription is a template function
        // subscription handles Odometry messages, which is nav_msgs::msg::Odometry 
        // takes 3 inputs : topic name is /ego_odom, queue depth (size of a waiting line for messages) is 10, third input is a lambda to be called each time a message is received.
        // returns rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr, which is the subscription object, stored in subscription_
        subscription_ =
            this->create_subscription<nav_msgs::msg::Odometry>(
                "/ego_odom",
                10,
                // function invoked by ROS on each incoming message
                // takes 1 input: const nav_msgs::msg::Odometry::SharedPtr msg, which is a pointer to the received message
                // returns nothing
                // this lambda function reads position & velocity and logs them
                // -------------------------------------------------------------
                [this](const nav_msgs::msg::Odometry::SharedPtr msg)
                {
                    const auto &p = msg->pose.pose.position;  // x, y, z
                    const auto &v = msg->twist.twist.linear;  // vx, vy, vz

                    RCLCPP_INFO(
                        this->get_logger(),
                        "[%s] pos = [%.3f, %.3f, %.3f]  vel = [%.3f, %.3f, %.3f]",
                        msg->header.frame_id.c_str(),
                        p.x, p.y, p.z,
                        v.x, v.y, v.z);
                });
    }           

// purpose of this is to keep the subscription alive as long as the node is alive
// subscription_ variable is a private member variable of the class
// subscription_ is a pointer to a Subscription object
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

// starts ROS2, creates the node, spins to process messages until shutdown, then cleans up.
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                            
    rclcpp::spin(std::make_shared<EgoStateListener>());   
    rclcpp::shutdown();                                  
    return 0;
}
