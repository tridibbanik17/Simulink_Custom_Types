#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "custom_messages/msg/nums.hpp"
#include "custom_messages/msg/nums_vectorized.hpp"

using namespace std::chrono_literals;

class NumsDecoder : public rclcpp::Node
{
public:
    NumsDecoder() : Node("nums_decoder")
    {
        subscription_ =
            this->create_subscription<custom_messages::msg::Nums>(
                "nums_topic",
                10,
                [this](const custom_messages::msg::Nums::SharedPtr msg)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "Received custom message! %ld and %ld", msg->num1, msg->num2);
                });
    }

private:
    rclcpp::Subscription<custom_messages::msg::Nums>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NumsDecoder>());
    rclcpp::shutdown();
    return 0;
}