#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter")
    {
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
            "number", 10,
            std::bind(&NumberCounterNode::updateCounter, this, std::placeholders::_1));

        server_ = this->create_service<example_interfaces::srv::SetBool>(
            "reset_counter",
            std::bind(&NumberCounterNode::reset, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Number counter cpp started");
    }

private:
    void updateCounter(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        counter_ += msg->data;

        auto new_msg = example_interfaces::msg::Int64();
        new_msg.data = counter_;
        publisher_->publish(new_msg);
    }

    void reset(const example_interfaces::srv::SetBool::Request::SharedPtr request,
               const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if (request->data) {
            counter_ = 0;

            response->set__success(true);
            RCLCPP_INFO(this->get_logger(), "Number counter reset done");
        } else {
            response->set__success(false);
            response->set__message("the value set has to be true");
        }
    }

    int counter_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}