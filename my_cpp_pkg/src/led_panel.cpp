#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/led_states.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

class LEDPanelNode : public rclcpp::Node
{
public:
    LEDPanelNode() : Node("led_panel")
    {
        this->declare_parameter("led_states", std::vector<int64_t>{0, 0, 0});
        led_states_ = this->get_parameter("led_states").as_integer_array();
        publisher_ = this->create_publisher<my_robot_interfaces::msg::LEDStates>("led_states", 10);

        server_ = this->create_service<my_robot_interfaces::srv::SetLED>(
            "set_led",
            std::bind(&LEDPanelNode::setLED, this, std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&LEDPanelNode::publishLEDStates, this));

        RCLCPP_INFO(this->get_logger(), "LED panel cpp started");
    }

private:
    void setLED(const my_robot_interfaces::srv::SetLED::Request::SharedPtr request,
                const my_robot_interfaces::srv::SetLED::Response::SharedPtr response)
    {
        if (request->data)
        {
            led_states_[2] = 1;
        }
        else
        {
            led_states_[2] = 0;
        }
        response->set__success(true);
        RCLCPP_INFO(this->get_logger(), "setting LED: %d", request->data);
        publishLEDStates();
    }

    void publishLEDStates()
    {
        auto msg = my_robot_interfaces::msg::LEDStates();
        msg.data = led_states_;
        publisher_->publish(msg);
    }

    std::vector<int64_t> led_states_;
    rclcpp::Publisher<my_robot_interfaces::msg::LEDStates>::SharedPtr publisher_;
    rclcpp::Service<my_robot_interfaces::srv::SetLED>::SharedPtr server_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LEDPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}