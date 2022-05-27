#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("battery")
    {
        RCLCPP_INFO(this->get_logger(), "BatteryNode Cpp Node");
        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&BatteryNode::updateBattery, this));
    }

private:
    void updateBattery()
    {
        if (is_charging_) {
            battery_state_ += 1.0/6;
        } else {
            battery_state_ -= 1.0/4;
        }

        RCLCPP_INFO(this->get_logger(), "battery state %.2f", battery_state_);

        if (battery_state_ >= 1.0) {
            battery_state_ = 1.0;
            if (!is_calling_server_) {
                threads_.empty();
                threads_.push_back(std::thread(std::bind(&BatteryNode::setLEDState, this, false)));
            }
        } else if (battery_state_ <= 0.0) {
            battery_state_ = 0.0;
            if (!is_calling_server_) {
                threads_.empty();
                threads_.push_back(std::thread(std::bind(&BatteryNode::setLEDState, this, true)));
            }
        }
    }

    void setLEDState(bool on) {
        RCLCPP_INFO(this->get_logger(), "setting battery charge %d", on);

        if (is_calling_server_) {
            RCLCPP_INFO(this->get_logger(), "setting battery already in process");
            return;
        }

        is_calling_server_ = true;

        auto client = this->create_client<my_robot_interfaces::srv::SetLED>("set_led");
        while(!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "waiting for the server to be up");
        }

        auto request = std::make_shared<my_robot_interfaces::srv::SetLED::Request>();
        request->data = on;

        auto future = client->async_send_request(request);

        try 
        {
            auto response  = future.get();
            if (response->success) {
                is_charging_ = on;
                RCLCPP_INFO(this->get_logger(), "set led success, new state is %d", is_charging_);
            } else {
                RCLCPP_INFO(this->get_logger(), "set led failed");
            }
            is_calling_server_ = false;
        }
        catch (const std::exception &e)
        {
            is_calling_server_ = false;
            RCLCPP_INFO(this->get_logger(), "service call failed");
        } 
    
    }
    rclcpp::TimerBase::SharedPtr timer_;
    double battery_state_ = 1.0;
    bool is_charging_;
    bool is_calling_server_ = false;
    std::vector<std::thread> threads_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}