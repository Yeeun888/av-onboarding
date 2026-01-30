#include <thread>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
// #include "cipher_interfaces/include/cipher_interfaces/cipher_interfaces/msg/cipher_message.hpp"
#include "cipher_interfaces/msg/cipher_message.hpp"

class CipherNode : public rclcpp::Node {
    public:
        CipherNode() : Node("cipher_node"), frame_count_(0) {
            publisher_ = this->create_publisher<cipher_interfaces::msg::CipherMessage>("cipher_topic", 10);
            input_thread_ = std::thread(&CipherNode::get_user_input, this);
        }    

        ~CipherNode() {
            if (input_thread_.joinable()) {
                input_thread_.join();
            }
        }
    
    private:
        void get_user_input() {
            std::string msg;
            std::string shift;

            while(rclcpp::ok()) {
                std::cout << "Enter Message ('quit' to exit): ";
                std::getline(std::cin, msg);

                if (msg == "quit") {
                    rclcpp::shutdown();
                    break;
                }

                std::cout << "Enter shift: ";
                std::getline(std::cin, shift);

                // Publishing the message taken
                auto message = cipher_interfaces::msg::CipherMessage();
                message.header.frame_id = frame_count_;
                message.header.set__stamp(this->now());
                message.message = msg;
                message.key = std::stoi(shift);

                // Info for debugging
                // TODO: Remove later
                RCLCPP_INFO(this->get_logger(), "Publishing '%s'", message.message.c_str());
                publisher_->publish(message);
            }
        }

        rclcpp::Publisher<cipher_interfaces::msg::CipherMessage>::SharedPtr publisher_;
        size_t frame_count_;
        std::thread input_thread_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CipherNode>());
    rclcpp::shutdown();
    return 0;
}
