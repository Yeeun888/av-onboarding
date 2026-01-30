#include <thread>
#include <iostream>
#include <string>
#include <cctype>

#include "rclcpp/rclcpp.hpp"
#include "cipher_interfaces/msg/cipher_message.hpp"
#include "cipher_interfaces/srv/cipher_answer.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class CipherNode : public rclcpp::Node {
    public:
        CipherNode() : Node("cipher_node"), frame_count_(0) {
            publisher_ = this->create_publisher<cipher_interfaces::msg::CipherMessage>("cipher_topic", 10);
            cipher_check_ = this->create_service<cipher_interfaces::srv::CipherAnswer>("cipher_check", 
                std::bind(&CipherNode::verify_answer, this, std::placeholders::_1, std::placeholders::_2));

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
                std::cout << "Enter Message ('quit' / 'CTRL-D' to exit): ";
                std::getline(std::cin, msg);

                if (msg == "quit") {
                    rclcpp::shutdown();
                    break;
                }

                std::cout << "Enter shift: ";
                std::getline(std::cin, shift);

                // Publishing the message taken
                auto message = cipher_interfaces::msg::CipherMessage();
                message.header.frame_id = std::to_string(frame_count_);
                message.header.set__stamp(this->now());
                message.message = caesar_shifter(msg, std::stoi(shift));
                message.key = std::stoi(shift);

                // Keep track of last encoded message
                last_published_unencrypted_ = msg;

                RCLCPP_INFO(this->get_logger(), "Publishing '%s'", message.message.c_str());
                publisher_->publish(message);
            }
        }

        void verify_answer(const std::shared_ptr<cipher_interfaces::srv::CipherAnswer::Request> req,
            std::shared_ptr<cipher_interfaces::srv::CipherAnswer::Response> res) {
                // TODO: Uncomment for final
                // RCLCPP_INFO(this->get_logger(), "Last Pub: '%s' Received: %s", last_published_unencrypted_.c_str(), req->answer.c_str());
                if (req->answer == last_published_unencrypted_) {
                    res->result = true;
                } else {
                    res->result = false;
                }
            }

        std::string caesar_shifter(const std::string& message, uint8_t shift) {
            std::string encoded;
            encoded.reserve(message.size());
            
            for (char c : message) {
                if (std::isalpha(c)) {
                    // Determine if uppercase or lowercase
                    char base = std::isupper(c) ? 'A' : 'a';
                    
                    // Shift forwards (add the shift value)
                    char encoded_char = base + ((c - base + shift) % 26);
                    encoded += encoded_char;
                } else {
                    // Non-alphabetic characters remain unchanged
                    encoded += c;
                }
            }
            
            return encoded;
        }

        rclcpp::Publisher<cipher_interfaces::msg::CipherMessage>::SharedPtr publisher_;
        size_t frame_count_;
        std::thread input_thread_;
        std::string last_published_unencrypted_;

        rclcpp::Service<cipher_interfaces::srv::CipherAnswer>::SharedPtr cipher_check_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CipherNode>());
    rclcpp::shutdown();
    return 0;
}
