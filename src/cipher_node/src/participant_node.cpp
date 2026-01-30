#include <string>
#include <cctype>
#include <chrono> // Requirement for time in wait_for_service

#include "rclcpp/rclcpp.hpp"
#include "cipher_interfaces/msg/cipher_message.hpp"
#include "cipher_interfaces/srv/cipher_answer.hpp"

using std::placeholders::_1;

class ParticipantNode : public rclcpp::Node {
    public:
        ParticipantNode() : Node("participant_node") {
            subscription_ = this->create_subscription<cipher_interfaces::msg::CipherMessage>(
                "cipher_topic", 10, std::bind(&ParticipantNode::topic_callback, this, _1));
            client_ = this->create_client<cipher_interfaces::srv::CipherAnswer>("cipher_check");
        }
    private:
        void topic_callback(const cipher_interfaces::msg::CipherMessage msg) {
            // Get connection
            while (!client_->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    break;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }

            // Create result and test caesar_unshifter
            auto req = std::make_shared<cipher_interfaces::srv::CipherAnswer::Request>();
            req->answer = caesar_unshifter(msg.message, msg.key);
            RCLCPP_INFO(this->get_logger(), "Undecoded message: %s", req->answer.c_str());
            client_->async_send_request(req,
                [this](rclcpp::Client<cipher_interfaces::srv::CipherAnswer>::SharedFuture future) {
                    if (future.get()->result) {
                        RCLCPP_INFO(this->get_logger(), "Participant decoded successfully!");
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Wrong decoding -> Change algorithm");
                    }
                });
        }

        std::string caesar_unshifter(const std::string& message, uint8_t shift) {
            std::string decoded;
            decoded.reserve(message.size());
            
            for (char c : message) {
                if (std::isalpha(c)) {
                    // Determine if uppercase or lowercase
                    char base = std::isupper(c) ? 'A' : 'a';
                    
                    // Shift backwards (subtract the shift value)
                    // Add 26 before modulo to handle negative results correctly
                    char decoded_char = base + ((c - base - shift + 26) % 26);
                    decoded += decoded_char;
                } else {
                    // Non-alphabetic characters remain unchanged
                    decoded += c;
                }
            }
            
            return decoded;
        }

        rclcpp::Subscription<cipher_interfaces::msg::CipherMessage>::SharedPtr subscription_;
        rclcpp::Client<cipher_interfaces::srv::CipherAnswer>::SharedPtr client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParticipantNode>());
    rclcpp::shutdown();

    return 0;
}