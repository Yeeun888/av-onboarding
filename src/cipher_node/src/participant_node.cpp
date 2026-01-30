#include "rclcpp/rclcpp.hpp"
// #include "cipher_interfaces/include/cipher_interfaces/cipher_interfaces/msg/cipher_message.hpp"
#include "cipher_interfaces/msg/cipher_message.hpp"

using std::placeholders::_1;

class ParticipantNode : public rclcpp::Node {
    public:
        ParticipantNode() : Node("participant_node") {
            subscription_ = this->create_subscription<cipher_interfaces::msg::CipherMessage>(
                "cipher_topic", 10, std::bind(&ParticipantNode::topic_callback, this, _1));
        }
    private:
        void topic_callback(const cipher_interfaces::msg::CipherMessage msg) {
            RCLCPP_INFO(this->get_logger(), "Msg: '%s' Shift: '%d'", msg.message.c_str(), msg.key);
        }

        rclcpp::Subscription<cipher_interfaces::msg::CipherMessage>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParticipantNode>());
    rclcpp::shutdown();

    return 0;
}