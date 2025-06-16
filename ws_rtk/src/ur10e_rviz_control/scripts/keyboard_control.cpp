#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <iostream>
#include <string>
#include <vector>

class KeyboardControl : public rclcpp::Node {
public:
    KeyboardControl() : Node("keyboard_control"), active_joint_(0) {
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        joint_state_.name = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        };
        joint_state_.position.resize(6, 0.0);

        std::cout << "Use arrow keys: ←/→ to switch joint, ↑/↓ to move. Press 'q' to quit.\n";

        loop();
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    sensor_msgs::msg::JointState joint_state_;
    int active_joint_;

    int get_key() {
        struct termios oldt, newt;
        int ch;
        int oldf;

        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        struct timeval tv {0, 0};
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);

        int ret = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv);

        if (ret > 0) {
            ch = getchar();
            if (ch == 27) {  // Escape
                if (getchar() == '[') {
                    ch = getchar();
                    // return special key code
                    switch (ch) {
                        case 'A': return 'U';  // Up
                        case 'B': return 'D';  // Down
                        case 'C': return 'R';  // Right
                        case 'D': return 'L';  // Left
                    }
                }
            }
            return ch;
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return -1;
    }

    void loop() {
        rclcpp::Rate rate(10);
        while (rclcpp::ok()) {
            int key = get_key();

            if (key == 'q') break;
            if (key == 'R') {  // →
                active_joint_ = (active_joint_ + 1) % 6;
                std::cout << "Selected joint " << active_joint_ << ": " << joint_state_.name[active_joint_] << "\n";
            } else if (key == 'L') {  // ←
                active_joint_ = (active_joint_ - 1 + 6) % 6;
                std::cout << "Selected joint " << active_joint_ << ": " << joint_state_.name[active_joint_] << "\n";
            } else if (key == 'U') {  // ↑
                joint_state_.position[active_joint_] += 0.05;
                std::cout << joint_state_.name[active_joint_] << ": " << joint_state_.position[active_joint_] << "\n";
            } else if (key == 'D') {  // ↓
                joint_state_.position[active_joint_] -= 0.05;
                std::cout << joint_state_.name[active_joint_] << ": " << joint_state_.position[active_joint_] << "\n";
            }

            joint_state_.header.stamp = this->now();
            joint_pub_->publish(joint_state_);
            rate.sleep();
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardControl>();
    rclcpp::shutdown();
    return 0;
}

