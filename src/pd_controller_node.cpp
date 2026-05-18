/*
    PD Controller Node for Inverse Dynamics Linearization
    Outputs the auxiliary control input 'y' (published as desired_joint_accelerations)
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <vector>

class PDControllerNode : public rclcpp::Node
{
public:
    PDControllerNode() : Node("pd_controller_node")
    {
        // Declare parameters with default values based on critical damping (wn = 10, zeta = 1)
        this->declare_parameter<std::vector<double>>("qd", {1.0, 1.0}); // Default desired position
        this->declare_parameter<double>("kp", 100.0);
        this->declare_parameter<double>("kd", 20.0);

        // Get parameters
        std::vector<double> qd_param = this->get_parameter("qd").as_double_array();
        qd_ = Eigen::VectorXd::Map(qd_param.data(), qd_param.size());
        
        kp_ = this->get_parameter("kp").as_double();
        kd_ = this->get_parameter("kd").as_double();

        // Create publisher for the auxiliary control input 'y' (desired accelerations)
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("desired_joint_accelerations", 1);
        
        // Create subscription to current joint states
        subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 1, std::bind(&PDControllerNode::joint_states_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "PD Controller Node initialized with Kp=%.2f, Kd=%.2f", kp_, kd_);
    }

private:
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        Eigen::VectorXd q(2);
        Eigen::VectorXd q_dot(2);

        // Extract joint positions and velocities
        auto joint1_index = std::find(msg->name.begin(), msg->name.end(), "joint_1") - msg->name.begin();
        auto joint2_index = std::find(msg->name.begin(), msg->name.end(), "joint_2") - msg->name.begin();

        if (static_cast<size_t>(joint1_index) < msg->name.size() && 
            static_cast<size_t>(joint2_index) < msg->name.size())
        {
            q(0) = msg->position[joint1_index];
            q(1) = msg->position[joint2_index];
            q_dot(0) = msg->velocity[joint1_index];
            q_dot(1) = msg->velocity[joint2_index];

            // Calculate the stabilizing linear control law: y = Kp*(qd - q) - Kd*q_dot
            // (Assuming q_ddot_d = 0 and q_dot_d = 0 as per instructions)
            Eigen::VectorXd y = kp_ * (qd_ - q) - kd_ * q_dot;

            // Publish 'y' to the topic expected by the dynamics cancellation node
            auto y_msg = std_msgs::msg::Float64MultiArray();
            y_msg.data.assign(y.data(), y.data() + y.size());
            publisher_->publish(y_msg);
        }
    }

    // Member variables
    Eigen::VectorXd qd_;
    double kp_;
    double kd_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PDControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}