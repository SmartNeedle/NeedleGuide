#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <sstream>
#include <iomanip>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;
using std::cout; using std::cerr;
using std::endl; using std::string;
using std::ifstream;

using namespace std::chrono_literals;


/** 
 * Get and publish depth and rotation values from sensors connected to Arduino board if in real mode. If in simulation mode, publish increasing values starting from 0. 
 **/
 
class SensorsNode : public rclcpp::Node
{
public:

    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using Float64 = std_msgs::msg::Float64;

    SensorsNode()
        : Node("sensors_node")
    {
    	
        // Needle pose publisher
        pose_publisher = this->create_publisher<PoseStamped>("/needle/state/pose", 10);

        // Arduino sensors subscriber
        yAndThetaArduino_subscriber = this->create_subscription<std_msgs::msg::String>(
            "yAndThetaArduinoValues",
            10,
            std::bind(&SensorsNode::arduinoValues_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Needle sensors ready.");
        
    }

private:
    
    void arduinoValues_callback(const std_msgs::msg::String::SharedPtr msg) 
    {
        //RCLCPP_INFO(this->get_logger(), "I heard in ROS2: '%s'", msg->data.c_str());
        
        auto message = PoseStamped();
        auto position = geometry_msgs::msg::Point();
        auto orientation = geometry_msgs::msg::Quaternion();
	
	
	std::string yAndTheta;
	
	yAndTheta = msg->data.c_str();
        //std::cout << yAndTheta << std::endl;
	
    	// Retrieve depth and rotation separated by a semicolon
    	size_t pos = yAndTheta.rfind(";"); 
	std::string depth_value = yAndTheta.substr(0, pos);
	std::string rotation_value = yAndTheta.substr(pos + 1);
	//std::cout << depth_value << std::endl;
	//std::cout << rotation_value << std::endl;
	
	// convert string to float
    	y = std::stod(depth_value);
    	theta = std::stod(rotation_value);	
	//std::cout << theta << std::endl;
	
        position.set__y(y); // corresponds to depth
        orientation.set__x(0); // Not used 
        orientation.set__y(theta); // corresponds to theta
        orientation.set__z(0); // Not used
        orientation.set__w(0); // Not used
        message.pose.set__position(position);
        message.pose.set__orientation(orientation);
        message.header.set__stamp(this->get_clock()->now());
        pose_publisher->publish(message);
    
    }

    
    rclcpp::TimerBase::SharedPtr pose_timer_;
    rclcpp::Publisher<PoseStamped>::SharedPtr pose_publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yAndThetaArduino_subscriber;
    double y;
    double theta;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorsNode>());
    rclcpp::shutdown();
    return 0;
}
