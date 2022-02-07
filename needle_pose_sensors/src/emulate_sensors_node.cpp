#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
//#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>
#include <fstream>
#include "std_msgs/msg/string.hpp"
#include <stdio.h>
#include <sstream>
#include <iomanip>
#include <vector>

using namespace std::chrono_literals;
using std::cout; using std::cerr;
using std::endl; using std::string;
using std::ifstream;

using namespace std::chrono_literals;

void read_sample_values();
std::vector<std::string> vecOfStr;
int idx =0;
/**
 * Emulate the needle depth and rotation sensors, use until
 * real sensors have been fully integrated. Also exposes topics
 * to modify virtual needle pose.
 **/
 
class EmulateSensorsNode : public rclcpp::Node
{
public:
// may throw PackageNotFoundError exception

	//std::string package_share_directory = 	ament_index_cpp::get_package_share_directory('adaptive_guide');

    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using Float64 = std_msgs::msg::Float64;

    EmulateSensorsNode()
        : Node("emulate_sensors_node")
    {
        // Needle pose publisher
        pose_publisher = this->create_publisher<PoseStamped>("needle/state/pose", 10);
        pose_timer_ = this->create_wall_timer(
            50ms, std::bind(&EmulateSensorsNode::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Needle sensor emulators ready.");
        
        // Virtual sensors
        
        std::vector<std::string> vecOfStr;

        // Needle control subscribers
        yAndTheta_subscriber = this->create_subscription<std_msgs::msg::String>(
            "needle/emulated/yAndTheta",
            10,
            std::bind(&EmulateSensorsNode::yAndTheta_sub_callback, this, std::placeholders::_1));

	// Needle depth and rotation publisher
        yAndTheta_publisher = this->create_publisher<std_msgs::msg::String>("needle/emulated/yAndTheta", 10);
        timer_ = this->create_wall_timer(
            50ms, std::bind(&EmulateSensorsNode::yAndTheta_pub_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Needle control emulators ready.");
    }

private:
    void timer_callback()
    { 	
        auto message = PoseStamped();
        auto position = geometry_msgs::msg::Point();
        auto orientation = geometry_msgs::msg::Quaternion();

        position.set__y(y); // corresponds to depth 
        orientation.set__x(theta); // corresponds to theta
	orientation.set__y(0); // Not used
	orientation.set__z(0); // Not used
	orientation.set__w(0); // Not used
        message.pose.set__position(position);
        message.pose.set__orientation(orientation);
        message.header.set__stamp(this->get_clock()->now());
        pose_publisher->publish(message);
    }

    void yAndTheta_sub_callback(const std_msgs::msg::String::SharedPtr msg) 
    {
    	yAndTheta = msg->data;
    	// Retrieve depth and rotation separated by a semicolon
    	size_t pos = yAndTheta.rfind(";"); 
	std::string depth_value = yAndTheta.substr(0, pos);
	std::string rotation_value = yAndTheta.substr(pos + 1);
	//std::cout << depth_value << std::endl;
	//std::cout << rotation_value << std::endl;
	//RCLCPP_INFO(this->get_logger(), "Measured values are: depth: %s and rotation: %s", 	depth_value.c_str(),rotation_value.c_str());
	// convert string to float
    	y = std::stof(depth_value);	
    	theta = std::stof(rotation_value);	
    }
    
    
    
    void read_sample_values()
    {
    
    	std::vector<std::string> vecOfStr;
	const std::string filename = "/home/snr/new_ws/src/NeedleGuide/SampleSensorsValues.log";
  	std::ifstream fs;
  	fs.open(filename.c_str(), std::fstream::in);
  	std::string temp_string;
  	
  	if(fs.is_open())
  	{
  		
  		while (std::getline(fs, temp_string))
    		{
        		if(temp_string.size() > 0)
        		{
            			vecOfStr.push_back(temp_string);
            			//std::cout << temp_string << std::endl;
			} 
    		}
   
	}

    	else
  	{
  	  	std::cout << "Could open file" << std::endl;
  	}
    }
    
    // For simulation
    void yAndTheta_pub_callback()
    {
    
    	// For simulation with sample values fro potentiometer and rotary encoder
    	
    	auto message = std_msgs::msg::String();
	std::string lastline;
	lastline = vecOfStr[idx];
        std::cout << lastline << std::endl;
        message.data = lastline;
	//RCLCPP_INFO(this->get_logger(), "Publishing depth;rotation value: %s", 		   message.data.c_str());
	yAndTheta_publisher->publish(message);  
	idx++;
    }
    
    /*
    
    // For Real hardware
    void yAndTheta_pub_callback()
    {
    	auto message = std_msgs::msg::String();
	const std::string filename = "/home/snr/new_ws/src/NeedleGuide/Test.log";
  	std::ifstream fs;
  	fs.open(filename.c_str(), std::fstream::in);
  	if(fs.is_open())
  	{
    		//Got to the last character before EOF
    		fs.seekg(-1, std::ios_base::end);
    		if(fs.peek() == '\n')
    		{
      			//Start searching for \n occurrences
      			fs.seekg(-1, std::ios_base::cur);
     			int i = fs.tellg();
      			for(i;i > 0; i--)
      			{
        			if(fs.peek() == '\n')
        			{
          				//Found
          				fs.get();
          				break;
        			}	
        			//Move one character back
        			fs.seekg(i, std::ios_base::beg);
      			}
      		}
      		std::string lastline;
    		getline(fs, lastline);
    		//std::cout << lastline << std::endl;
    		
		
		message.data = lastline;
		//RCLCPP_INFO(this->get_logger(), "Publishing depth;rotation value: %s", 		   message.data.c_str());
		yAndTheta_publisher->publish(message);

	}

    	else
  	{
  	  	std::cout << "Could open file" << std::endl;
  	}
    }*/
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr pose_timer_;
    rclcpp::Publisher<PoseStamped>::SharedPtr pose_publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yAndTheta_subscriber;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr yAndTheta_publisher;
    std::string yAndTheta;
    float y;
    float theta;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    //read_sample_values();
    std::cout << "Reading sample values" << std::endl;
    
    const std::string filename = "/home/snr/new_ws/src/NeedleGuide/SampleSensorsValues.log";
    std::ifstream fs;
    fs.open(filename.c_str(), std::fstream::in);
    std::string temp_string;
  	
    if(fs.is_open())
    {
  		
  	while (std::getline(fs, temp_string))
    	{
        	if(temp_string.size() > 0)
        	{
            		vecOfStr.push_back(temp_string);
            		//std::cout << temp_string << std::endl;
		} 
    	}
   
    }

    else
    {
  	  std::cout << "Could open file" << std::endl;
    }
    rclcpp::spin(std::make_shared<EmulateSensorsNode>());
    rclcpp::shutdown();
    return 0;
}
