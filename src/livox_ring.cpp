//add ring data to pcl2 from livox mid-360

#include "livox_ring/livox_ring.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace livox_ring{
	LivoxConvert::LivoxConvert(const rclcpp::NodeOptions& options) : rclcpp::Node("livox_ring", options){
		using namespace std::placeholders;

		cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("ring_cloud", 10);
		cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("livox/lidar", 10, std::bind(&LivoxConvert::cloudCallback, this, _1));
	}

	LivoxConvert::~LivoxConvert(void){
		;
	}

	void LivoxConvert::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_){
		sensor_msgs::msg::PointCloud2 new_cloud;

		new_cloud.header.frame_id = "livox_frame";
		new_cloud.height = msg_->height;
		new_cloud.width = msg_->width;
		new_cloud.fields.resize(6);
		new_cloud.fields[0].offset = 0;
		new_cloud.fields[0].name = "x";
		new_cloud.fields[0].count = 1;
		new_cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
	
		new_cloud.fields[1].offset = 4;
		new_cloud.fields[1].name = "y";
		new_cloud.fields[1].count = 1;
		new_cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
	
		new_cloud.fields[2].offset = 8;
		new_cloud.fields[2].name = "z";
		new_cloud.fields[2].count = 1;
		new_cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;

		new_cloud.fields[3].offset = 12;
		new_cloud.fields[3].name = "intensity";
		new_cloud.fields[3].count = 1;
		new_cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
	
		new_cloud.fields[4].offset = 16;
		new_cloud.fields[4].name = "ring";
		new_cloud.fields[4].count = 1;
		new_cloud.fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
	
		new_cloud.fields[0].offset = 18;
		new_cloud.fields[0].name = "time";
		new_cloud.fields[0].count = 1;
		new_cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT64;

		new_cloud.data = msg_->data;

		new_cloud.point_step = 24;

		cloud_pub->publish(new_cloud);
	}
}

RCLCPP_COMPONENTS_REGISTER_NODE(livox_ring::LivoxConvert)