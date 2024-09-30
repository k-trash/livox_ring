//add ring data to pcl2 from livox mid-360

#include "livox_ring/livox_ring.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <iostream>
#include <vector>

namespace livox_ring{
	LivoxConvert::LivoxConvert(const rclcpp::NodeOptions& options) : rclcpp::Node("livox_ring", options){
		using namespace std::placeholders;

		cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensing/lidar/concatenated/pointcloud", 10);
		cloud_pub2 = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensing/lidar/top/pointcloud_raw", 10);
	//	cloud_pub3 = this->create_publisher<sensor_msgs::msg::PointCloud2>("perception/obstacle_segmentation/range_cropped/pointcloud", 10);
		cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("livox/lidar", 10, std::bind(&LivoxConvert::cloudCallback, this, _1));
	}

	LivoxConvert::~LivoxConvert(void){
			;
	}

	void LivoxConvert::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_){
		sensor_msgs::msg::PointCloud2 new_cloud;
		std::vector<uint8_t> xyzi(16);
		std::vector<uint8_t> aed(12);
		float izyx[4];
		float dea[3];
		uint32_t zero = 0u;

		new_cloud.header.stamp = msg_->header.stamp;
		new_cloud.header.frame_id = "livox_frame";
		new_cloud.height = msg_->height;
		new_cloud.width = msg_->width;
		new_cloud.is_bigendian = msg_->is_bigendian;
		new_cloud.row_step = msg_->height*msg_->width*32;
		new_cloud.is_dense = msg_->is_dense;

/*
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
	
		new_cloud.fields[5].offset = 18;
		new_cloud.fields[5].name = "time";
		new_cloud.fields[5].count = 1;
		new_cloud.fields[5].datatype = sensor_msgs::msg::PointField::FLOAT64;

		new_cloud.data = msg_->data;

		new_cloud.point_step = 26;
*/
		new_cloud.fields.resize(10);
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
		new_cloud.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
	
		new_cloud.fields[4].offset = 13;
		new_cloud.fields[4].name = "return type";
		new_cloud.fields[4].count = 1;
		new_cloud.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
	
		new_cloud.fields[5].offset = 14;
		new_cloud.fields[5].name = "channel";
		new_cloud.fields[5].count = 1;
		new_cloud.fields[5].datatype = sensor_msgs::msg::PointField::UINT16;

		new_cloud.fields[6].offset = 16;
		new_cloud.fields[6].name = "azimuth";
		new_cloud.fields[6].count = 1;
		new_cloud.fields[6].datatype = sensor_msgs::msg::PointField::FLOAT32;

		new_cloud.fields[7].offset = 20;
		new_cloud.fields[7].name = "elevation";
		new_cloud.fields[7].count = 1;
		new_cloud.fields[7].datatype = sensor_msgs::msg::PointField::FLOAT32;

		new_cloud.fields[8].offset = 24;
		new_cloud.fields[8].name = "distance";
		new_cloud.fields[8].count = 1;
		new_cloud.fields[8].datatype = sensor_msgs::msg::PointField::FLOAT32;

		new_cloud.fields[9].offset = 28;
		new_cloud.fields[9].name = "time";
		new_cloud.fields[9].count = 1;
		new_cloud.fields[9].datatype = sensor_msgs::msg::PointField::UINT32;

		new_cloud.data.resize(msg_->height*msg_->width*32);

		for(uint32_t i=0;i<msg_->height*msg_->width;i++){
			std::memcpy(&new_cloud.data[32*i], &(msg_->data[26*i]), 12);
			std::memcpy(&xyzi[0], &(msg_->data[26*i]), 16);
			std::reverse(xyzi.begin(), xyzi.end());
			std::memcpy(izyx, &xyzi[0], 16);
			new_cloud.data[32*i+12] = 80;//static_cast<uint8_t>(izyx[0]);
			new_cloud.data[32*i+13] = msg_->data[26*i+16];
			new_cloud.data[32*i+14] = msg_->data[26*i+17];
			dea[0] = std::hypot(izyx[1], izyx[2], izyx[3]);
			dea[1] = std::atan2(izyx[1], dea[0]);
			dea[2] = std::atan2(izyx[2], izyx[3]);
			std::memcpy(&aed[0], dea, 12);
			std::reverse(aed.begin(), aed.end());
			std::memcpy(&new_cloud.data[32*i+16], &aed[0], 12);
			std::memcpy(&new_cloud.data[32*i+28], &zero, 4);
		}
			
		new_cloud.point_step = 32;


		cloud_pub->publish(new_cloud);
		cloud_pub2->publish(new_cloud);
	//	cloud_pub3->publish(new_cloud);
	}
}

RCLCPP_COMPONENTS_REGISTER_NODE(livox_ring::LivoxConvert)
