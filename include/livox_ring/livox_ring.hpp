//add ring data to pcl2 from livox mid-360

#ifndef __LIVOX_RING_LIB__
#define __LIVOX_RING_LIB__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace livox_ring{
	class LivoxConvert : public rclcpp::Node{
		public:
			explicit LivoxConvert(const rclcpp::NodeOptions& options);
			virtual ~LivoxConvert(void);
		private:
			rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
			rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;

			void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_);
	};
}

#endif