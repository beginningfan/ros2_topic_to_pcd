#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node
{
  public:
    Publisher():Node("pcd_publisher")
    {
      this->declare_parameter<std::string>("pcd_filename", "123.pcd");
      this->get_parameter("pcd_filename", pcd_filename_);

      this->declare_parameter<std::string>("topic_name", "/sensing/lidar/top/rectified/pointcloud");
      this->get_parameter("topic_name", pcd_filename_);

      if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_filename_, cloud)== -1)
      {
        std::cout << "NO such file" << std::endl;
        rclcpp::shutdown();
      }
      pcl::toROSMsg(cloud, output);
      rclcpp::QoS qos{10};
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("topic", qos);
      timer_ = this->create_wall_timer(
      100ms, std::bind(&Publisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      output.header.frame_id = "base_link";
      output.header.stamp = rclcpp::Clock().now();
      RCLCPP_INFO(this->get_logger(), "Publishing PCD");
      publisher_->publish(output);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    std::string pcd_filename_;
    std::string topic_name_;
    sensor_msgs::msg::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZ> cloud;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}
