#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// #define LINE printf("%d\n", __LINE__)

class Subscriber : public rclcpp::Node
{
  public:
    Subscriber():Node("subscriber")
    {
      this->declare_parameter<std::string>("pcd_filepath", "/home/file/path/");
      this->get_parameter("pcd_filepath", pcd_filepath_);
      this->declare_parameter<std::string>("topic_name", "/sensing/lidar/top/rectified/pointcloud");
      this->get_parameter("topic_name", topic_name_);


      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_name_, 
      rclcpp::SensorDataQoS().keep_last(1), 
      std::bind(&Subscriber::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2 & msg)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(msg, *pcl_pointcloud);
      auto stamp = msg.header.stamp;
      std::string filename = pcd_filepath_ + std::to_string(stamp.sec + stamp.nanosec/1000000000.0) + ".pcd";
      RCLCPP_INFO(this->get_logger(), "Writing pcd file");
      pcl::io::savePCDFileASCII(filename, *pcl_pointcloud);
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

    std::string topic_name_;
    std::string pcd_filepath_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}