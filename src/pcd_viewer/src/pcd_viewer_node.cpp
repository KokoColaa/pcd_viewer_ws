#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <string>
#include <tf2/LinearMath/Quaternion.h>

class PCDViewer : public rclcpp::Node
{
public:
  PCDViewer() : Node("pcd_viewer")
  {
    this->declare_parameter<std::string>("pcd_file", "rmul_2024.pcd");
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<std::string>("child_frame_id", "pcd_frame");
    this->declare_parameter<std::string>("config_file", "transform.yaml");
    this->declare_parameter<std::string>("topic_name", "pcd_points");

    pcd_file_ = this->get_parameter("pcd_file").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    child_frame_id_ = this->get_parameter("child_frame_id").as_string();
    config_file_ = this->get_parameter("config_file").as_string();
    topic_name_ = this->get_parameter("topic_name").as_string();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *cloud) == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s", pcd_file_.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %d data points from %s", cloud->width * cloud->height, pcd_file_.c_str());

    loadTransformFromYAML();

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_, 10);

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = child_frame_id_;

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this, msg]() {
        auto new_msg = msg;
        new_msg.header.stamp = this->now();
        publisher_->publish(new_msg);

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = frame_id_;
        transform.child_frame_id = child_frame_id_;
        transform.transform.translation.x = translation_[0];
        transform.transform.translation.y = translation_[1];
        transform.transform.translation.z = translation_[2];
        transform.transform.rotation.x = rotation_[0];
        transform.transform.rotation.y = rotation_[1];
        transform.transform.rotation.z = rotation_[2];
        transform.transform.rotation.w = rotation_[3];
        tf_broadcaster_->sendTransform(transform);
      });
  }

private:
  void loadTransformFromYAML()
  {
    try
    {
      YAML::Node config = YAML::LoadFile(config_file_);
      YAML::Node transform = config["transform"];
      
      translation_[0] = transform["translation"][0].as<float>();
      translation_[1] = transform["translation"][1].as<float>();
      translation_[2] = transform["translation"][2].as<float>();
      
      if(transform["rotation_euler"])
      {
        float roll = transform["rotation_euler"][0].as<float>();
        float pitch = transform["rotation_euler"][1].as<float>();
        float yaw = transform["rotation_euler"][2].as<float>();
        
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);  
        rotation_[0] = q.x();
        rotation_[1] = q.y();
        rotation_[2] = q.z();
        rotation_[3] = q.w();
      }
      else if(transform["rotation"]) 
      {
        rotation_[0] = transform["rotation"][0].as<float>();
        rotation_[1] = transform["rotation"][1].as<float>();
        rotation_[2] = transform["rotation"][2].as<float>();
        rotation_[3] = transform["rotation"][3].as<float>();
      }
      
      RCLCPP_INFO(this->get_logger(), "Loaded transform from %s", config_file_.c_str());
    }
    catch (const YAML::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error loading YAML file: %s. Using default transform.", e.what());
      translation_ = {0.0, 0.0, 0.0};
      rotation_ = {0.0, 0.0, 0.0, 1.0};  // 无旋转
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{
    std::make_unique<tf2_ros::TransformBroadcaster>(*this)};
  
  std::string pcd_file_;
  std::string frame_id_;
  std::string child_frame_id_;
  std::string config_file_;
  std::string topic_name_;
  
  std::array<float, 3> translation_;
  std::array<float, 4> rotation_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCDViewer>());
  rclcpp::shutdown();
  return 0;
}