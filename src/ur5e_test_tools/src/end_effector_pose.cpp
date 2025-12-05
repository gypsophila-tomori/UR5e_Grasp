#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // 修复的头文件
#include <iostream>
#include <iomanip>

class EndEffectorPoseMonitor : public rclcpp::Node {
public:
    EndEffectorPoseMonitor() : Node("end_effector_pose_monitor") {
        // 初始化 TF
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }
    
    void startPoseMonitoring() {
        RCLCPP_INFO(this->get_logger(), "开始实时监测末端执行器位姿");
        RCLCPP_INFO(this->get_logger(), "按 Ctrl+C 停止监测");
        
        rclcpp::Rate rate(10); // 10Hz 更新频率
        
        while (rclcpp::ok()) {
            try {
                geometry_msgs::msg::TransformStamped transform;
                transform = tf_buffer_->lookupTransform(
                    "base_link", "tool0", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
                
                auto& translation = transform.transform.translation;
                auto& rotation = transform.transform.rotation;
                
                tf2::Quaternion quat;
                tf2::fromMsg(rotation, quat);
                tf2::Matrix3x3 mat(quat);
                double roll, pitch, yaw;
                mat.getRPY(roll, pitch, yaw);
                
                printPoseInfo(translation.x, translation.y, translation.z, roll, pitch, yaw, quat);
                
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "TF 变换错误: %s", ex.what());
            }
            
            rate.sleep();
        }
    }
    
private:
    void printPoseInfo(double x, double y, double z, double roll, double pitch, double yaw,
                      [[maybe_unused]] const tf2::Quaternion& quat) {  // 修复的未使用参数
        std::system("clear");
        
        std::cout << "=== UR5e 末端执行器实时位姿 ===" << std::endl;
        std::cout << "时间: " << get_clock()->now().seconds() << "s" << std::endl;
        std::cout << std::endl;
        
        std::cout << "位置 (相对于 base_link):" << std::endl;
        std::cout << "  X: " << std::fixed << std::setprecision(4) << x << " m" << std::endl;
        std::cout << "  Y: " << std::fixed << std::setprecision(4) << y << " m" << std::endl;
        std::cout << "  Z: " << std::fixed << std::setprecision(4) << z << " m" << std::endl;
        std::cout << std::endl;
        
        std::cout << "欧拉角姿态:" << std::endl;
        std::cout << "  Roll:  " << std::fixed << std::setprecision(4) << roll << " rad" 
                  << " (" << std::setprecision(1) << roll * 180/M_PI << "°)" << std::endl;
        std::cout << "  Pitch: " << std::fixed << std::setprecision(4) << pitch << " rad" 
                  << " (" << std::setprecision(1) << pitch * 180/M_PI << "°)" << std::endl;
        std::cout << "  Yaw:   " << std::fixed << std::setprecision(4) << yaw << " rad" 
                  << " (" << std::setprecision(1) << yaw * 180/M_PI << "°)" << std::endl;
        std::cout << std::endl;
        
        std::cout << "按 Ctrl+C 停止监测" << std::endl;
    }
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EndEffectorPoseMonitor>();
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() {
        executor.spin();
    });
    
    rclcpp::sleep_for(std::chrono::seconds(2));
    node->startPoseMonitoring();
    
    rclcpp::shutdown();
    if (spinner_thread.joinable()) {
        spinner_thread.join();
    }
    
    return 0;
}