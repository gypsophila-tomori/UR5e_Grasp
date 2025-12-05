#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iostream>
#include <sstream>
#include <vector>
#include <thread>
#include <memory>

class ManualUR5eControl : public rclcpp::Node {
public:
    ManualUR5eControl() : Node("manual_ur5e_control") {
        // 注意：MoveGroupInterface 在构造函数中初始化，但需要确保 Node 完全构造
    }
    
    void initialize() {
        // 在 Node 完全构造后初始化 MoveIt
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "ur_manipulator");
        
        // 设置运动参数
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);
        
        RCLCPP_INFO(this->get_logger(), "手动控制节点初始化完成");
        RCLCPP_INFO(this->get_logger(), "规划组: %s", move_group_->getName().c_str());
        RCLCPP_INFO(this->get_logger(), "末端执行器链接: %s", move_group_->getEndEffectorLink().c_str());
    }
    
    void startManualControl() {
        printWelcomeMessage();
        
        std::string input;
        while (rclcpp::ok()) {
            std::cout << "\n请输入命令或坐标: ";
            std::getline(std::cin, input);
            
            if (input.empty()) {
                continue;
            }
            
            if (input == "q" || input == "quit" || input == "exit") {
                RCLCPP_INFO(this->get_logger(), "退出程序");
                break;
            }
            
            if (input == "home" || input == "h") {
                go_to_home_position();
                continue;
            }
            
            if (input == "status" || input == "s") {
                print_current_pose();
                continue;
            }
            
            if (input == "help" || input == "?") {
                printWelcomeMessage();
                continue;
            }
            
            std::vector<double> target_pose;
            if (parsePositionInput(input, target_pose)) {
                bool success = plan_and_execute(target_pose);
                if (success) {
                    RCLCPP_INFO(this->get_logger(), "✅ 运动完成!");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "❌ 运动规划失败!");
                }
            }
        }
    }

private:
    void printWelcomeMessage() {
        std::system("clear");
        std::cout << "==========================================" << std::endl;
        std::cout << "        UR5e 机械臂手动控制程序" << std::endl;
        std::cout << "==========================================" << std::endl;
        std::cout << "命令列表:" << std::endl;
        std::cout << "  home/h     - 返回初始位置" << std::endl;
        std::cout << "  status/s   - 显示当前位姿" << std::endl;
        std::cout << "  help/?     - 显示帮助信息" << std::endl;
        std::cout << "  quit/q     - 退出程序" << std::endl;
        std::cout << std::endl;
        std::cout << "坐标输入格式:" << std::endl;
        std::cout << "  x y z roll pitch yaw" << std::endl;
        std::cout << "  例如: 0.3 0.2 0.5 0 3.14 0" << std::endl;
        std::cout << "  单位: 米(位置), 弧度(角度)" << std::endl;
        std::cout << "==========================================" << std::endl;
        std::cout << std::endl;
    }
    
    bool parsePositionInput(const std::string& input, std::vector<double>& target_pose) {
        std::istringstream iss(input);
        double value;
        target_pose.clear();
        
        while (iss >> value) {
            target_pose.push_back(value);
        }
        
        if (target_pose.size() == 6) {
            RCLCPP_INFO(this->get_logger(), 
                       "目标位姿: x=%.3f, y=%.3f, z=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f",
                       target_pose[0], target_pose[1], target_pose[2],
                       target_pose[3], target_pose[4], target_pose[5]);
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), 
                        "输入格式错误! 需要6个数值 (x,y,z,roll,pitch,yaw), 当前收到 %zu 个",
                        target_pose.size());
            return false;
        }
    }
    
    bool plan_and_execute(const std::vector<double>& target_pose) {
        if (!move_group_) {
            RCLCPP_ERROR(this->get_logger(), "MoveGroup 未初始化");
            return false;
        }
        
        if (target_pose.size() != 6) {
            RCLCPP_ERROR(this->get_logger(), "目标位姿需要6个参数");
            return false;
        }
        
        geometry_msgs::msg::Pose target_pose_msg;
        target_pose_msg.position.x = target_pose[0];
        target_pose_msg.position.y = target_pose[1];
        target_pose_msg.position.z = target_pose[2];
        
        tf2::Quaternion quat;
        quat.setRPY(target_pose[3], target_pose[4], target_pose[5]);
        target_pose_msg.orientation = tf2::toMsg(quat);
        
        move_group_->setPoseTarget(target_pose_msg);
        
        RCLCPP_INFO(this->get_logger(), "开始运动规划...");
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = move_group_->plan(plan);
        
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "规划成功，开始执行...");
            auto execute_result = move_group_->execute(plan);
            
            if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "运动执行完成");
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "运动执行失败");
                return false;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "运动规划失败");
            return false;
        }
    }
    
    void go_to_home_position() {
        RCLCPP_INFO(this->get_logger(), "返回初始位置...");
        std::vector<double> home_pose = {0.3, 0.2, 0.5, 0.0, M_PI, 0.0};
        
        bool success = plan_and_execute(home_pose);
        if (success) {
            RCLCPP_INFO(this->get_logger(), "✅ 已返回初始位置");
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ 返回初始位置失败");
        }
    }
    
    void print_current_pose() {
        if (!move_group_) {
            RCLCPP_ERROR(this->get_logger(), "MoveGroup 未初始化");
            return;
        }
        
        try {
            auto current_pose = move_group_->getCurrentPose();
            
            tf2::Quaternion quat;
            tf2::fromMsg(current_pose.pose.orientation, quat);
            tf2::Matrix3x3 mat(quat);
            double roll, pitch, yaw;
            mat.getRPY(roll, pitch, yaw);
            
            std::cout << "\n=== 当前机械臂位姿 ===" << std::endl;
            std::cout << "位置:" << std::endl;
            std::cout << "  X: " << std::fixed << std::setprecision(4) 
                     << current_pose.pose.position.x << " m" << std::endl;
            std::cout << "  Y: " << current_pose.pose.position.y << " m" << std::endl;
            std::cout << "  Z: " << current_pose.pose.position.z << " m" << std::endl;
            std::cout << "姿态 (欧拉角):" << std::endl;
            std::cout << "  Roll:  " << std::fixed << std::setprecision(4) << roll << " rad" 
                     << " (" << std::setprecision(1) << roll * 180/M_PI << "°)" << std::endl;
            std::cout << "  Pitch: " << pitch << " rad" 
                     << " (" << std::setprecision(1) << pitch * 180/M_PI << "°)" << std::endl;
            std::cout << "  Yaw:   " << yaw << " rad" 
                     << " (" << std::setprecision(1) << yaw * 180/M_PI << "°)" << std::endl;
            std::cout << "==========================" << std::endl;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "获取当前位姿失败: %s", e.what());
        }
    }
    
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ManualUR5eControl>();
    
    // 初始化 MoveIt（在 Node 完全构造后）
    node->initialize();
    
    // 启动执行器在后台运行
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() {
        executor.spin();
    });
    
    // 等待节点初始化完成
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    // 开始手动控制
    node->startManualControl();
    
    // 清理
    rclcpp::shutdown();
    if (spinner_thread.joinable()) {
        spinner_thread.join();
    }
    
    return 0;
}