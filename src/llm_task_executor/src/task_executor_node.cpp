#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ur5e_gripper_control/ur5e_gripper.h" // 引用UR5eGripper类
#include "nlohmann/json.hpp"

#include <map>
#include <string>
#include <vector>

using json = nlohmann::json;

// 创建一个新的节点类来封装我们的逻辑
class TaskExecutorNode : public rclcpp::Node {
public:
    // 构造函数，接收一个UR5eGripper的共享指针
    explicit TaskExecutorNode(std::shared_ptr<UR5eGripper> ur5e_gripper)
        : Node("task_executor_node"), ur5e_gripper_(ur5e_gripper) {
        
        RCLCPP_INFO(this->get_logger(), "Task Executor Node has been started.");

        // 创建订阅者
        llm_plan_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/llm_plan", 10, std::bind(&TaskExecutorNode::llm_plan_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to /llm_plan topic.");

        // 从参数服务器加载配置
        this->declare_parameter("place_locations.red_bin", std::vector<double>{});
        this->declare_parameter("place_locations.green_bin", std::vector<double>{});
        this->declare_parameter("place_locations.blue_bin", std::vector<double>{});
        
        place_locations_["red_bin"] = this->get_parameter("place_locations.red_bin").as_double_array();
        place_locations_["green_bin"] = this->get_parameter("place_locations.green_bin").as_double_array();
        place_locations_["blue_bin"] = this->get_parameter("place_locations.blue_bin").as_double_array();

        this->declare_parameter("grasp_pose_correction.orientation_override", std::vector<double>{0.0, 3.14, 0.0});
        this->declare_parameter("grasp_pose_correction.z_offset", 0.14);
        this->declare_parameter("grasp_pose_correction.x_offset", -0.012);
        this->declare_parameter("grasp_pose_correction.y_offset", 0.01);

        grasp_orientation_override_ = this->get_parameter("grasp_pose_correction.orientation_override").as_double_array();
        grasp_z_offset_ = this->get_parameter("grasp_pose_correction.z_offset").as_double();
        grasp_x_offset_ = this->get_parameter("grasp_pose_correction.x_offset").as_double();
        grasp_y_offset_ = this->get_parameter("grasp_pose_correction.y_offset").as_double();

        RCLCPP_INFO(this->get_logger(), "Loaded place locations and grasp corrections from parameters.");
    }

private:
    // 核心的回调函数
    void llm_plan_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received a new plan: %s", msg->data.c_str());

        try {
            auto plan = json::parse(msg->data);

            for (const auto& step : plan["actions"]) {
                if (step.contains("pick")) {
                    std::string object_name = step["pick"];
                    RCLCPP_INFO(this->get_logger(), "Executing PICK on object: %s", object_name.c_str());

                    std::vector<double> object_pose;
                    ur5e_gripper_->get_cube_pose("base_link", object_name, object_pose);

                    if (object_pose.empty()) {
                        RCLCPP_ERROR(this->get_logger(), "Could not find TF for '%s'. Aborting this step.", object_name.c_str());
                        continue;
                    }

                    // 应用配置文件中的姿态修正
                    object_pose[0] += grasp_x_offset_;
                    object_pose[1] += grasp_y_offset_;
                    object_pose[2] += grasp_z_offset_;
                    object_pose[3] = grasp_orientation_override_[0]; // Roll
                    object_pose[4] = grasp_orientation_override_[1]; // Pitch
                    object_pose[5] = grasp_orientation_override_[2]; // Yaw

                    RCLCPP_INFO(this->get_logger(), "Planning to grasp pose: [%.2f, %.2f, %.2f]", object_pose[0], object_pose[1], object_pose[2]);
                    if (ur5e_gripper_->plan_and_execute(object_pose)) {
                        ur5e_gripper_->grasp(0.36); // 假设0.36是闭合
                        rclcpp::sleep_for(std::chrono::seconds(2));
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Pick failed for %s.", object_name.c_str());
                        return; // 终止整个计划
                    }

                } else if (step.contains("place")) {
                    std::string place_name = step["place"];
                    RCLCPP_INFO(this->get_logger(), "Executing PLACE at location: %s", place_name.c_str());

                    if (place_locations_.count(place_name)) {
                        std::vector<double> place_pose = place_locations_[place_name];
                        RCLCPP_INFO(this->get_logger(), "Planning to place pose: [%.2f, %.2f, %.2f]", place_pose[0], place_pose[1], place_pose[2]);
                        if (ur5e_gripper_->plan_and_execute(place_pose)) {
                            ur5e_gripper_->grasp(0.0); // 假设0.0是张开
                            rclcpp::sleep_for(std::chrono::seconds(2));
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "Place failed at %s.", place_name.c_str());
                            return; // 终止整个计划
                        }
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Unknown place location: '%s'. Aborting.", place_name.c_str());
                        return;
                    }
                }
            }

            RCLCPP_INFO(this->get_logger(), "Plan executed successfully!");
            ur5e_gripper_->go_to_ready_position();

        } catch (json::parse_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON plan: %s", e.what());
        }
    }

    // 成员变量
    std::shared_ptr<UR5eGripper> ur5e_gripper_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr llm_plan_subscription_;
    std::map<std::string, std::vector<double>> place_locations_;
    
    // 从参数加载的抓取修正值
    std::vector<double> grasp_orientation_override_;
    double grasp_z_offset_;
    double grasp_x_offset_;
    double grasp_y_offset_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // 为两个节点都使用参数
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // 1. 创建UR5eGripper节点实例
    auto ur5e_gripper = std::make_shared<UR5eGripper>(node_options);
    ur5e_gripper->init(); // Don't forget to init!

    // 2. 创建TaskExecutorNode节点实例，并将ur5e_gripper传递给它
    auto task_executor = std::make_shared<TaskExecutorNode>(ur5e_gripper);

    // 3. 使用多线程执行器来同时运行两个节点
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(ur5e_gripper);
    executor.add_node(task_executor);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting executor...");
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
