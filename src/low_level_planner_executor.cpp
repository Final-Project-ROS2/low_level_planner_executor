#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include "custom_interfaces/srv/get_current_pose.hpp"
#include "custom_interfaces/srv/get_joint_angles.hpp"
#include "custom_interfaces/action/set_joint_angles.hpp"

using namespace std::placeholders;

class RobotInterfaceNode : public rclcpp::Node
{
public:
    using GetCurrentPose = custom_interfaces::srv::GetCurrentPose;
    using GetJointAngles = custom_interfaces::srv::GetJointAngles;
    using SetJointAngles = custom_interfaces::action::SetJointAngles;
    using GoalHandleSetJointAngles = rclcpp_action::ServerGoalHandle<SetJointAngles>;

    explicit RobotInterfaceNode(const rclcpp::NodeOptions &options) 
    : Node("robot_interface_node", options)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Robot Interface Node...");

        RCLCPP_INFO(this->get_logger(), "Initializing MoveIt relative motion action server...");

        this->declare_parameter<bool>("real_hardware", false);
        real_hardware = this->get_parameter("real_hardware").as_bool();
        RCLCPP_INFO(this->get_logger(), "Running with real_hardware: %s", real_hardware ? "true" : "false");

        bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
        RCLCPP_INFO(this->get_logger(), "Use sim time: %s", use_sim_time ? "true" : "false");

        // === Subscriber for /joint_states ===
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&RobotInterfaceNode::joint_state_cb, this, _1));

        // Services
        get_pose_srv_ = this->create_service<GetCurrentPose>(
            "get_current_pose",
            std::bind(&RobotInterfaceNode::get_current_pose_cb, this, _1, _2));

        get_joint_srv_ = this->create_service<GetJointAngles>(
            "get_joint_angles",
            std::bind(&RobotInterfaceNode::get_joint_angles_cb, this, _1, _2));

        // Action server
        action_server_ = rclcpp_action::create_server<SetJointAngles>(
            this,
            "set_joint_angles",
            std::bind(&RobotInterfaceNode::handle_goal, this, _1, _2),
            std::bind(&RobotInterfaceNode::handle_cancel, this, _1),
            std::bind(&RobotInterfaceNode::handle_accepted, this, _1));
    }

    void init_move_group()
    {
        RCLCPP_INFO(this->get_logger(), "Using real_hardware: %s", real_hardware ? "true" : "false");
        if (real_hardware) {
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
            BASE_LINK_X_OFFSET = 0.0;
            BASE_LINK_Y_OFFSET = 0.0;
            BASE_LINK_Z_OFFSET = 0.0;
        } else {
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur5_manipulator");
            BASE_LINK_X_OFFSET = 0.0;
            BASE_LINK_Y_OFFSET = 0.0;
            BASE_LINK_Z_OFFSET = 0.8;
        }
        move_group_->setPoseReferenceFrame("base_link");
        move_group_->setPlanningTime(10.0);

        RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
        RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized.");
    }

private:
    // === MoveIt interfaces ===
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // === ROS entities ===
    rclcpp::Service<GetCurrentPose>::SharedPtr get_pose_srv_;
    rclcpp::Service<GetJointAngles>::SharedPtr get_joint_srv_;
    rclcpp_action::Server<SetJointAngles>::SharedPtr action_server_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    bool real_hardware;
    double BASE_LINK_X_OFFSET;
    double BASE_LINK_Y_OFFSET;
    double BASE_LINK_Z_OFFSET;
    std::vector<double> latest_joint_positions_;
    std::mutex joint_mutex_;

    // === Joint state subscriber callback ===
    void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        latest_joint_positions_ = msg->position;
    }

    // === Service callbacks ===
    void get_current_pose_cb(
        const std::shared_ptr<GetCurrentPose::Request>,
        std::shared_ptr<GetCurrentPose::Response> res)
    {
        RCLCPP_INFO(this->get_logger(), "Clock type: %d", this->get_clock()->get_clock_type());
        geometry_msgs::msg::Pose current_pose = move_group_->getCurrentPose("tool0").pose;
        auto &p = current_pose.position;
        auto &o = current_pose.orientation;

        // Check if pose is all zeros (with w=1.0)
        bool is_default_pose = (p.x == 0.0 && p.y == 0.0 && p.z == 0.0 &&
                                o.x == 0.0 && o.y == 0.0 && o.z == 0.0 && o.w == 1.0);

        if (!is_default_pose) {
            res->pose = current_pose;
            res->success = true;
            res->message = "Current pose retrieved successfully.";
            RCLCPP_INFO(this->get_logger(), "End-effector pose retrieved.");
        } else {
            res->success = false;
            res->message = "Failed to fetch current robot state (default pose returned).";
            RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
        }
    }


    void get_joint_angles_cb(
        const std::shared_ptr<GetJointAngles::Request>,
        std::shared_ptr<GetJointAngles::Response> res)
    {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        if (!latest_joint_positions_.empty()) {
            res->joint_positions = latest_joint_positions_;
            res->success = true;
            res->message = "Joint angles retrieved successfully.";
            RCLCPP_INFO(this->get_logger(), "Joint angles retrieved from /joint_states.");
        } else {
            res->success = false;
            res->message = "No joint states received yet.";
            RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
        }
    }


    // === Action callbacks ===
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const SetJointAngles::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal to set joint angles (%zu values).", goal->joint_positions.size());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleSetJointAngles>)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleSetJointAngles> goal_handle)
    {
        std::thread([this, goal_handle]() {
            execute(goal_handle);
        }).detach();
    }

    void execute(const std::shared_ptr<GoalHandleSetJointAngles> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<SetJointAngles::Feedback>();
        auto result = std::make_shared<SetJointAngles::Result>();

        RCLCPP_INFO(this->get_logger(), "Planning motion to target joint positions...");

        move_group_->setJointValueTarget(goal->joint_positions);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Executing planned motion...");
            feedback->progress = 0.5;
            goal_handle->publish_feedback(feedback);

            move_group_->execute(plan);
            feedback->progress = 1.0;
            goal_handle->publish_feedback(feedback);

            result->success = true;
            result->message = "Motion execution completed successfully.";
            goal_handle->succeed(result);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
            result->success = false;
            result->message = "Motion planning failed.";
            goal_handle->abort(result);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Declare default for the 'real_hardware' argument
    bool real_hardware = false;

    // Parse command-line arguments (like ros2 run pkg node --ros-args -p real_hardware:=true)
    rclcpp::NodeOptions temp_options;
    auto temp_node = std::make_shared<rclcpp::Node>("temp_node_for_args", temp_options);
    temp_node->declare_parameter("real_hardware", rclcpp::ParameterValue(false));
    real_hardware = temp_node->get_parameter("real_hardware").as_bool();

    // Determine use_sim_time based on 'real_hardware'
    bool use_sim_time = !real_hardware;

    rclcpp::NodeOptions options;
    options.parameter_overrides({{"use_sim_time", rclcpp::ParameterValue(use_sim_time)},
                                {"real_hardware", rclcpp::ParameterValue(real_hardware)}});
    auto node = std::make_shared<RobotInterfaceNode>(options);
    node->init_move_group();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
