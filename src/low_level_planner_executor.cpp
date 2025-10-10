#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose.hpp>
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
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "ur5_manipulator");
        move_group_->setPoseReferenceFrame("base_link");
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

    // === Service callbacks ===
    void get_current_pose_cb(
        const std::shared_ptr<GetCurrentPose::Request>,
        std::shared_ptr<GetCurrentPose::Response> res)
    {
        geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose("tool0");
        res->pose = current_pose.pose;
        res->success = true;
        res->message = "Current pose retrieved successfully.";
        RCLCPP_INFO(this->get_logger(), "End-effector pose retrieved.");
    }

    void get_joint_angles_cb(
        const std::shared_ptr<GetJointAngles::Request>,
        std::shared_ptr<GetJointAngles::Response> res)
    {
        std::vector<double> joints = move_group_->getCurrentJointValues();
        res->joint_positions = joints;
        res->success = true;
        res->message = "Joint angles retrieved successfully.";
        RCLCPP_INFO(this->get_logger(), "Joint angles retrieved.");
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
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"use_sim_time", rclcpp::ParameterValue(true)}});
    auto node = std::make_shared<RobotInterfaceNode>(options);
    node->init_move_group();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
