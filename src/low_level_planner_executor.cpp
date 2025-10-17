#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include "custom_interfaces/action/get_current_pose.hpp"
#include "custom_interfaces/action/get_joint_angles.hpp"
#include "custom_interfaces/action/set_joint_angles.hpp"

using namespace std::placeholders;

class RobotInterfaceNode : public rclcpp::Node
{
public:
    using GetCurrentPose = custom_interfaces::action::GetCurrentPose;
    using GetJointAngles = custom_interfaces::action::GetJointAngles;
    using SetJointAngles = custom_interfaces::action::SetJointAngles;

    using GoalHandleGetCurrentPose = rclcpp_action::ServerGoalHandle<GetCurrentPose>;
    using GoalHandleGetJointAngles = rclcpp_action::ServerGoalHandle<GetJointAngles>;
    using GoalHandleSetJointAngles = rclcpp_action::ServerGoalHandle<SetJointAngles>;

    explicit RobotInterfaceNode(const rclcpp::NodeOptions &options)
        : Node("robot_interface_node", options)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Robot Interface Node...");

        this->declare_parameter<bool>("real_hardware", false);
        real_hardware = this->get_parameter("real_hardware").as_bool();
        RCLCPP_INFO(this->get_logger(), "Running with real_hardware: %s", real_hardware ? "true" : "false");

        bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
        RCLCPP_INFO(this->get_logger(), "Use sim time: %s", use_sim_time ? "true" : "false");

        // === Action servers ===
        get_pose_action_server_ = rclcpp_action::create_server<GetCurrentPose>(
            this,
            "get_current_pose",
            std::bind(&RobotInterfaceNode::handle_goal_pose, this, _1, _2),
            std::bind(&RobotInterfaceNode::handle_cancel_pose, this, _1),
            std::bind(&RobotInterfaceNode::handle_accepted_pose, this, _1));

        get_joint_action_server_ = rclcpp_action::create_server<GetJointAngles>(
            this,
            "get_joint_angles",
            std::bind(&RobotInterfaceNode::handle_goal_joint, this, _1, _2),
            std::bind(&RobotInterfaceNode::handle_cancel_joint, this, _1),
            std::bind(&RobotInterfaceNode::handle_accepted_joint, this, _1));

        set_joint_action_server_ = rclcpp_action::create_server<SetJointAngles>(
            this,
            "set_joint_angles",
            std::bind(&RobotInterfaceNode::handle_goal_set_joint, this, _1, _2),
            std::bind(&RobotInterfaceNode::handle_cancel_set_joint, this, _1),
            std::bind(&RobotInterfaceNode::handle_accepted_set_joint, this, _1));
    }

    void init_move_group()
    {
        if (real_hardware)
        {
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
            BASE_LINK_Z_OFFSET = 0.0;
        }
        else
        {
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur5_manipulator");
            BASE_LINK_Z_OFFSET = 0.8;
        }
        move_group_->setPoseReferenceFrame("base_link");
        move_group_->setPlanningTime(10.0);

        RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized.");
    }

private:
    // === MoveIt interfaces ===
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // === Action servers ===
    rclcpp_action::Server<GetCurrentPose>::SharedPtr get_pose_action_server_;
    rclcpp_action::Server<GetJointAngles>::SharedPtr get_joint_action_server_;
    rclcpp_action::Server<SetJointAngles>::SharedPtr set_joint_action_server_;

    bool real_hardware;
    double BASE_LINK_Z_OFFSET;

    // === GetCurrentPose action callbacks ===
    rclcpp_action::GoalResponse handle_goal_pose(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const GetCurrentPose::Goal>)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal for get_current_pose");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_pose(
        const std::shared_ptr<GoalHandleGetCurrentPose>)
    {
        RCLCPP_INFO(this->get_logger(), "Cancel request for get_current_pose");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_pose(
        const std::shared_ptr<GoalHandleGetCurrentPose> goal_handle)
    {
        std::thread([this, goal_handle]() {
            auto result = std::make_shared<GetCurrentPose::Result>();
            geometry_msgs::msg::Pose pose = move_group_->getCurrentPose().pose;
            auto &p = pose.position;
            auto &o = pose.orientation;

            bool is_default_pose = (p.x == 0.0 && p.y == 0.0 && p.z == 0.0 &&
                                    o.x == 0.0 && o.y == 0.0 && o.z == 0.0 && o.w == 1.0);

            if (!is_default_pose)
            {
                result->pose = pose;
                result->success = true;
                result->message = "Current pose retrieved successfully.";
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "End-effector pose retrieved.");
            }
            else
            {
                result->success = false;
                result->message = "Failed to fetch current robot state.";
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
            }
        }).detach();
    }

    // === GetJointAngles action callbacks ===
    rclcpp_action::GoalResponse handle_goal_joint(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const GetJointAngles::Goal>)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal for get_joint_angles");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_joint(
        const std::shared_ptr<GoalHandleGetJointAngles>)
    {
        RCLCPP_INFO(this->get_logger(), "Cancel request for get_joint_angles");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_joint(
        const std::shared_ptr<GoalHandleGetJointAngles> goal_handle)
    {
        std::thread([this, goal_handle]() {
            auto result = std::make_shared<GetJointAngles::Result>();
            try
            {
                std::vector<double> joints = move_group_->getCurrentJointValues();
                result->joint_positions = joints;
                result->success = true;
                result->message = "Joint angles retrieved successfully.";
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Joint angles retrieved.");
            }
            catch (const std::runtime_error &e)
            {
                result->success = false;
                result->message = std::string("Failed to get joint angles: ") + e.what();
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
            }
        }).detach();
    }

    // === SetJointAngles action callbacks ===
    rclcpp_action::GoalResponse handle_goal_set_joint(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const SetJointAngles::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal to set joint angles (%zu values).", goal->joint_positions.size());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_set_joint(
        const std::shared_ptr<GoalHandleSetJointAngles>)
    {
        RCLCPP_INFO(this->get_logger(), "Cancel request for set_joint_angles");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_set_joint(
        const std::shared_ptr<GoalHandleSetJointAngles> goal_handle)
    {
        std::thread([this, goal_handle]() {
            execute_set_joint(goal_handle);
        }).detach();
    }

    void execute_set_joint(const std::shared_ptr<GoalHandleSetJointAngles> goal_handle)
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

    bool real_hardware = false;
    rclcpp::NodeOptions temp_options;
    auto temp_node = std::make_shared<rclcpp::Node>("temp_node_for_args", temp_options);
    temp_node->declare_parameter("real_hardware", rclcpp::ParameterValue(false));
    real_hardware = temp_node->get_parameter("real_hardware").as_bool();

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
