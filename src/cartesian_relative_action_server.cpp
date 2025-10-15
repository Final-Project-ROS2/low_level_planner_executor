#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "custom_interfaces/action/moveit_relative.hpp"

using MoveItRelative = custom_interfaces::action::MoveitRelative;

class MoveItRelativeActionServer : public rclcpp::Node
{
public:
    using GoalHandleMoveItRelative = rclcpp_action::ServerGoalHandle<MoveItRelative>;

    explicit MoveItRelativeActionServer(const rclcpp::NodeOptions &options)
        : Node("moveit_relative_action_server", options)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing MoveIt relative motion action server...");

        this->declare_parameter<bool>("real_hardware", false);
        real_hardware = this->get_parameter("real_hardware").as_bool();
        RCLCPP_INFO(this->get_logger(), "Running with real_hardware: %s", real_hardware ? "true" : "false");

        bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
        RCLCPP_INFO(this->get_logger(), "Use sim time: %s", use_sim_time ? "true" : "false");

        // Create the action server
        action_server_ = rclcpp_action::create_server<MoveItRelative>(
            this,
            "plan_cartesian_relative",
            std::bind(&MoveItRelativeActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveItRelativeActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&MoveItRelativeActionServer::handle_accepted, this, std::placeholders::_1)
        );
    }

    void init_move_group()
    {
        RCLCPP_INFO(this->get_logger(), "Using real_hardware: %s", real_hardware ? "true" : "false");
        if (real_hardware) {
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
        } else {
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur5_manipulator");
        }
        move_group_->setPoseReferenceFrame("base_link");
        move_group_->setPlanningTime(10.0);

        RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
    }

private:
    rclcpp_action::Server<MoveItRelative>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    bool real_hardware;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const MoveItRelative::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(),
            "Received relative motion goal: Δx=%.3f, Δy=%.3f, Δz=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f",
            goal->distance_x, goal->distance_y, goal->distance_z,
            goal->roll, goal->pitch, goal->yaw);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveItRelative> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveItRelative> goal_handle)
    {
        std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
    }

    void execute(const std::shared_ptr<GoalHandleMoveItRelative> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveItRelative::Feedback>();
        auto result = std::make_shared<MoveItRelative::Result>();

        RCLCPP_INFO(this->get_logger(), "Executing relative motion...");

        // Get current pose
        geometry_msgs::msg::Pose current_pose = move_group_->getCurrentPose().pose;
        geometry_msgs::msg::Pose target_pose = current_pose;

        RCLCPP_INFO(this->get_logger(), "Current pose: %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
                current_pose.position.x, current_pose.position.y, current_pose.position.z,
                current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);

        // Apply relative translation
        target_pose.position.x += goal->distance_x;
        target_pose.position.y += goal->distance_y;
        target_pose.position.z += goal->distance_z;

        // Apply relative rotation
        tf2::Quaternion q_orig, q_rot, q_new;
        tf2::fromMsg(current_pose.orientation, q_orig);
        q_rot.setRPY(goal->roll, goal->pitch, goal->yaw);
        q_new = q_rot * q_orig;
        q_new.normalize();
        target_pose.orientation = tf2::toMsg(q_new);

        RCLCPP_INFO(this->get_logger(), "Target pose: %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
                target_pose.position.x, target_pose.position.y, target_pose.position.z,
                target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);

        // Plan Cartesian path
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.01;
        const double jump_threshold = 0.0;
        if (!real_hardware) {
            

            // Collision object

            std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
            collision_objects.resize(2);

            collision_objects[0].id = "table1";
            collision_objects[0].header.frame_id = "world";
            collision_objects[0].primitives.resize(1);
            collision_objects[0].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
            collision_objects[0].primitives[0].dimensions = {0.608, 2.0, 0.8};
            collision_objects[0].primitive_poses.resize(1);
            collision_objects[0].primitive_poses[0].position.x = 0.576;
            collision_objects[0].primitive_poses[0].position.y = 0.0;
            collision_objects[0].primitive_poses[0].position.z = 0.4;
            collision_objects[0].operation = moveit_msgs::msg::CollisionObject::ADD;

            collision_objects[1].id = "base";
            collision_objects[1].header.frame_id = "world";
            collision_objects[1].primitives.resize(1);
            collision_objects[1].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
            collision_objects[1].primitives[0].dimensions = {1, 1, 0.79}; 
            collision_objects[1].primitive_poses.resize(1);
            collision_objects[1].primitive_poses[0].position.x = -0.3;
            collision_objects[1].primitive_poses[0].position.y = 0.0;
            collision_objects[1].primitive_poses[0].position.z = 0.4;
            collision_objects[1].operation = moveit_msgs::msg::CollisionObject::ADD;


            // Add objects to the scene
            planning_scene_interface.applyCollisionObjects(collision_objects);
        }
        

        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        feedback->progress = fraction;
        goal_handle->publish_feedback(feedback);

        if (fraction > 0.0)
        {
            move_group_->execute(trajectory);
            RCLCPP_INFO(this->get_logger(), "Motion executed successfully (%.2f%% of path planned)", fraction * 100.0);
            result->success = true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Cartesian path planning failed!");
            result->success = false;
        }

        goal_handle->succeed(result);
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
    auto node = std::make_shared<MoveItRelativeActionServer>(options);
    node->init_move_group();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
