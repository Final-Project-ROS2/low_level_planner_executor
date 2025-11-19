#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <atomic>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "ur_msgs/srv/set_speed_slider_fraction.hpp"

#include <rclcpp_action/rclcpp_action.hpp>
#include "custom_interfaces/action/moveit_pose.hpp"

using MoveItPose = custom_interfaces::action::MoveitPose; // Custom action

class MoveItPoseActionServer : public rclcpp::Node
{
public:
    using GoalHandleMoveItPose = rclcpp_action::ServerGoalHandle<MoveItPose>;

    explicit MoveItPoseActionServer(const rclcpp::NodeOptions &options)
    : Node("cartesian_path_action_server", options),
      emergency_stop_(false),
      normal_speed_(0.3),
      emergency_speed_(0.0)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing MoveIt pose action server...");

        this->declare_parameter<double>("normal_speed", 0.3);
        this->declare_parameter<double>("emergency_speed", 0.0);

        this->declare_parameter<bool>("real_hardware", false);
        real_hardware = this->get_parameter("real_hardware").as_bool();
        RCLCPP_INFO(this->get_logger(), "Running with real_hardware: %s", real_hardware ? "true" : "false");

        bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
        RCLCPP_INFO(this->get_logger(), "Use sim time: %s", use_sim_time ? "true" : "false");

        normal_speed_ = this->get_parameter("normal_speed").as_double();
        emergency_speed_ = this->get_parameter("emergency_speed").as_double();

        // Declare kinematics parameters
        this->declare_parameter("robot_description_kinematics.ur5_manipulator.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
        this->declare_parameter("robot_description_kinematics.ur5_manipulator.kinematics_solver_search_resolution", 0.005);
        this->declare_parameter("robot_description_kinematics.ur5_manipulator.kinematics_solver_timeout", 0.005);
        this->declare_parameter("robot_description_kinematics.ur5_manipulator.kinematics_solver_attempts", 3);

        // Create the action server
        action_server_ = rclcpp_action::create_server<MoveItPose>(
            this,
            "plan_cartesian_execute_pose",
            std::bind(&MoveItPoseActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveItPoseActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&MoveItPoseActionServer::handle_accepted, this, std::placeholders::_1)
        );

        // Create service client for speed control (UR-specific)
        if (real_hardware) {
            speed_slider_client_ = this->create_client<ur_msgs::srv::SetSpeedSliderFraction>(
                "/io_and_status_controller/set_speed_slider");
            
            // Subscribe to speed scaling state to monitor current speed
            speed_scaling_sub_ = this->create_subscription<std_msgs::msg::Float64>(
                "/speed_scaling_state_broadcaster/speed_scaling", 10,
                [this](std_msgs::msg::Float64::SharedPtr msg) {
                    current_speed_scaling_ = msg->data;
                }
            );
            
            RCLCPP_INFO(this->get_logger(), "UR speed control enabled - using hardware speed scaling");
        }

        emergency_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/emergency", 10,
            [this](std_msgs::msg::Bool::SharedPtr msg) {
                bool prev_state = emergency_stop_.load();
                emergency_stop_ = msg->data;
                
                if (emergency_stop_ && !prev_state) {
                    RCLCPP_ERROR(this->get_logger(), "!!! EMERGENCY STOP ACTIVATED !!!");
                    setRobotSpeed(emergency_speed_);  // Slow down or stop via hardware
                    
                    // Also call MoveIt stop as backup
                    if (move_group_) {
                        move_group_->stop();
                    }
                } else if (!emergency_stop_ && prev_state) {
                    RCLCPP_INFO(this->get_logger(), "Emergency cleared - restoring normal speed");
                    setRobotSpeed(normal_speed_);
                }
            }
        );
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

        // Use normal MoveIt velocity scaling - hardware will handle the actual speed
        move_group_->setMaxVelocityScalingFactor(0.3);
        move_group_->setMaxAccelerationScalingFactor(0.3);

        RCLCPP_INFO(this->get_logger(), "Pose reference frame set to: %s", move_group_->getPoseReferenceFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
    }

private:
    std::atomic<bool> emergency_stop_;
    std::atomic<double> current_speed_scaling_{1.0};
    double normal_speed_;
    double emergency_speed_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_scaling_sub_;
    rclcpp::Client<ur_msgs::srv::SetSpeedSliderFraction>::SharedPtr speed_slider_client_;
    rclcpp_action::Server<MoveItPose>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    bool real_hardware;
    double BASE_LINK_X_OFFSET;
    double BASE_LINK_Y_OFFSET;
    double BASE_LINK_Z_OFFSET;
    double BASE_WIDTH = 0.805;
    double BASE_LENGTH = 0.72;
    double BASE_HEIGHT = 0.805;
    double BASE_OFFSET = 0.195;

    // Set robot speed using UR's hardware speed slider
    void setRobotSpeed(double speed_fraction)
    {
        if (!real_hardware || !speed_slider_client_) {
            RCLCPP_DEBUG(this->get_logger(), "Speed control not available (simulation or client not ready)");
            return;
        }

        // Wait for service to be available
        if (!speed_slider_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Speed slider service not available");
            return;
        }

        auto request = std::make_shared<ur_msgs::srv::SetSpeedSliderFraction::Request>();
        request->speed_slider_fraction = std::clamp(speed_fraction, 0.0, 1.0);

        RCLCPP_INFO(this->get_logger(), "Setting robot speed to %.1f%%", request->speed_slider_fraction * 100.0);

        // Call service asynchronously
        auto future = speed_slider_client_->async_send_request(request);
        
        // Wait a bit for the speed change to take effect
        if (future.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Speed successfully set to %.1f%%", request->speed_slider_fraction * 100.0);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to set speed");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Speed setting timed out");
        }
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
                                            std::shared_ptr<const MoveItPose::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received new MoveIt pose goal request");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveItPose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        if (move_group_) {
            move_group_->stop();
        }
        setRobotSpeed(0.0);  // Stop via hardware
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveItPose> goal_handle)
    {
        std::thread([this, goal_handle]() {
            execute(goal_handle);
        }).detach();
    }

    void execute(const std::shared_ptr<GoalHandleMoveItPose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing MoveIt trajectory...");
        auto goal = goal_handle->get_goal();
        auto pose = goal->pose;
        auto result = std::make_shared<MoveItPose::Result>();
        RCLCPP_INFO(this->get_logger(), "Received Pose: position(%.3f, %.3f, %.3f) orientation(%.3f, %.3f, %.3f, %.3f)",
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

        if (emergency_stop_) {
            RCLCPP_ERROR(this->get_logger(), "Emergency Locked! Clear emergency before executing.");
            result->success = false;
            goal_handle->succeed(result);
            return;
        }

        // Ensure robot is at normal speed before starting
        setRobotSpeed(normal_speed_);
        
        geometry_msgs::msg::Pose target_pose;
        tf2::Quaternion orientation;
        orientation.setRPY(pose.orientation.x, pose.orientation.y, pose.orientation.z);
        target_pose.orientation = tf2::toMsg(orientation);
        target_pose.position.x = pose.position.x;
        target_pose.position.y = pose.position.y;
        target_pose.position.z = pose.position.z;

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0; // Disable jump threshold
        const double eef_step = 0.01; // 1 cm

        setupCollisionObjects();

        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if (emergency_stop_) {
            RCLCPP_ERROR(this->get_logger(), "Emergency Locked!");
            result->success = false;
            goal_handle->succeed(result);
            return;
        }

        if (fraction > 0.0)
        {
            // Check for emergency before starting execution
            if (emergency_stop_) {
                RCLCPP_ERROR(this->get_logger(), "Emergency stop detected before execution!");
                result->success = false;
                goal_handle->succeed(result);
                return;
            }

            // Execute with hardware-level monitoring
            bool success = executeWithHardwareMonitoring(trajectory, goal_handle);
            
            if (success && !emergency_stop_) {
                RCLCPP_INFO(this->get_logger(), "Motion executed successfully (%.2f%% of path planned)", fraction * 100.0);
                result->success = true;
            } else {
                if (emergency_stop_) {
                    RCLCPP_ERROR(this->get_logger(), "Motion stopped due to emergency!");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Motion execution failed!");
                }
                result->success = false;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Motion planning failed!");
            result->success = false;
        }

        goal_handle->succeed(result);
    }

    void setupCollisionObjects()
    {
        if (!real_hardware) {
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

            planning_scene_interface.applyCollisionObjects(collision_objects);
        } else {
            std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
            collision_objects.resize(1);

            collision_objects[0].id = "base";
            collision_objects[0].header.frame_id = "world";
            collision_objects[0].primitives.resize(1);
            collision_objects[0].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
            collision_objects[0].primitives[0].dimensions = {BASE_WIDTH, BASE_LENGTH, BASE_HEIGHT};
            collision_objects[0].primitive_poses.resize(1);
            collision_objects[0].primitive_poses[0].position.x = 0.0;
            collision_objects[0].primitive_poses[0].position.y = (BASE_LENGTH / 2) - BASE_OFFSET;
            collision_objects[0].primitive_poses[0].position.z = -(BASE_HEIGHT / 2) - 0.01;
            collision_objects[0].operation = moveit_msgs::msg::CollisionObject::ADD;

            planning_scene_interface.applyCollisionObjects(collision_objects);
        }
    }

    bool executeWithHardwareMonitoring(const moveit_msgs::msg::RobotTrajectory& trajectory,
                                       const std::shared_ptr<GoalHandleMoveItPose>& goal_handle)
    {
        std::atomic<bool> execution_complete{false};
        std::atomic<bool> execution_success{false};

        // Execute trajectory in a separate thread
        std::thread execution_thread([this, &trajectory, &execution_complete, &execution_success]() {
            auto result = move_group_->execute(trajectory);
            execution_success = (result == moveit::core::MoveItErrorCode::SUCCESS);
            execution_complete = true;
        });

        // Monitor for emergency - hardware speed scaling handles the actual stopping
        rclcpp::Rate rate(50);  // 50Hz monitoring is sufficient since hardware handles the stop
        
        while (!execution_complete && rclcpp::ok()) {
            // Check for emergency stop
            if (emergency_stop_) {
                RCLCPP_ERROR(this->get_logger(), "Emergency detected! Hardware speed scaling active.");
                // Speed is already set to emergency_speed_ by the emergency callback
                // The robot will smoothly decelerate due to hardware speed scaling
                // Wait a bit for deceleration
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                
                // Then call MoveIt stop
                move_group_->stop();
                
                // Wait for execution thread to finish
                if (execution_thread.joinable()) {
                    execution_thread.join();
                }
                
                return false;
            }

            // Check for cancellation
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal cancelled");
                setRobotSpeed(0.0);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                move_group_->stop();
                
                if (execution_thread.joinable()) {
                    execution_thread.join();
                }
                
                return false;
            }

            rate.sleep();
        }

        // Wait for execution thread to complete
        if (execution_thread.joinable()) {
            execution_thread.join();
        }

        return execution_success;
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
    auto node = std::make_shared<MoveItPoseActionServer>(options);
    node->init_move_group(); // Initialize MoveGroupInterface after node creation
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);
    // rclcpp::shutdown();
    return 0;
}
