#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <atomic>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "ur_msgs/srv/set_speed_slider_fraction.hpp"

#include "custom_interfaces/action/moveit_relative.hpp"

using MoveItRelative = custom_interfaces::action::MoveitRelative;

class MoveItRelativeActionServer : public rclcpp::Node
{
public:
    using GoalHandleMoveItRelative = rclcpp_action::ServerGoalHandle<MoveItRelative>;

    explicit MoveItRelativeActionServer(const rclcpp::NodeOptions &options)
        : Node("moveit_relative_action_server", options), 
          emergency_stop_(false),
          normal_speed_(0.3),
          emergency_speed_(0.0)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing MoveIt relative motion action server...");

        this->declare_parameter<bool>("real_hardware", false);
        this->declare_parameter<double>("normal_speed", 0.3);
        this->declare_parameter<double>("emergency_speed", 0.0);  // 0 = complete stop
        
        real_hardware = this->get_parameter("real_hardware").as_bool();
        normal_speed_ = this->get_parameter("normal_speed").as_double();
        emergency_speed_ = this->get_parameter("emergency_speed").as_double();
        
        RCLCPP_INFO(this->get_logger(), "Running with real_hardware: %s", real_hardware ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Normal speed: %.2f, Emergency speed: %.2f", normal_speed_, emergency_speed_);

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
    rclcpp_action::Server<MoveItRelative>::SharedPtr action_server_;
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
        if (move_group_) {
            move_group_->stop();
        }
        setRobotSpeed(0.0);  // Stop via hardware
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

        if (emergency_stop_) {
            RCLCPP_ERROR(this->get_logger(), "Emergency Locked! Clear emergency before executing.");
            result->success = false;
            goal_handle->succeed(result);
            return;
        }

        // Ensure robot is at normal speed before starting
        setRobotSpeed(normal_speed_);
        
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

        target_pose.position.x -= BASE_LINK_X_OFFSET;
        target_pose.position.y -= BASE_LINK_Y_OFFSET;
        target_pose.position.z -= BASE_LINK_Z_OFFSET;

        // Apply relative rotation
        tf2::Quaternion q_orig, q_rot, q_new;
        tf2::fromMsg(current_pose.orientation, q_orig);
        q_rot.setRPY(goal->roll, goal->pitch, goal->yaw);
        q_new = q_orig * q_rot;
        q_new.normalize();
        target_pose.orientation = tf2::toMsg(q_new);

        RCLCPP_INFO(this->get_logger(), "Target pose: %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
                target_pose.position.x, target_pose.position.y, target_pose.position.z,
                target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);

        // Setup collision objects
        setupCollisionObjects();

        // Plan Cartesian path
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.01;
        const double jump_threshold = 0.0;

        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        feedback->progress = fraction;
        goal_handle->publish_feedback(feedback);

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
            
            goal_handle->succeed(result);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Cartesian path planning failed!");
            result->success = false;
            goal_handle->succeed(result);
        }
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
                                       const std::shared_ptr<GoalHandleMoveItRelative>& goal_handle)
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

    bool real_hardware = false;
    rclcpp::NodeOptions temp_options;
    auto temp_node = std::make_shared<rclcpp::Node>("temp_node_for_args", temp_options);
    temp_node->declare_parameter("real_hardware", rclcpp::ParameterValue(false));
    real_hardware = temp_node->get_parameter("real_hardware").as_bool();

    bool use_sim_time = !real_hardware;

    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"use_sim_time", rclcpp::ParameterValue(use_sim_time)},
        {"real_hardware", rclcpp::ParameterValue(real_hardware)}
    });
    
    auto node = std::make_shared<MoveItRelativeActionServer>(options);
    node->init_move_group();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    return 0;
}