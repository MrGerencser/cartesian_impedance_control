#include <rclcpp/rclcpp.hpp>
#include <messages_fr3/srv/set_pose.hpp>
#include <messages_fr3/srv/set_param.hpp>
#include <messages_fr3/srv/set_stiffness.hpp>
#include <messages_fr3/srv/set_mode.hpp>
#include <messages_fr3/srv/controller_activation.hpp>
#include <messages_fr3/srv/planner_service.hpp>
#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/move.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> // Add this header
#include <std_srvs/srv/set_bool.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <functional>

/**
 * FR3RobotController - A class to control the FR3 robot arm and gripper
 */
class FR3RobotController {
public:
    // Constants for default positions and configurations
    static constexpr double HOME_X = 0.5;
    static constexpr double HOME_Y = 0.0;
    static constexpr double HOME_Z = 0.4;
    static constexpr double HOME_ROLL = M_PI;
    static constexpr double HOME_PITCH = 0.0;
    static constexpr double HOME_YAW = M_PI_2;
    
    static constexpr double SAFETY_Z_OFFSET = 0.05;
    static constexpr double DEFAULT_GRASP_WIDTH = 0.04;
    static constexpr double DEFAULT_GRASP_SPEED = 0.1;
    static constexpr double DEFAULT_GRASP_FORCE = 10.0;
    static constexpr double DEFAULT_INNER_EPSILON = 0.005;
    static constexpr double DEFAULT_OUTER_EPSILON = 0.010;
    static constexpr double DEFAULT_OPEN_WIDTH = 0.08;
    static constexpr double DEFAULT_OPEN_SPEED = 0.1;

    // Add this flag to track object following state
    bool is_following_object_ = false;

    /**
     * Constructor initializes the node and all necessary clients
     */
    FR3RobotController(const std::shared_ptr<rclcpp::Node>& node)
        : node_(node),
          logger_(node->get_logger()),
          has_received_object_pose_(false),
          is_following_object_(false) {
        
        // Initialize service clients
        pose_client_ = node->create_client<messages_fr3::srv::SetPose>("set_pose");
        param_client_ = node->create_client<messages_fr3::srv::SetParam>("set_param");
        mode_client_ = node->create_client<messages_fr3::srv::SetMode>("set_mode");
        activation_client_ = node->create_client<messages_fr3::srv::ControllerActivation>("controller_activation");
        stiffness_client_ = node->create_client<messages_fr3::srv::SetStiffness>("set_stiffness");
        planner_client_ = node->create_client<messages_fr3::srv::PlannerService>("planner_service");
        
        // Initialize action clients
        gripper_grasp_client_ = rclcpp_action::create_client<franka_msgs::action::Grasp>(node, "/fr3_gripper/grasp");
        gripper_move_client_ = rclcpp_action::create_client<franka_msgs::action::Move>(node, "/fr3_gripper/move");
        
        // Initialize requests
        pose_request_ = std::make_shared<messages_fr3::srv::SetPose::Request>();
        param_request_ = std::make_shared<messages_fr3::srv::SetParam::Request>();
        mode_request_ = std::make_shared<messages_fr3::srv::SetMode::Request>();
        activation_request_ = std::make_shared<messages_fr3::srv::ControllerActivation::Request>();
        stiffness_request_ = std::make_shared<messages_fr3::srv::SetStiffness::Request>();
        planner_request_ = std::make_shared<messages_fr3::srv::PlannerService::Request>();
        
        // Subscribe to object pose topic
        object_pose_subscription_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/perception/object_pose", 10, 
            std::bind(&FR3RobotController::objectPoseCallback, this, std::placeholders::_1));
        
        // Wait for services to be available
        waitForServices();
    }

    /**
     * Run the main control loop
     */
    void run() {
        while (rclcpp::ok()) {
            // Process any pending callbacks
            rclcpp::spin_some(node_);
            
            displayMenu();
            
            int task_selection;
            if (!(std::cin >> task_selection)) {
                RCLCPP_ERROR(logger_, "Invalid input. Please enter a number.");
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                continue;
            }
            
            switch (task_selection) {
                case 1:
                    getSafePositionFromUser();
                    break;
                case 2:
                    moveToPosition();
                    break;
                case 3:
                    graspObject();
                    break;
                case 4:
                    moveToHomePosition();
                    break;
                case 5:
                    openGripper();
                    break;
                case 6:
                    moveToObjectPose();
                    break;
                case 7:
                    toggleObjectFollowing();
                    break;
                case 0:
                    is_following_object_ = false; // Make sure to stop following before exiting
                    return; // Exit the loop
                default:
                    RCLCPP_WARN(logger_, "Invalid selection. Please try again.");
                    break;
            }
            
            // Process any pending callbacks again
            rclcpp::spin_some(node_);
        }
    }

private:
    // Node and logger
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Logger logger_;
    
    // Service clients
    rclcpp::Client<messages_fr3::srv::SetPose>::SharedPtr pose_client_;
    rclcpp::Client<messages_fr3::srv::SetParam>::SharedPtr param_client_;
    rclcpp::Client<messages_fr3::srv::SetMode>::SharedPtr mode_client_;
    rclcpp::Client<messages_fr3::srv::ControllerActivation>::SharedPtr activation_client_;
    rclcpp::Client<messages_fr3::srv::SetStiffness>::SharedPtr stiffness_client_;
    rclcpp::Client<messages_fr3::srv::PlannerService>::SharedPtr planner_client_;
    
    // Action clients
    rclcpp_action::Client<franka_msgs::action::Grasp>::SharedPtr gripper_grasp_client_;
    rclcpp_action::Client<franka_msgs::action::Move>::SharedPtr gripper_move_client_;
    
    // Service requests
    std::shared_ptr<messages_fr3::srv::SetPose::Request> pose_request_;
    std::shared_ptr<messages_fr3::srv::SetParam::Request> param_request_;
    std::shared_ptr<messages_fr3::srv::SetMode::Request> mode_request_;
    std::shared_ptr<messages_fr3::srv::ControllerActivation::Request> activation_request_;
    std::shared_ptr<messages_fr3::srv::SetStiffness::Request> stiffness_request_;
    std::shared_ptr<messages_fr3::srv::PlannerService::Request> planner_request_;

    rclcpp::Time last_pose_update_time_;
    double min_pose_update_interval_ = 0.2;  // Minimum time between pose updates (seconds)
    double min_position_change_ = 0.002;     // Minimum position change to trigger an update (meters)
    double min_orientation_change_ = 0.01;   // Minimum orientation change to trigger an update (radians)
    int consecutive_failures_ = 0;
    const int max_consecutive_failures_ = 3;
    
    // Current pose
    std::array<double, 6> current_pose_ = {HOME_X, HOME_Y, HOME_Z, HOME_ROLL, HOME_PITCH, HOME_YAW};

    // Object pose subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_subscription_;
    geometry_msgs::msg::PoseStamped latest_object_pose_;
    bool has_received_object_pose_;

    

    /**
     * Wait for all services to be available
     */
    void waitForServices() {
        RCLCPP_INFO(logger_, "Waiting for services...");
        
        bool services_available = 
            pose_client_->wait_for_service(std::chrono::seconds(5)) &&
            param_client_->wait_for_service(std::chrono::seconds(5)) &&
            mode_client_->wait_for_service(std::chrono::seconds(5)) &&
            activation_client_->wait_for_service(std::chrono::seconds(5)) &&
            stiffness_client_->wait_for_service(std::chrono::seconds(5)) &&
            planner_client_->wait_for_service(std::chrono::seconds(5));
            
        if (!services_available) {
            RCLCPP_WARN(logger_, "Not all services are available. Some functionality may be limited.");
        } else {
            RCLCPP_INFO(logger_, "All services are available.");
        }
    }

    // Add this after waitForServices() in the constructor
    void checkAvailableServices() {
        RCLCPP_INFO(logger_, "Available services:");
        auto services = node_->get_node_names();
        for (const auto& service : services) {
            RCLCPP_INFO(logger_, "  %s", service.c_str());
        }
    }

    /**
     * Display the main menu
     */
    void displayMenu() {
        std::cout << "\n=== FR3 Robot Control Menu ===\n"
                  << " [1] --> Enter 6D Pose from Vision (automatically 10cm above for security)\n"
                  << " [2] --> Move to 6D Position\n"
                  << " [3] --> Grasp object\n"
                  << " [4] --> Move back to home position\n"
                  << " [5] --> Open gripper\n"
                  << " [6] --> Move to detected object pose (no safety offset)\n"
                  << " [7] --> " << (is_following_object_ ? "Stop" : "Start") << " following object\n"
                  << " [0] --> Exit\n"
                  << "Enter your choice: ";
    }
    
    /**
     * Get position from user input with safety offset
     */
    void getSafePositionFromUser() {
        std::cout << "Enter your desired position and orientation (x y z roll pitch yaw):\n";
        
        for (auto &value : current_pose_) {
            if (!(std::cin >> value)) {
                RCLCPP_ERROR(logger_, "Invalid input. Using previous value.");
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                return;
            }
        }
        
        // Create a safe position request (10cm above target)
        updatePoseRequest(current_pose_[0], current_pose_[1], 
                         current_pose_[2] + SAFETY_Z_OFFSET, 
                         current_pose_[3], current_pose_[4], current_pose_[5]);
        
        // Print for verification
        printCurrentPose();
        
        // Send the request
        sendPoseRequest();
    }
    
    /**
     * Move to the previously stored position
     */
    void moveToPosition() {
        int choice;
        std::cout << "Move to 6D Position:\n [1] --> Use stored position\n [2] --> Use home position\n"
                  << "Enter your choice: ";
        
        if (!(std::cin >> choice)) {
            RCLCPP_ERROR(logger_, "Invalid input. Using default.");
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            choice = 2;
        }
        
        if (choice == 1) {
            // Use the stored position (without safety offset)
            updatePoseRequest(current_pose_[0], current_pose_[1], current_pose_[2],
                             current_pose_[3], current_pose_[4], current_pose_[5]);
        } else {
            // Use home position
            updatePoseRequest(HOME_X, HOME_Y, HOME_Z, HOME_ROLL, HOME_PITCH, HOME_YAW);
        }
        
        sendPoseRequest();
    }
    
    /**
     * Send a grasp command to the gripper
     */
    void graspObject() {
        int choice;
        std::cout << "Grasp object:\n [1] --> Yes\n [2] --> No\n"
                  << "Enter your choice: ";
        
        if (!(std::cin >> choice) || choice != 1) {
            RCLCPP_INFO(logger_, "Grasp canceled.");
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            return;
        }
        
        // Create and send grasp goal
        auto grasp_goal = franka_msgs::action::Grasp::Goal();
        grasp_goal.width = DEFAULT_GRASP_WIDTH;
        grasp_goal.speed = DEFAULT_GRASP_SPEED;
        grasp_goal.force = DEFAULT_GRASP_FORCE;
        grasp_goal.epsilon.inner = DEFAULT_INNER_EPSILON;
        grasp_goal.epsilon.outer = DEFAULT_OUTER_EPSILON;
        
        auto send_goal_options = rclcpp_action::Client<franka_msgs::action::Grasp>::SendGoalOptions();
        
        send_goal_options.result_callback = 
            [this](const rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>::WrappedResult& result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(this->logger_, "Grasp succeeded!");
                } else {
                    RCLCPP_ERROR(this->logger_, "Grasp failed with code: %d", static_cast<int>(result.code));
                }
            };
        
        RCLCPP_INFO(logger_, "Sending grasp command...");
        gripper_grasp_client_->async_send_goal(grasp_goal, send_goal_options);
    }
    
    /**
     * Move the robot to the home position with safety Z movement first
     */
    void moveToHomePosition() {
        int choice;
        std::cout << "Move back to home position:\n [1] --> Yes\n [2] --> No\n"
                << "Enter your choice: ";
        
        if (!(std::cin >> choice) || choice != 1) {
            RCLCPP_INFO(logger_, "Move canceled.");
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            return;
        }
        
        // Step 1: First move up in Z by 10cm from current position
        RCLCPP_INFO(logger_, "Step 1: Moving up by 10cm for safety...");
        updatePoseRequest(
            current_pose_[0],           // Keep current X
            current_pose_[1],           // Keep current Y
            current_pose_[2] + 0.10,    // Current Z + 10cm
            current_pose_[3],           // Keep current roll
            current_pose_[4],           // Keep current pitch
            current_pose_[5]            // Keep current yaw
        );
        sendPoseRequest();
        
        // Wait for 1 second to allow the first movement to complete
        RCLCPP_INFO(logger_, "Waiting for safety movement to complete...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // Step 2: Move to the home position
        RCLCPP_INFO(logger_, "Step 2: Moving to home position...");
        updatePoseRequest(HOME_X, HOME_Y, HOME_Z, HOME_ROLL, HOME_PITCH, HOME_YAW);
        sendPoseRequest();
    }
    
    /**
     * Open the gripper
     */
    void openGripper() {
        int choice;
        std::cout << "Open gripper:\n [1] --> Yes\n [2] --> No\n"
                  << "Enter your choice: ";
        
        if (!(std::cin >> choice) || choice != 1) {
            RCLCPP_INFO(logger_, "Open gripper canceled.");
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            return;
        }
        
        // Create and send move goal to open the gripper
        auto move_goal = franka_msgs::action::Move::Goal();
        move_goal.width = DEFAULT_OPEN_WIDTH;
        move_goal.speed = DEFAULT_OPEN_SPEED;
        
        auto send_goal_options = rclcpp_action::Client<franka_msgs::action::Move>::SendGoalOptions();
        
        send_goal_options.result_callback = 
            [this](const rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>::WrappedResult& result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(this->logger_, "Gripper opened successfully!");
                } else {
                    RCLCPP_ERROR(this->logger_, "Failed to open gripper with code: %d", static_cast<int>(result.code));
                }
            };
        
        RCLCPP_INFO(logger_, "Sending open gripper command...");
        gripper_move_client_->async_send_goal(move_goal, send_goal_options);
    }
    
    /**
     * Move to the detected object pose
     */
    void moveToObjectPose() {
        if (!has_received_object_pose_) {
            RCLCPP_WARN(logger_, "No object pose has been received yet. Waiting for pose...");
            
            // Wait a bit for a pose to arrive
            rclcpp::Time start_time = node_->now();
            while (rclcpp::ok() && !has_received_object_pose_ && 
                (node_->now() - start_time).seconds() < 5.0) {
                rclcpp::spin_some(node_);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            if (!has_received_object_pose_) {
                RCLCPP_ERROR(logger_, "Timeout waiting for object pose. Please try again later.");
                return;
            }
        }
        
        int choice;
        std::cout << "Move to detected object pose (with two-step approach):\n [1] --> Yes\n [2] --> No\n"
                << "Enter your choice: ";
        
        if (!(std::cin >> choice) || choice != 1) {
            RCLCPP_INFO(logger_, "Move canceled.");
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            return;
        }
        
        // Process any new messages that might have arrived during user input
        RCLCPP_INFO(logger_, "Getting fresh object position...");
        for (int i = 0; i < 10; i++) {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        // Step 1: Move to current object pose with safety offset
        RCLCPP_INFO(logger_, "Step 1: Moving to safety position above object...");
        moveToCurrentObjectPose(true);
        
        // Wait for motion to complete
        std::cout << "Waiting for safe position move to complete..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Process any new messages that might have arrived while moving
        RCLCPP_INFO(logger_, "Getting updated object position for final move...");
        for (int i = 0; i < 10; i++) {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        // Step 2: Move to the exact object pose
        RCLCPP_INFO(logger_, "Step 2: Moving to final object position...");
        moveToCurrentObjectPose(false);
    }

    /**
     * Toggle object following mode
     */
    void toggleObjectFollowing() {
        is_following_object_ = !is_following_object_;
        
        // Create a client for the toggle_object_following service
        auto toggle_client = node_->create_client<std_srvs::srv::SetBool>("toggle_object_following");
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        
        if (is_following_object_) {
            RCLCPP_INFO(logger_, "Starting to follow object. Press '7' again to stop following.");
            request->data = true;
            
            // Initialize the time variable to prevent time source error
            last_pose_update_time_ = node_->now();
            consecutive_failures_ = 0;  // Reset failure counter
            
            // Move to current object pose to start following
            if (has_received_object_pose_) {
                moveToCurrentObjectPose(false);
            } else {
                RCLCPP_WARN(logger_, "No object pose received yet. Will follow when pose is detected.");
            }
        } else {
            RCLCPP_INFO(logger_, "Stopped following object.");
            request->data = false;
        }
        
        // Send request to controller
        if (toggle_client->wait_for_service(std::chrono::seconds(1))) {
            auto future = toggle_client->async_send_request(request);
            rclcpp::spin_until_future_complete(node_, future);
        } else {
            RCLCPP_WARN(logger_, "Toggle following service not available");
        }
    }

    /**
     * Move to the current object pose
     * @param with_safety_offset Whether to add a safety offset in Z
     * @param from_callback Whether this call is coming from a callback
     */
    void moveToCurrentObjectPose(bool with_safety_offset = false, bool from_callback = false) {
        // Convert pose to roll, pitch, yaw
        double roll, pitch, yaw;
        quaternionToRPY(latest_object_pose_.pose.orientation, roll, pitch, yaw);
        
        // Adjust roll to make zero orientation be (pi, 0, 0) instead of (0, 0, 0)
        roll = M_PI + roll;
        
        // Set z position based on whether safety offset is desired
        double z_position = latest_object_pose_.pose.position.z;
        if (with_safety_offset) {
            z_position += SAFETY_Z_OFFSET;
        }
        
        // Create position request
        updatePoseRequest(
            latest_object_pose_.pose.position.x,
            latest_object_pose_.pose.position.y,
            z_position,
            roll, pitch, yaw
        );
        
        // Update current_pose_ for future reference
        current_pose_[0] = latest_object_pose_.pose.position.x;
        current_pose_[1] = latest_object_pose_.pose.position.y;
        current_pose_[2] = latest_object_pose_.pose.position.z;
        current_pose_[3] = roll;
        current_pose_[4] = pitch;
        current_pose_[5] = yaw;
        
        // Print for verification
        printCurrentPose();
        
        // Send the request with the from_callback parameter
        sendPoseRequest(from_callback);
    }

    /**
     * Callback function for object pose subscription
     */
    void objectPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // Store the new pose
        latest_object_pose_ = *msg;
        has_received_object_pose_ = true;
        
        // Convert quaternion to roll, pitch, yaw for debugging
        double roll, pitch, yaw;
        quaternionToRPY(msg->pose.orientation, roll, pitch, yaw);
        
        // Log the new pose (less frequently to reduce console spam)
        static int log_counter = 0;
        if (log_counter++ % 10 == 0) {
            RCLCPP_INFO(logger_, "Received object pose: x=%.3f, y=%.3f, z=%.3f, roll=%.3f (adjusted=%.3f), pitch=%.3f, yaw=%.3f",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                    roll, M_PI + roll, pitch, yaw);
        }
        
        // If in following mode, check if we should update the robot position
        if (is_following_object_) {
            // Check if enough time has passed since the last update
            auto current_time = node_->now();
            if ((current_time - last_pose_update_time_).seconds() < min_pose_update_interval_) {
                return;  // Skip this update, it's too soon
            }
            
            // Check if the pose has changed enough to warrant an update
            if (has_pose_changed_significantly()) {
                // If we've had too many consecutive failures, temporarily pause following
                if (consecutive_failures_ >= max_consecutive_failures_) {
                    RCLCPP_WARN(logger_, "Too many consecutive pose request failures. Pausing following for 2 seconds.");
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    consecutive_failures_ = 0;
                }
                
                RCLCPP_INFO(logger_, "Following object - updating robot position");
                last_pose_update_time_ = current_time;
                moveToCurrentObjectPose(false, true);  // No safety offset in follow mode, from callback=true
            }
        }
    }

    /**
     * Check if the pose has changed significantly
     */
    bool has_pose_changed_significantly() {
        // If this is the first update, consider it significant
        if (current_pose_[0] == HOME_X && current_pose_[1] == HOME_Y && current_pose_[2] == HOME_Z) {
            return true;
        }
        
        // Calculate position change
        double position_change = std::sqrt(
            std::pow(latest_object_pose_.pose.position.x - current_pose_[0], 2) +
            std::pow(latest_object_pose_.pose.position.y - current_pose_[1], 2) +
            std::pow(latest_object_pose_.pose.position.z - current_pose_[2], 2)
        );
        
        // Calculate orientation change
        double roll, pitch, yaw;
        quaternionToRPY(latest_object_pose_.pose.orientation, roll, pitch, yaw);
        roll = M_PI + roll;
        
        double orientation_change = std::sqrt(
            std::pow(roll - current_pose_[3], 2) +
            std::pow(pitch - current_pose_[4], 2) +
            std::pow(yaw - current_pose_[5], 2)
        );
        
        return position_change > min_position_change_ || orientation_change > min_orientation_change_;
    }
    
    /**
     * Convert quaternion to roll, pitch, yaw angles
     */
    void quaternionToRPY(const geometry_msgs::msg::Quaternion& q, double& roll, double& pitch, double& yaw) {
        // Convert quaternion to Euler angles (roll, pitch, yaw)
        // roll (x-axis rotation)
        double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        roll = std::atan2(sinr_cosp, cosr_cosp);
        
        // pitch (y-axis rotation)
        double sinp = 2.0 * (q.w * q.y - q.z * q.x);
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            pitch = std::asin(sinp);
        
        // yaw (z-axis rotation)
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }
    
    /**
     * Update the pose request with the given parameters
     */
    void updatePoseRequest(double x, double y, double z, double roll, double pitch, double yaw) {
        pose_request_->x = x;
        pose_request_->y = y;
        pose_request_->z = z;
        pose_request_->roll = roll;
        pose_request_->pitch = pitch;
        pose_request_->yaw = yaw;
    }

    /**
     * Send the pose request to the robot without waiting for response
     */
    void sendPoseRequest(bool from_callback = false) {
        if (!pose_client_->wait_for_service(std::chrono::milliseconds(500))) {
            RCLCPP_ERROR(logger_, "Service not available. Cannot send pose request.");
            return;
        }
        
        RCLCPP_INFO(logger_, "Sending pose request...");
        
        // Fire and forget - send the request with a callback that just logs
        auto callback = [this](rclcpp::Client<messages_fr3::srv::SetPose>::SharedFuture future) {
            try {
                // Log success but don't block on the future result
                RCLCPP_INFO(this->logger_, "Pose request callback triggered");
                consecutive_failures_ = 0;
            } catch (const std::exception& e) {
                // Only log errors but keep going
                RCLCPP_WARN(this->logger_, "Exception in pose request callback: %s", e.what());
            }
        };
        
        // Send the request asynchronously and immediately return
        pose_client_->async_send_request(pose_request_, callback);
        
        // Assume success and continue without waiting
        RCLCPP_INFO(logger_, "Pose request sent, continuing without waiting for response");
    }


    /**
     * Print the current pose for verification
     */
    void printCurrentPose() {
        std::cout << "Current pose: x=" << current_pose_[0]
                  << ", y=" << current_pose_[1]
                  << ", z=" << current_pose_[2]
                  << ", roll=" << current_pose_[3]
                  << ", pitch=" << current_pose_[4]
                  << ", yaw=" << current_pose_[5] << std::endl;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("fr3_robot_controller");
    FR3RobotController controller(node);
    controller.run();
    rclcpp::shutdown();
    return 0;
}