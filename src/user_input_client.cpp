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
    
    static constexpr double SAFETY_Z_OFFSET = 0.1;
    static constexpr double DEFAULT_GRASP_WIDTH = 0.04;
    static constexpr double DEFAULT_GRASP_SPEED = 0.1;
    static constexpr double DEFAULT_GRASP_FORCE = 10.0;
    static constexpr double DEFAULT_INNER_EPSILON = 0.005;
    static constexpr double DEFAULT_OUTER_EPSILON = 0.010;
    static constexpr double DEFAULT_OPEN_WIDTH = 0.08;
    static constexpr double DEFAULT_OPEN_SPEED = 0.1;

    /**
     * Constructor initializes the node and all necessary clients
     */
    FR3RobotController(const std::shared_ptr<rclcpp::Node>& node)
        : node_(node),
          logger_(node->get_logger()) {
        
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
        
        // Wait for services to be available
        waitForServices();
    }

    /**
     * Run the main control loop
     */
    void run() {
        while (rclcpp::ok()) {
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
                case 0:
                    return; // Exit the loop
                default:
                    RCLCPP_WARN(logger_, "Invalid selection. Please try again.");
                    break;
            }
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
    
    // Current pose
    std::array<double, 6> current_pose_ = {HOME_X, HOME_Y, HOME_Z, HOME_ROLL, HOME_PITCH, HOME_YAW};

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
     * Move the robot to the home position
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
        
        // Use home position
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
     * Update the pose request with new values
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
     * Print the current pose for debugging
     */
    void printCurrentPose() {
        RCLCPP_INFO(logger_, "Current pose: x: %.3f, y: %.3f, z: %.3f, roll: %.3f, pitch: %.3f, yaw: %.3f",
                  pose_request_->x, pose_request_->y, pose_request_->z,
                  pose_request_->roll, pose_request_->pitch, pose_request_->yaw);
    }
    
    /**
     * Send the pose request and handle the response
     */
    void sendPoseRequest() {
        RCLCPP_INFO(logger_, "Sending pose request...");
        
        auto pose_result = pose_client_->async_send_request(
            pose_request_,
            [this](rclcpp::Client<messages_fr3::srv::SetPose>::SharedFuture future) {
                auto result = future.get();
                if (result->success) {
                    RCLCPP_INFO(this->logger_, "Pose set successfully.");
                } else {
                    RCLCPP_ERROR(this->logger_, "Failed to set pose.");
                }
            }
        );
    }
};

/**
 * Main function
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("user_input_client");
    
    FR3RobotController controller(node);
    controller.run();
    
    rclcpp::shutdown();
    return 0;
}