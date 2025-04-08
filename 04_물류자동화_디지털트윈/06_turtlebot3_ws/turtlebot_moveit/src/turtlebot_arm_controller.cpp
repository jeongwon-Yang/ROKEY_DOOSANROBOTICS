#include <rclcpp/rclcpp.hpp>
#include <turtlebot_cosmo_interface/srv/moveit_control.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_array.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>  
#include <vector>
#include <tuple>

#define PI 3.14159265358979323846
class TurtlebotArmController : public rclcpp::Node {
public:
    TurtlebotArmController(const rclcpp::NodeOptions &options) : Node("turtlebot_arm_controller",options),
    node_(std::make_shared<rclcpp::Node>("move_group_interface")), // Create an additional ROS node
    move_group_interface_(node_, "arm"), // Initialize MoveGroupInterface for controlling the arm
    executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) // Create a single-threaded executor
  
    {
        service_ = this->create_service<turtlebot_cosmo_interface::srv::MoveitControl>(
            "moveit_control", std::bind(&TurtlebotArmController::handleMoveitControl, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Ready to receive MoveitControl commands.");
        executor_->add_node(node_);
        executor_thread_ = std::thread([this]() {
        RCLCPP_INFO(node_->get_logger(), "Starting executor thread"); // Log message indicating the thread start
        executor_->spin(); // Run the executor to process callbacks
        });
    }

    void printCurrentPose() {
        auto current_pose = move_group_interface_.getCurrentPose().pose; // Get the current pose
        std::cout << "Current Pose:" << std::endl;

        // 쿼터니언 가져오기
        const auto& orientation = current_pose.orientation;

        // tf2::Quaternion 객체로 변환
        tf2::Quaternion quaternion(orientation.x, orientation.y, orientation.z, orientation.w);

        // RPY 계산
        double roll, pitch, yaw;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

        // RPY 출력
        std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;


        std::cout<<"yaw : "<<yaw<< std::endl;  
        std::cout << "Position: (" << current_pose.position.x << ", "
                << current_pose.position.y << ", "
                << current_pose.position.z << ")" << std::endl;
        std::cout << "Orientation: (" << current_pose.orientation.x << ", "
                << current_pose.orientation.y << ", "
                << current_pose.orientation.z << ", "
                << current_pose.orientation.w << ")" << std::endl;
    }
    geometry_msgs::msg::Quaternion rpyToQuaternion(double roll, double pitch, double yaw) {
        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

        geometry_msgs::msg::Quaternion quaternion;
        quaternion.w = q.w();
        quaternion.x = q.x();
        quaternion.y = q.y();
        quaternion.z = q.z();
        return quaternion;
    }


    bool planAndExecute(moveit::planning_interface::MoveGroupInterface& move_group_interface, moveit::planning_interface::MoveGroupInterface::Plan& plan) {
        bool success = static_cast<bool>(move_group_interface.plan(plan));
        if (success) {
            move_group_interface.execute(plan);
            return true;
        } else {
            return false;
        }
    }

    geometry_msgs::msg::Quaternion multiply(const geometry_msgs::msg::Quaternion& q2, const geometry_msgs::msg::Quaternion& q1) {
        geometry_msgs::msg::Quaternion out;
        out.w = q2.w*q1.w - q2.x*q1.x - q2.y*q1.y - q2.z*q1.z;
        out.x = q2.w*q1.x + q2.x*q1.w + q2.y*q1.z - q2.z*q1.y;
        out.y = q2.w*q1.y - q2.x*q1.z + q2.y*q1.w + q2.z*q1.x;
        out.z = q2.w*q1.z + q2.x*q1.y - q2.y*q1.x + q2.z*q1.w;
        return out;
    }


private:
    void handleMoveitControl(const std::shared_ptr<turtlebot_cosmo_interface::srv::MoveitControl::Request> req,
                             std::shared_ptr<turtlebot_cosmo_interface::srv::MoveitControl::Response> res) {
        
        // moveit core 의 arm 그룹을 사용하기위한 선언
        using moveit::planning_interface::MoveGroupInterface;
        auto arm_interface = MoveGroupInterface(shared_from_this(), "arm");
        arm_interface.setPlanningPipelineId("move_group");
        arm_interface.setPlanningTime(10.0); 
        arm_interface.setGoalPositionTolerance(0.01);
        arm_interface.setGoalOrientationTolerance(0.05);

        // moveit core 의 gripper 그룹을 사용하기위한 선언
        auto gripper_interface = MoveGroupInterface(shared_from_this(), "gripper");
        

        // 좌표를 통한 이동 
        if (req->cmd == 0) {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            for (const auto &pose : req->waypoints.poses) {
                waypoints.push_back(pose);
            }
            auto _pose = waypoints[0];
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto current_pose = arm_interface.getCurrentPose().pose;
            float target_z = _pose.position.z;
            float target_x = _pose.position.x;
            float target_y = _pose.position.y;
            double yaw = atan2(target_y, target_x);
            std::cout<<"yaw : "<<yaw;  
            // 원점 좌표에서 로봇팔 좌표의 축차이를 직접계산해서 넣어줌
            // geometry_msgs::msg::Quaternion multiply_quaternion = multiply(rpyToQuaternion(0, 0, yaw),current_pose.orientation);

            geometry_msgs::msg::Quaternion multiply_quaternion = multiply(rpyToQuaternion(0, 0, yaw),rpyToQuaternion(0, PI/2, 0));
            std::cout << "multiply_quaternion: " << multiply_quaternion.x << " " << multiply_quaternion.y << " " << multiply_quaternion.z << " " << multiply_quaternion.w << std::endl;

            current_pose = arm_interface.getCurrentPose().pose;
            auto target_pose = [multiply_quaternion, target_x, target_y, target_z]{
                geometry_msgs::msg::Pose msg;
                msg.orientation = multiply_quaternion;
                msg.position.x = target_x;
                msg.position.y = target_y;
                msg.position.z = target_z; 
                return msg;
            }();
            arm_interface.setPoseTarget(target_pose);
            planAndExecute(arm_interface, plan);

        }

        else if (req->cmd == 1) {
            arm_interface.setNamedTarget(req->posename);
            // res->response = (arm_interface.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            res->response = (arm_interface.move() == moveit::core::MoveItErrorCode::SUCCESS);

        } 

        else if (req->cmd == 2) {
            gripper_interface.setNamedTarget(req->posename);
            // res->response = (gripper_interface.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            res->response = (gripper_interface.move() == moveit::core::MoveItErrorCode::SUCCESS);

        } 

        else if (req->cmd == 3){
            std::vector<geometry_msgs::msg::Pose> waypoints;
            for (const auto &pose : req->waypoints.poses) {
                waypoints.push_back(pose);
            }
            auto _pose = waypoints[0];
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto current_pose = arm_interface.getCurrentPose().pose;
            float target_z = _pose.position.z;
            float target_x = _pose.position.x;
            float target_y = _pose.position.y;
            double yaw = atan2(target_x, target_y);  
            std::cout<<yaw;
            geometry_msgs::msg::Quaternion multiply_quaternion = multiply(rpyToQuaternion(0, 0, yaw),rpyToQuaternion(0, 0, M_PI/2));

            current_pose = arm_interface.getCurrentPose().pose;
            auto target_pose = [multiply_quaternion, target_x, target_y, target_z]{
                geometry_msgs::msg::Pose msg;
                msg.orientation = multiply_quaternion;
                msg.position.x = target_x;
                msg.position.y = target_y;
                msg.position.z = target_z; 
                return msg;
            }();
            arm_interface.setPoseTarget(target_pose);
            planAndExecute(arm_interface, plan);
        }


        else if (req->cmd == 4){
            std::vector<geometry_msgs::msg::Pose> waypoints;
            for (const auto &pose : req->waypoints.poses) {
                waypoints.push_back(pose);
            }
            auto _pose = waypoints[0];
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto current_pose = arm_interface.getCurrentPose().pose;
            float target_z = _pose.position.z;
            float target_x = _pose.position.x;
            float target_y = _pose.position.y;
            float quaternion_x = _pose.orientation.x;
            float quaternion_y = _pose.orientation.y;
            float quaternion_z = _pose.orientation.z;
            float quaternion_w = _pose.orientation.w;
            
            // double yaw = atan2(target_x, target_y);  
            // std::cout<<yaw;

            current_pose = arm_interface.getCurrentPose().pose;
            auto target_pose = [quaternion_x, quaternion_y, quaternion_z, quaternion_w, target_x, target_y, target_z]{
                geometry_msgs::msg::Pose msg;
                msg.orientation.x = quaternion_x;
                msg.orientation.y = quaternion_y;
                msg.orientation.z = quaternion_z;
                msg.orientation.w = quaternion_w;
                msg.position.x = target_x;
                msg.position.y = target_y;
                msg.position.z = target_z; 
                return msg;
            }();
            arm_interface.setPoseTarget(target_pose);
            planAndExecute(arm_interface, plan);
        }

        else if (req->cmd==9){
            printCurrentPose();
        }


        else {
            res->response = false;
        }
    }

    rclcpp::Service<turtlebot_cosmo_interface::srv::MoveitControl>::SharedPtr service_;
    rclcpp::Node::SharedPtr node_; // Additional ROS node pointer
    moveit::planning_interface::MoveGroupInterface move_group_interface_;  // MoveIt interface for controlling the arm
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;  // Single-threaded executor
    std::thread executor_thread_;  // Thread to run the executor
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true); // Allow automatic parameter declaration
    node_options.use_intra_process_comms(false); // Disable intra-process communication
    auto node = std::make_shared<TurtlebotArmController>(node_options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}