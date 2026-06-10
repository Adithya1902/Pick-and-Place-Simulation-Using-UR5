#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>
#include <sstream>
#include <algorithm>
#include <chrono>

// ==========================================================
// 1. THE GRIPPER CLASS
// ==========================================================
class ControlGripperAction : public BT::SyncActionNode {
public:
    ControlGripperAction(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("command") };
    }

    BT::NodeStatus tick() override {
        std::string command;
        if (!getInput("command", command)) { return BT::NodeStatus::FAILURE; }

        auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        RCLCPP_INFO(node->get_logger(), "Actuating Gripper: %s", command.c_str());

        using GripperCommand = control_msgs::action::GripperCommand;
        auto action_client = rclcpp_action::create_client<GripperCommand>(node, "/gripper_position_controller/gripper_cmd");

        if (!action_client->wait_for_action_server(std::chrono::seconds(3))) {
            RCLCPP_ERROR(node->get_logger(), "Gripper action server not available!");
            return BT::NodeStatus::FAILURE;
        }

        auto goal_msg = GripperCommand::Goal();
        goal_msg.command.position = (command == "close") ? 0.7929 : 0.0;
        goal_msg.command.max_effort = 100.0;

        auto send_goal_future = action_client->async_send_goal(goal_msg);
        if (send_goal_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
            return BT::NodeStatus::FAILURE;
        }

        auto goal_handle = send_goal_future.get();
        if (!goal_handle) { return BT::NodeStatus::FAILURE; }

        auto result_future = action_client->async_get_result(goal_handle);
        result_future.wait();

        return BT::NodeStatus::SUCCESS;
    }
};

// ==========================================================
// 2. THE ARM CLASS (Optimized MoveIt 2 Integration)
// ==========================================================
class MoveArmAction : public BT::SyncActionNode {
public:
    MoveArmAction(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { 
            BT::InputPort<std::string>("target"),
            BT::InputPort<bool>("is_named_pose") 
        };
    }

    BT::NodeStatus tick() override {
        std::string target;
        bool is_named_pose;
        
        if (!getInput("target", target) || !getInput("is_named_pose", is_named_pose)) {
            return BT::NodeStatus::FAILURE;
        }

        auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        
        // Retrieve the persistent MoveIt interface from the Blackboard
        auto move_group = config().blackboard->get<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group");
        
        RCLCPP_INFO(node->get_logger(), "Calculating kinematics for target: %s", target.c_str());

        // Route the logic based on the XML flag
        if (is_named_pose) {
            move_group->setNamedTarget(target);
        } else {
            std::replace(target.begin(), target.end(), ';', ' ');
            std::istringstream iss(target);
            geometry_msgs::msg::Pose target_pose;
            
            iss >> target_pose.position.x >> target_pose.position.y >> target_pose.position.z 
                >> target_pose.orientation.x >> target_pose.orientation.y >> target_pose.orientation.z >> target_pose.orientation.w;

            move_group->setPoseTarget(target_pose);
        }

        // 1. Calculate the path
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success) {
            RCLCPP_ERROR(node->get_logger(), "MoveIt failed to find a valid kinematic path!");
            return BT::NodeStatus::FAILURE;
        }

        // 2. Execute the path on the Gazebo simulation
        RCLCPP_INFO(node->get_logger(), "Path found! Executing trajectory...");
        move_group->execute(my_plan);

        return BT::NodeStatus::SUCCESS;
    }
};

// ==========================================================
// 3. MAIN EXECUTION LOOP
// ==========================================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("pick_and_place_bt_node", node_options);

    std::thread spin_thread([node]() {
        rclcpp::spin(node);
    });

    // Initialize MoveIt 2 Interface HERE, so it stays alive and tracks joint states
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "ur5_manipulator");
    
    // Give MoveIt a second to download the robot's current joint states
    RCLCPP_INFO(node->get_logger(), "Warming up MoveIt interface...");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    BT::BehaviorTreeFactory factory;
    auto blackboard = BT::Blackboard::create();
    
    // Pass both the Node and the MoveGroupInterface to the BT
    blackboard->set<rclcpp::Node::SharedPtr>("node", node);
    blackboard->set<std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>("move_group", move_group);

    factory.registerNodeType<MoveArmAction>("MoveArmAction");
    factory.registerNodeType<ControlGripperAction>("ControlGripperAction");

    std::string xml_path = "/home/adithyabijoy/ur5_ws/src/my_pick_and_place/trees/pick_and_place.xml";
    auto tree = factory.createTreeFromFile(xml_path, blackboard);

    RCLCPP_INFO(node->get_logger(), "Behavior Tree Initialized. Commencing sequence...");
    tree.tickWhileRunning();

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}