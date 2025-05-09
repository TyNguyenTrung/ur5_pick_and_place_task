#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometric_shapes/shape_operations.h>

class PickPlaceDemo
{
public:
  PickPlaceDemo(const rclcpp::Node::SharedPtr& node)
    : node_(node),
      arm_(node_, "arm"),
      gripper_(node_, "gripper")
  {
    using moveit::planning_interface::MoveGroupInterface;

    // Configure planning
    arm_.setPlannerId("ompl");
    arm_.setPlanningTime(5.0);

    moveit_msgs::msg::CollisionObject object;
    object.id = "object";
    object.header.frame_id = "world";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = { 0.1, 0.02 };

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.5;
    pose.position.y = -0.25;
    pose.position.z = 0.35;
    pose.orientation.w = 1.0;
    object.pose = pose;

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);


    // Step 1: Open gripper
    RCLCPP_INFO(node_->get_logger(), "Opening gripper...");
    gripper_.setNamedTarget("open");
    gripper_.move();

    // Step 2: Move to pick pose
    RCLCPP_INFO(node_->get_logger(), "Moving to pick pose...");
    geometry_msgs::msg::Pose pick_pose;
    pick_pose.position.x = 0.2;
    pick_pose.position.y = -0.25;
    pick_pose.position.z = 0.35;
    pick_pose.orientation.w = 1.0;

    arm_.setPoseTarget(pick_pose);
    arm_.move();


    arm_.setPlanningPipelineId("pilz_industrial_motion_planner");
    arm_.setPlannerId("LIN");

    RCLCPP_INFO(node_->get_logger(), "Approaching object...");
    geometry_msgs::msg::Pose aproaching_pose = pick_pose;
    aproaching_pose.position.x += 0.17;
    arm_.setPoseTarget(aproaching_pose);
    arm_.move();

    // Step 3: Open gripper
    RCLCPP_INFO(node_->get_logger(), "Closing gripper...");
    gripper_.setNamedTarget("close");
    gripper_.move();

    gripper_.attachObject(object.id, std::string("robotiq_85_base_link"));

    // Step 4: Lift up
    arm_.setPlanningPipelineId("pilz_industrial_motion_planner");
    arm_.setPlannerId("LIN");

    RCLCPP_INFO(node_->get_logger(), "Lifting object...");
    geometry_msgs::msg::Pose lift_pose = aproaching_pose;
    lift_pose.position.z += 0.2;
    arm_.setPoseTarget(lift_pose);
    arm_.move();

    // // Step 5: Move to place position
    arm_.setPlanningPipelineId("pilz_industrial_motion_planner");
    arm_.setPlannerId("PTP");
    RCLCPP_INFO(node_->get_logger(), "Moving to place position...");
    geometry_msgs::msg::Pose place_pose = lift_pose;
    place_pose.position.x = 0.3;
    place_pose.position.y = 0.2;
    arm_.setPoseTarget(place_pose);
    arm_.move();

    // Step 6: Open gripper to release
    gripper_.detachObject(object.id);
    RCLCPP_INFO(node_->get_logger(), "Releasing object...");
    gripper_.setNamedTarget("open");
    gripper_.move();
    


    // // Step 7: Retreat
    arm_.setPlanningPipelineId("pilz_industrial_motion_planner");
    arm_.setPlannerId("LIN");
    RCLCPP_INFO(node_->get_logger(), "Retreating...");
    geometry_msgs::msg::Pose retreat_pose = place_pose;
    retreat_pose.position.x -= 0.1;
    arm_.setPoseTarget(retreat_pose);
    arm_.move();

    RCLCPP_INFO(node_->get_logger(), "Pick and place done!");
  }

private:
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface arm_;
  moveit::planning_interface::MoveGroupInterface gripper_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pick_place_node");


  // Create demo (will run logic in constructor)
  auto demo = std::make_shared<PickPlaceDemo>(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
