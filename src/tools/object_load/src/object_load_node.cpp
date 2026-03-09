#include <memory>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/mesh.hpp>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>

#include <Eigen/Geometry>
#include <boost/variant/get.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("object_load_node");
  auto logger = node->get_logger();

  const std::string object_id = node->declare_parameter<std::string>("object_id", "target_object");
  const std::string frame_id = node->declare_parameter<std::string>("frame_id", "world");
  const std::string mesh_resource = node->declare_parameter<std::string>(
      "mesh_resource", "package://object_load/meshes/PlacementArea.STL");

  const double px = node->declare_parameter<double>("pose.position.x", 1.0);
  const double py = node->declare_parameter<double>("pose.position.y", 0.0);
  const double pz = node->declare_parameter<double>("pose.position.z", 0.9);

  const double ox = node->declare_parameter<double>("pose.orientation.x", 0.0);
  const double oy = node->declare_parameter<double>("pose.orientation.y", 0.0);
  const double oz = node->declare_parameter<double>("pose.orientation.z", 0.0);
  const double ow = node->declare_parameter<double>("pose.orientation.w", 1.0);

  const double sx = node->declare_parameter<double>("scale.x", 0.001);
  const double sy = node->declare_parameter<double>("scale.y", 0.001);
  const double sz = node->declare_parameter<double>("scale.z", 0.001);
  // STL files are often in millimeters; set scale to 0.001 if needed.

  const double wait_timeout = node->declare_parameter<double>("wait_service_timeout_sec", 5.0);

  const std::string move_group_name =
      node->declare_parameter<std::string>("move_group_name", "engineer_arm");

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> planning_scene;
  try
  {
    planning_scene = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node, move_group_name);
  }
  catch (const std::exception &ex)
  {
    RCLCPP_ERROR(logger, "Failed to create MoveGroupInterface for group '%s': %s",
                 move_group_name.c_str(), ex.what());
    rclcpp::shutdown();
    return 1;
  }

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  (void)planning_scene;

  auto client = node->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
  RCLCPP_INFO(logger, "Waiting for /apply_planning_scene service (timeout: %.2f sec)...",
              wait_timeout);
  if (!client->wait_for_service(std::chrono::duration<double>(wait_timeout)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(logger, "ROS shutdown before /apply_planning_scene became available.");
    }
    else
    {
      RCLCPP_ERROR(logger, "Timed out waiting for /apply_planning_scene service.");
    }
    rclcpp::shutdown();
    return 1;
  }

  const Eigen::Vector3d scale(sx, sy, sz);
  std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(mesh_resource, scale));
  if (!mesh)
  {
    RCLCPP_ERROR(logger, "Failed to load mesh from resource: %s", mesh_resource.c_str());
    rclcpp::shutdown();
    return 1;
  }

  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(mesh.get(), mesh_msg);
  shape_msgs::msg::Mesh mesh_shape = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

  geometry_msgs::msg::Pose pose;
  pose.position.x = px;
  pose.position.y = py;
  pose.position.z = pz;
  pose.orientation.x = ox;
  pose.orientation.y = oy;
  pose.orientation.z = oz;
  pose.orientation.w = ow;

  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = frame_id;
  obj.id = object_id;
  obj.meshes.push_back(mesh_shape);
  obj.mesh_poses.push_back(pose);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;

  RCLCPP_INFO(logger, "Mesh resource: %s", mesh_resource.c_str());
  RCLCPP_INFO(logger, "Object ID: %s", object_id.c_str());
  RCLCPP_INFO(logger, "Frame ID: %s", frame_id.c_str());
  RCLCPP_INFO(logger, "Pose position: [%.6f, %.6f, %.6f]", px, py, pz);
  RCLCPP_INFO(logger, "Pose orientation: [%.6f, %.6f, %.6f, %.6f]", ox, oy, oz, ow);
  RCLCPP_INFO(logger, "Scale: [%.6f, %.6f, %.6f]", sx, sy, sz);

  const bool ok = planning_scene_interface.applyCollisionObject(obj);
  RCLCPP_INFO(logger, "applyCollisionObject result: %s", ok ? "true" : "false");

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
