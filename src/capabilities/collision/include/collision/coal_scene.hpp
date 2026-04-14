#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <coal/collision.h>
#include <coal/collision_object.h>
#include <coal/shape/geometric_shapes.h>

namespace collision
{

enum class CollisionObjectRole
{
  Self,
  External,
};

enum class CollisionGeometryType
{
  Box,
  Sphere,
  Capsule,
  Cylinder,
};

struct CollisionGeometrySpec
{
  CollisionGeometryType type{ CollisionGeometryType::Box };
  Eigen::Vector3d size{ 1.0, 1.0, 1.0 };
  double radius{ 0.5 };
  double length{ 1.0 };

  static CollisionGeometrySpec makeBox(double x, double y, double z);
  static CollisionGeometrySpec makeSphere(double radius);
  static CollisionGeometrySpec makeCapsule(double radius, double length);
  static CollisionGeometrySpec makeCylinder(double radius, double length);
};

struct CollisionObjectSpec
{
  std::string id;
  CollisionObjectRole role{ CollisionObjectRole::Self };
  CollisionGeometrySpec geometry;
  Eigen::Isometry3d pose{ Eigen::Isometry3d::Identity() };
  bool enabled{ true };
};

struct CollisionPair
{
  std::string first_id;
  std::string second_id;
  CollisionObjectRole first_role{ CollisionObjectRole::Self };
  CollisionObjectRole second_role{ CollisionObjectRole::Self };
  double penetration_depth{ 0.0 };
  Eigen::Vector3d position{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d normal{ Eigen::Vector3d::Zero() };
};

struct CollisionCheckResult
{
  bool collision_free{ true };
  std::vector<CollisionPair> pairs;
  std::string message;
};

struct CollisionSceneOptions
{
  std::size_t max_contacts_per_pair{ 1 };
  std::size_t max_pairs{ 16 };
  double security_margin{ 0.0 };
};

class CollisionScene
{
public:
  explicit CollisionScene(CollisionSceneOptions options = {});

  bool upsertObject(CollisionObjectSpec spec, std::string& err);
  bool removeObject(const std::string& id);
  void clear();
  bool setPose(const std::string& id, const Eigen::Isometry3d& pose);
  bool setEnabled(const std::string& id, bool enabled);
  bool contains(const std::string& id) const;
  std::vector<std::string> ids(CollisionObjectRole role) const;

  bool checkSelfCollision(CollisionCheckResult& result) const;
  bool checkExternalCollision(CollisionCheckResult& result) const;
  bool checkAll(CollisionCheckResult& result) const;

  visualization_msgs::msg::MarkerArray makeFoxgloveMarkers(const std::string& frame_id, const rclcpp::Time& stamp,
                                                           const std::string& marker_ns = "coal_collision",
                                                           bool clear_previous = true) const;

private:
  enum class QueryMode
  {
    Self,
    External,
    All,
  };

  struct ObjectEntry
  {
    CollisionObjectSpec spec;
    std::shared_ptr<coal::CollisionGeometry> geometry;
    std::unique_ptr<coal::CollisionObject> object;
  };

  bool checkPairs(QueryMode mode, CollisionCheckResult& result) const;

  CollisionSceneOptions options_;
  mutable std::mutex mutex_;
  std::map<std::string, ObjectEntry> objects_;
};

}  // namespace collision
