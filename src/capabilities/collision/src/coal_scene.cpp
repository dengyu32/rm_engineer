#include "collision/coal_scene.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace collision
{
namespace
{
bool positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool finitePose(const Eigen::Isometry3d& pose)
{
  return pose.matrix().allFinite();
}

coal::Transform3s toCoalTransform(const Eigen::Isometry3d& pose)
{
  return coal::Transform3s(pose.linear().cast<coal::Scalar>(), pose.translation().cast<coal::Scalar>());
}

geometry_msgs::msg::Pose toPoseMsg(const Eigen::Isometry3d& pose)
{
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.translation().x();
  msg.position.y = pose.translation().y();
  msg.position.z = pose.translation().z();

  Eigen::Quaterniond quat(pose.linear());
  quat.normalize();
  msg.orientation.x = quat.x();
  msg.orientation.y = quat.y();
  msg.orientation.z = quat.z();
  msg.orientation.w = quat.w();
  return msg;
}

std_msgs::msg::ColorRGBA roleColor(CollisionObjectRole role, bool enabled)
{
  std_msgs::msg::ColorRGBA color;
  if (role == CollisionObjectRole::Self)
  {
    color.r = 0.10F;
    color.g = 0.72F;
    color.b = 0.34F;
    color.a = enabled ? 0.45F : 0.12F;
  }
  else
  {
    color.r = 0.95F;
    color.g = 0.48F;
    color.b = 0.12F;
    color.a = enabled ? 0.45F : 0.12F;
  }
  return color;
}

std::shared_ptr<coal::CollisionGeometry> makeGeometry(const CollisionGeometrySpec& spec, std::string& err)
{
  switch (spec.type)
  {
    case CollisionGeometryType::Box:
      if (!positive(spec.size.x()) || !positive(spec.size.y()) || !positive(spec.size.z()))
      {
        err = "box size must be positive finite";
        return nullptr;
      }
      return std::make_shared<coal::Box>(spec.size.x(), spec.size.y(), spec.size.z());
    case CollisionGeometryType::Sphere:
      if (!positive(spec.radius))
      {
        err = "sphere radius must be positive finite";
        return nullptr;
      }
      return std::make_shared<coal::Sphere>(spec.radius);
    case CollisionGeometryType::Capsule:
      if (!positive(spec.radius) || !positive(spec.length))
      {
        err = "capsule radius and length must be positive finite";
        return nullptr;
      }
      return std::make_shared<coal::Capsule>(spec.radius, spec.length);
    case CollisionGeometryType::Cylinder:
      if (!positive(spec.radius) || !positive(spec.length))
      {
        err = "cylinder radius and length must be positive finite";
        return nullptr;
      }
      return std::make_shared<coal::Cylinder>(spec.radius, spec.length);
  }
  err = "unsupported collision geometry type";
  return nullptr;
}

const char* roleName(CollisionObjectRole role)
{
  return role == CollisionObjectRole::Self ? "self" : "external";
}

std::string makeMessage(const CollisionCheckResult& result)
{
  if (result.collision_free)
  {
    return "collision free";
  }

  std::ostringstream oss;
  oss << "collision detected";
  for (std::size_t i = 0; i < result.pairs.size(); ++i)
  {
    oss << (i == 0 ? ": " : ", ") << result.pairs[i].first_id << " <-> " << result.pairs[i].second_id;
  }
  return oss.str();
}

void fillMarkerScale(const CollisionGeometrySpec& geometry, visualization_msgs::msg::Marker& marker)
{
  switch (geometry.type)
  {
    case CollisionGeometryType::Box:
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.scale.x = geometry.size.x();
      marker.scale.y = geometry.size.y();
      marker.scale.z = geometry.size.z();
      break;
    case CollisionGeometryType::Sphere:
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.scale.x = geometry.radius * 2.0;
      marker.scale.y = geometry.radius * 2.0;
      marker.scale.z = geometry.radius * 2.0;
      break;
    case CollisionGeometryType::Capsule:
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.scale.x = geometry.radius * 2.0;
      marker.scale.y = geometry.radius * 2.0;
      marker.scale.z = geometry.length + geometry.radius * 2.0;
      break;
    case CollisionGeometryType::Cylinder:
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.scale.x = geometry.radius * 2.0;
      marker.scale.y = geometry.radius * 2.0;
      marker.scale.z = geometry.length;
      break;
  }
}

}  // namespace

CollisionGeometrySpec CollisionGeometrySpec::makeBox(double x, double y, double z)
{
  CollisionGeometrySpec spec;
  spec.type = CollisionGeometryType::Box;
  spec.size = Eigen::Vector3d(x, y, z);
  return spec;
}

CollisionGeometrySpec CollisionGeometrySpec::makeSphere(double radius)
{
  CollisionGeometrySpec spec;
  spec.type = CollisionGeometryType::Sphere;
  spec.radius = radius;
  return spec;
}

CollisionGeometrySpec CollisionGeometrySpec::makeCapsule(double radius, double length)
{
  CollisionGeometrySpec spec;
  spec.type = CollisionGeometryType::Capsule;
  spec.radius = radius;
  spec.length = length;
  return spec;
}

CollisionGeometrySpec CollisionGeometrySpec::makeCylinder(double radius, double length)
{
  CollisionGeometrySpec spec;
  spec.type = CollisionGeometryType::Cylinder;
  spec.radius = radius;
  spec.length = length;
  return spec;
}

CollisionScene::CollisionScene(CollisionSceneOptions options) : options_(std::move(options))
{
  options_.max_contacts_per_pair = std::max<std::size_t>(1, options_.max_contacts_per_pair);
}

bool CollisionScene::upsertObject(CollisionObjectSpec spec, std::string& err)
{
  err.clear();
  if (spec.id.empty())
  {
    err = "collision object id is empty";
    return false;
  }
  if (!finitePose(spec.pose))
  {
    err = "collision object pose must be finite";
    return false;
  }

  auto geometry = makeGeometry(spec.geometry, err);
  if (!geometry)
  {
    return false;
  }

  ObjectEntry entry;
  entry.spec = std::move(spec);
  entry.geometry = std::move(geometry);
  entry.object = std::make_unique<coal::CollisionObject>(entry.geometry, toCoalTransform(entry.spec.pose));

  const std::string id = entry.spec.id;
  std::scoped_lock<std::mutex> lock(mutex_);
  objects_[id] = std::move(entry);
  return true;
}

bool CollisionScene::removeObject(const std::string& id)
{
  std::scoped_lock<std::mutex> lock(mutex_);
  return objects_.erase(id) > 0;
}

void CollisionScene::clear()
{
  std::scoped_lock<std::mutex> lock(mutex_);
  objects_.clear();
}

bool CollisionScene::setPose(const std::string& id, const Eigen::Isometry3d& pose)
{
  if (!finitePose(pose))
  {
    return false;
  }

  std::scoped_lock<std::mutex> lock(mutex_);
  auto it = objects_.find(id);
  if (it == objects_.end())
  {
    return false;
  }
  it->second.spec.pose = pose;
  it->second.object->setTransform(toCoalTransform(pose));
  it->second.object->computeAABB();
  return true;
}

bool CollisionScene::setEnabled(const std::string& id, bool enabled)
{
  std::scoped_lock<std::mutex> lock(mutex_);
  auto it = objects_.find(id);
  if (it == objects_.end())
  {
    return false;
  }
  it->second.spec.enabled = enabled;
  return true;
}

bool CollisionScene::contains(const std::string& id) const
{
  std::scoped_lock<std::mutex> lock(mutex_);
  return objects_.find(id) != objects_.end();
}

std::vector<std::string> CollisionScene::ids(CollisionObjectRole role) const
{
  std::vector<std::string> out;
  std::scoped_lock<std::mutex> lock(mutex_);
  for (const auto& [id, entry] : objects_)
  {
    if (entry.spec.role == role)
    {
      out.push_back(id);
    }
  }
  return out;
}

bool CollisionScene::checkSelfCollision(CollisionCheckResult& result) const
{
  return checkPairs(QueryMode::Self, result);
}

bool CollisionScene::checkExternalCollision(CollisionCheckResult& result) const
{
  return checkPairs(QueryMode::External, result);
}

bool CollisionScene::checkAll(CollisionCheckResult& result) const
{
  return checkPairs(QueryMode::All, result);
}

bool CollisionScene::checkPairs(QueryMode mode, CollisionCheckResult& result) const
{
  result = CollisionCheckResult{};

  coal::CollisionRequest request;
  request.num_max_contacts = options_.max_contacts_per_pair;
  request.enable_contact = true;
  request.security_margin = static_cast<coal::Scalar>(options_.security_margin);

  std::scoped_lock<std::mutex> lock(mutex_);
  for (auto lhs = objects_.begin(); lhs != objects_.end(); ++lhs)
  {
    if (!lhs->second.spec.enabled)
    {
      continue;
    }
    for (auto rhs = std::next(lhs); rhs != objects_.end(); ++rhs)
    {
      const bool self_pair =
          lhs->second.spec.role == CollisionObjectRole::Self && rhs->second.spec.role == CollisionObjectRole::Self;
      const bool external_pair = lhs->second.spec.role != rhs->second.spec.role;
      const bool selected = (mode == QueryMode::Self && self_pair) ||
                            (mode == QueryMode::External && external_pair) ||
                            (mode == QueryMode::All && (self_pair || external_pair));
      if (!rhs->second.spec.enabled || !selected)
      {
        continue;
      }

      coal::CollisionResult coal_result;
      coal::collide(lhs->second.object.get(), rhs->second.object.get(), request, coal_result);
      if (!coal_result.isCollision())
      {
        continue;
      }

      for (const auto& contact : coal_result.getContacts())
      {
        CollisionPair pair;
        pair.first_id = lhs->first;
        pair.second_id = rhs->first;
        pair.first_role = lhs->second.spec.role;
        pair.second_role = rhs->second.spec.role;
        pair.penetration_depth = contact.penetration_depth;
        pair.position = contact.pos.cast<double>();
        pair.normal = contact.normal.cast<double>();
        result.pairs.push_back(std::move(pair));
      }

      if (options_.max_pairs > 0 && result.pairs.size() >= options_.max_pairs)
      {
        result.collision_free = false;
        result.message = makeMessage(result);
        return false;
      }
    }
  }

  result.collision_free = result.pairs.empty();
  result.message = makeMessage(result);
  return result.collision_free;
}

visualization_msgs::msg::MarkerArray CollisionScene::makeFoxgloveMarkers(const std::string& frame_id,
                                                                         const rclcpp::Time& stamp,
                                                                         const std::string& marker_ns,
                                                                         bool clear_previous) const
{
  visualization_msgs::msg::MarkerArray marker_array;
  int marker_id = 0;
  if (clear_previous)
  {
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id;
    clear_marker.header.stamp = stamp;
    clear_marker.ns = marker_ns;
    clear_marker.id = marker_id++;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(std::move(clear_marker));
  }

  std::scoped_lock<std::mutex> lock(mutex_);
  for (const auto& [id, entry] : objects_)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = marker_ns + "/" + roleName(entry.spec.role);
    marker.id = marker_id++;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = toPoseMsg(entry.spec.pose);
    marker.color = roleColor(entry.spec.role, entry.spec.enabled);
    marker.text = id;
    fillMarkerScale(entry.spec.geometry, marker);
    marker_array.markers.push_back(std::move(marker));
  }
  return marker_array;
}

}  // namespace collision
