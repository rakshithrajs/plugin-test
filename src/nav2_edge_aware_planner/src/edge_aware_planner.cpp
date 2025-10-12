#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_util/node_utils.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "pluginlib/class_loader.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std;

namespace nav2_edge_aware_planner
{

class EdgeAwarePlanner : public nav2_core::GlobalPlanner
{
public:
  EdgeAwarePlanner() = default;
  ~EdgeAwarePlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    string name,
    shared_ptr<tf2_ros::Buffer> tf,
    shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    node_ = parent;
    name_ = move(name);
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    global_frame_ = costmap_ros_->getGlobalFrameID();

    auto node = node_.lock();

    node->declare_parameter(name_ + ".interpolation_resolution", 0.1);
    node->declare_parameter(name_ + ".delegate_plugin", string("nav2_navfn_planner::NavfnPlanner"));

    node->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
    node->get_parameter(name_ + ".delegate_plugin", delegate_plugin_type_);

    RCLCPP_INFO(node->get_logger(), "[%s] configured. delegate=%s, interp_res=%.3f",
                name_.c_str(), delegate_plugin_type_.c_str(), interpolation_resolution_);

    try {
      planner_loader_ = make_unique<pluginlib::ClassLoader<nav2_core::GlobalPlanner>>(
        "nav2_core", "nav2_core::GlobalPlanner");
      delegate_planner_ = planner_loader_->createSharedInstance(delegate_plugin_type_);
      delegate_planner_->configure(parent, name_ + ".delegate", tf_, costmap_ros_);
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_WARN(node->get_logger(), "Could not load delegate planner '%s': %s",
                  delegate_plugin_type_.c_str(), ex.what());
      delegate_planner_ = nullptr;
    }
  }

  void activate() override
  {
    auto node = node_.lock();
    if (delegate_planner_) {
      delegate_planner_->activate();
    }
    RCLCPP_INFO(node->get_logger(), "[%s] activated", name_.c_str());
  }

  void deactivate() override
  {
    auto node = node_.lock();
    if (delegate_planner_) {
      delegate_planner_->deactivate();
    }
    RCLCPP_INFO(node->get_logger(), "[%s] deactivated", name_.c_str());
  }

  void cleanup() override
  {
    auto node = node_.lock();
    if (delegate_planner_) {
      delegate_planner_->cleanup();
      delegate_planner_.reset();
    }
    planner_loader_.reset();
    RCLCPP_INFO(node->get_logger(), "[%s] cleaned up", name_.c_str());
  }


  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    nav_msgs::msg::Path global_path;
    auto node = node_.lock();
    global_path.header.stamp = node->now();
    global_path.header.frame_id = global_frame_;

    if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
      RCLCPP_ERROR(node->get_logger(), "Start and goal must be in global frame '%s'", global_frame_.c_str());
      return global_path;
    }

    unsigned int mx = 0, my = 0;
    bool goal_in_map = costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my);

    if (goal_in_map) {
      RCLCPP_INFO(node->get_logger(), "Goal is inside the costmap; using delegate planner only");
      if (delegate_planner_) {
        RCLCPP_INFO(node->get_logger(), "Using delegate planner '%s'", delegate_plugin_type_.c_str());
        auto dp = delegate_planner_->createPlan(start, goal);
        stampPath(dp);
        return dp;
      } else {
        RCLCPP_WARN(node->get_logger(), "No delegate planner available; returning straight-line plan");
        auto s = createStraightLinePath(start, goal);
        stampPath(s);
        return s;
      }
    }
    RCLCPP_INFO(node->get_logger(), "Goal is outside the costmap; using edge-aware planning");
    geometry_msgs::msg::PoseStamped edge_pose;
    bool ok = computeEdgeIntersection(start, goal, edge_pose);
    if (!ok) {
      RCLCPP_WARN(node->get_logger(), "Couldn't compute edge intersection; returning straight-line plan");
      return createStraightLinePath(start, goal);
    }

    nav_msgs::msg::Path path_to_edge;
    if (delegate_planner_) {
      path_to_edge = delegate_planner_->createPlan(start, edge_pose);
    } else {
      path_to_edge = createStraightLinePath(start, edge_pose);
    }
    stampPath(path_to_edge);
    nav_msgs::msg::Path tail = createStraightLinePath(edge_pose, goal);
    stampPath(tail);

    global_path = path_to_edge;
    for (size_t i = 0; i < tail.poses.size(); ++i) {
      if (!global_path.poses.empty() &&
          i == 0 &&
          posesEqual(global_path.poses.back(), tail.poses.front())) {
        continue;
      }
      global_path.poses.push_back(tail.poses[i]);
    }
    stampPath(global_path);

    geometry_msgs::msg::PoseStamped goal_pose = goal;
    {
      auto n = node_.lock();
      if(n){
        goal_pose.header.stamp = n->now();
      }
      goal_pose.header.frame_id = global_frame_;
    }
    global_path.poses.push_back(goal_pose);
    return global_path;
  }

private:
  void stampPath(nav_msgs::msg::Path & path)
  {
    auto n = node_.lock();
    if (!n) return;
    rclcpp::Time now = n->now();

    path.header.stamp = now;
    path.header.frame_id = global_frame_;

    for (auto & ps : path.poses) {
      ps.header.stamp = now;
      ps.header.frame_id = global_frame_;
    }
  }

  bool posesEqual(const geometry_msgs::msg::PoseStamped &a, const geometry_msgs::msg::PoseStamped &b)
  {
    constexpr double EPS = 1e-4;
    return (fabs(a.pose.position.x - b.pose.position.x) < EPS &&
            fabs(a.pose.position.y - b.pose.position.y) < EPS);
  }

  bool computeEdgeIntersection(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal,
      geometry_msgs::msg::PoseStamped & intersection_out)
  {
      double ox = costmap_->getOriginX();
      double oy = costmap_->getOriginY();
      double size_x = costmap_->getSizeInMetersX();
      double size_y = costmap_->getSizeInMetersY();
      double xmin = ox;
      double ymin = oy;
      double xmax = ox + size_x;
      double ymax = oy + size_y;

      double sx = start.pose.position.x;
      double sy = start.pose.position.y;
      double gx = goal.pose.position.x;
      double gy = goal.pose.position.y;
      double dx = gx - sx;
      double dy = gy - sy;

      struct Candidate { double t; double x; double y; bool valid; };
      std::vector<Candidate> cand; 
      cand.reserve(4);

      const double EPS = 1e-9;

      if (fabs(dx) > EPS) {
          double t1 = (xmin - sx) / dx;
          double y1 = sy + t1 * dy;
          if (t1 >= 0.0 && t1 <= 1.0 && y1 >= ymin - EPS && y1 <= ymax + EPS)
              cand.push_back({t1, xmin, y1, true});

          double t2 = (xmax - sx) / dx;
          double y2 = sy + t2 * dy;
          if (t2 >= 0.0 && t2 <= 1.0 && y2 >= ymin - EPS && y2 <= ymax + EPS)
              cand.push_back({t2, xmax, y2, true});
      }

      if (fabs(dy) > EPS) {
          double t3 = (ymin - sy) / dy;
          double x3 = sx + t3 * dx;
          if (t3 >= 0.0 && t3 <= 1.0 && x3 >= xmin - EPS && x3 <= xmax + EPS)
              cand.push_back({t3, x3, ymin, true});

          double t4 = (ymax - sy) / dy;
          double x4 = sx + t4 * dx;
          if (t4 >= 0.0 && t4 <= 1.0 && x4 >= xmin - EPS && x4 <= xmax + EPS)
              cand.push_back({t4, x4, ymax, true});
      }

      if (cand.empty()) return false;

      Candidate best = {std::numeric_limits<double>::infinity(), 0.0, 0.0, false};
      for (auto &c : cand) {
          if (!c.valid) continue;

          unsigned int mx, my;
          if (!costmap_->worldToMap(c.x, c.y, mx, my)) continue; // outside map
          unsigned char cost = costmap_->getCost(mx, my);
          if (cost == nav2_costmap_2d::NO_INFORMATION) continue; // skip unknown areas

          if (c.t >= 0.0 && c.t <= 1.0 && c.t < best.t) 
              best = c;
      }

      if (!best.valid) return false;

      intersection_out = start;
      intersection_out.pose.position.x = best.x;
      intersection_out.pose.position.y = best.y;
      intersection_out.pose.position.z = 0.0;

      double yaw = atan2(gy - best.y, gx - best.x);
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      intersection_out.pose.orientation = tf2::toMsg(q);

      intersection_out.header.frame_id = global_frame_;
      intersection_out.header.stamp = node_.lock()->now();

      return true;
  }


  nav_msgs::msg::Path createStraightLinePath(
    const geometry_msgs::msg::PoseStamped & from,
    const geometry_msgs::msg::PoseStamped & to)
  {
    nav_msgs::msg::Path path;
    auto node = node_.lock();
    path.header.frame_id = global_frame_;
    path.header.stamp = node->now();

    double dx = to.pose.position.x - from.pose.position.x;
    double dy = to.pose.position.y - from.pose.position.y;
    double dist = hypot(dx, dy);
    if (dist < 1e-6) {
      path.poses.push_back(from);
      return path;
    }
    int steps = max(1, static_cast<int>(ceil(dist / interpolation_resolution_)));
    for (int i = 0; i <= steps; ++i) {
      double alpha = static_cast<double>(i) / static_cast<double>(steps);
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = global_frame_;
      p.header.stamp = node->now();
      p.pose.position.x = from.pose.position.x + alpha * dx;
      p.pose.position.y = from.pose.position.y + alpha * dy;
      p.pose.position.z = 0.0;
      double yaw = atan2(dy, dx);
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      p.pose.orientation = tf2::toMsg(q);
      path.poses.push_back(p);
    }
    return path;
  }

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  string name_;
  shared_ptr<tf2_ros::Buffer> tf_;
  shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  string global_frame_;
  double interpolation_resolution_{0.1};
  string delegate_plugin_type_{"nav2_navfn_planner::NavfnPlanner"};

  unique_ptr<pluginlib::ClassLoader<nav2_core::GlobalPlanner>> planner_loader_;
  nav2_core::GlobalPlanner::Ptr delegate_planner_;
};

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_edge_aware_planner::EdgeAwarePlanner, nav2_core::GlobalPlanner)
