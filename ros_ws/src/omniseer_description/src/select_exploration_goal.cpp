#include <algorithm>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <limits>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav2_msgs/srv/get_costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <utility>
#include <vector>

#include "omniseer/frontier.hpp"
#include "omniseer/grid_utils.hpp"
#include "omniseer/nav2_adapters.hpp"
#include "omniseer/ray_cast.hpp"

namespace omniseer
{

  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using GoalHandle        = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
  using GoalHandleFuture  = std::shared_future<typename GoalHandle::SharedPtr>;
  using ResultFuture      = std::shared_future<typename GoalHandle::WrappedResult>;

  class SelectExplorationGoal : public BT::StatefulActionNode
  {
  public:
    // Constructor
    SelectExplorationGoal(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
    {
      auto bb = config.blackboard;
      if (!bb->get<rclcpp::Node::SharedPtr>("node", node_))
      {
        throw std::runtime_error("Blackboard missing 'node' (rclcpp::Node::SharedPtr)");
      }

      tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    static BT::PortsList providedPorts()
    {
      return {BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal"),
              BT::InputPort<std::string>("global_frame"),
              BT::InputPort<std::string>("robot_frame"),
              BT::InputPort<std::string>("costmap_service"),
              BT::InputPort<std::string>("planner_action"),
              BT::InputPort<int>("top_k"),
              BT::InputPort<double>("ig_max_range_m"),
              BT::InputPort<int>("ig_num_rays"),
              BT::InputPort<double>("ig_fov_rad"),
              BT::InputPort<bool>("use_nav2_plan_cost"),
              BT::InputPort<int>("costmap_timeout_ms"),
              BT::InputPort<int>("planner_timeout_ms"),
              BT::InputPort<bool>("debug_enabled")};
    }

  protected:
    BT::NodeStatus onStart() override
    {
      reset();

      global_frame_    = getOr("global_frame", std::string("map"));
      robot_frame_     = getOr("robot_frame", std::string("base_link"));
      costmap_service_ = getOr("costmap_service", std::string("/global_costmap/get_costmap"));
      planner_action_  = getOr("planner_action", std::string("compute_path_to_pose"));
      top_k_           = std::max(1, getOr("top_k", 8));
      costmap_timeout_ = std::chrono::milliseconds(getOr("costmap_timeout_ms", 200));
      planner_timeout_ = std::chrono::milliseconds(getOr("planner_timeout_ms", 800));
      raycast_params_.max_ray_len = getOr("ig_max_range_m", 6.0);
      raycast_params_.num_rays    = getOr("ig_num_rays", 48);
      raycast_params_.fov_rad     = getOr("ig_fov_rad", 2.0 * M_PI);
      raycast_params_.angle0      = 0.0;
      debug_enabled_              = getOr("debug_enabled", false);
      if (debug_enabled_)
      {
        ensureDebugPublishers();
      }

      const bool use_plan_cost = getOr("use_nav2_plan_cost", true);
      if (!use_plan_cost && !warned_disable_plan_cost_)
      {
        RCLCPP_WARN(
            node_->get_logger(),
            "[SelectExplorationGoal] 'use_nav2_plan_cost' is ignored; planner is always used");
        warned_disable_plan_cost_ = true;
      }

      if (!costmap_client_ || costmap_client_->get_service_name() != costmap_service_)
      {
        costmap_client_ = node_->create_client<nav2_msgs::srv::GetCostmap>(costmap_service_);
      }
      if (!planner_client_ || planner_client_action_name_ != planner_action_)
      {
        planner_client_ = rclcpp_action::create_client<ComputePathToPose>(node_, planner_action_);
        planner_client_action_name_ = planner_action_;
      }

      costmap_deadline_ = std::chrono::steady_clock::now() + costmap_timeout_;
      stage_            = Stage::WaitingForService;
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
      while (stage_ != Stage::Finished)
      {
        const auto now = std::chrono::steady_clock::now();

        switch (stage_)
        {
        case Stage::WaitingForService:
        {
          if (costmap_client_->wait_for_service(std::chrono::seconds(0)))
          {
            auto req            = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
            auto pending        = costmap_client_->async_send_request(req);
            costmap_future_     = pending.share();
            costmap_request_id_ = pending.request_id;
            costmap_deadline_ =
                std::chrono::steady_clock::now() + costmap_timeout_; // response deadline
            stage_ = Stage::WaitingForCostmap;
            continue;
          }
          if (now > costmap_deadline_)
          {
            RCLCPP_WARN(node_->get_logger(),
                        "[SelectExplorationGoal] Costmap service '%s' not available (timeout)",
                        costmap_service_.c_str());
            return finish(BT::NodeStatus::FAILURE);
          }
          return BT::NodeStatus::RUNNING;
        }

        case Stage::WaitingForCostmap:
        {
          if (costmap_future_.valid() &&
              costmap_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
          {
            auto resp       = costmap_future_.get();
            costmap_future_ = {};
            if (costmap_request_id_)
            {
              costmap_client_->remove_pending_request(*costmap_request_id_);
              costmap_request_id_.reset();
            }
            if (!resp)
            {
              RCLCPP_WARN(node_->get_logger(), "[SelectExplorationGoal] Costmap response null");
              return finish(BT::NodeStatus::FAILURE);
            }
            costmap_msg_ = resp->map;
            stage_       = Stage::PreparingGoals;
            continue;
          }
          if (now > costmap_deadline_)
          {
            RCLCPP_WARN(node_->get_logger(),
                        "[SelectExplorationGoal] Costmap service request timeout");
            if (costmap_request_id_)
            {
              costmap_client_->remove_pending_request(*costmap_request_id_);
              costmap_request_id_.reset();
            }
            costmap_future_ = {};
            return finish(BT::NodeStatus::FAILURE);
          }
          return BT::NodeStatus::RUNNING;
        }

        case Stage::PreparingGoals:
        {
          if (!prepareGoals())
          {
            return finish(BT::NodeStatus::FAILURE);
          }

          planner_index_    = 0;
          planner_deadline_ = std::chrono::steady_clock::now() + planner_timeout_;
          stage_            = Stage::PlanningDispatch;
          continue;
        }

        case Stage::PlanningDispatch:
        {
          if (planner_index_ >= goals_.size())
          {
            stage_ = Stage::Finalizing;
            continue;
          }

          if (!planner_client_->wait_for_action_server(std::chrono::seconds(0)))
          {
            if (now > planner_deadline_)
            {
              RCLCPP_WARN(node_->get_logger(),
                          "[SelectExplorationGoal] Planner action '%s' not available (timeout)",
                          planner_action_.c_str());
              return finish(BT::NodeStatus::FAILURE);
            }
            return BT::NodeStatus::RUNNING;
          }

          const auto& goal = goals_[planner_index_];

          ComputePathToPose::Goal goal_msg;
          goal_msg.start.header.frame_id    = global_frame_;
          goal_msg.goal.header.frame_id     = global_frame_;
          goal_msg.start.pose.position.x    = robot_.x;
          goal_msg.start.pose.position.y    = robot_.y;
          goal_msg.start.pose.orientation.w = 1.0;
          goal_msg.goal.pose.position.x     = goal.pose.x;
          goal_msg.goal.pose.position.y     = goal.pose.y;
          goal_msg.goal.pose.orientation.w  = 1.0;

          planner_goal_future_ = planner_client_->async_send_goal(goal_msg);
          planner_deadline_    = std::chrono::steady_clock::now() + planner_timeout_;
          stage_               = Stage::PlanningWaitingHandle;
          continue;
        }

        case Stage::PlanningWaitingHandle:
        {
          if (planner_goal_future_.valid() &&
              planner_goal_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
          {
            planner_goal_handle_ = planner_goal_future_.get();
            if (!planner_goal_handle_)
            {
              RCLCPP_DEBUG(node_->get_logger(),
                           "[SelectExplorationGoal] Planner goal handle null (goal %zu)",
                           planner_index_);
              advanceToNextGoal();
              continue;
            }

            planner_result_future_ = planner_client_->async_get_result(planner_goal_handle_);
            planner_deadline_      = std::chrono::steady_clock::now() + planner_timeout_;
            stage_                 = Stage::PlanningWaitingResult;
            continue;
          }

          if (std::chrono::steady_clock::now() > planner_deadline_)
          {
            RCLCPP_WARN(node_->get_logger(),
                        "[SelectExplorationGoal] Planner goal send timeout (goal %zu)",
                        planner_index_);
            advanceToNextGoal();
            continue;
          }

          return BT::NodeStatus::RUNNING;
        }

        case Stage::PlanningWaitingResult:
        {
          if (planner_result_future_.valid() &&
              planner_result_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
          {
            const auto wrapped = planner_result_future_.get();
            if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result &&
                !wrapped.result->path.poses.empty())
            {
              const double length              = pathLength(wrapped.result->path);
              goals_[planner_index_].path_cost = length;
            }
            else
            {
              goals_[planner_index_].path_cost = std::numeric_limits<double>::infinity();
              RCLCPP_DEBUG(node_->get_logger(),
                           "[SelectExplorationGoal] Planner failed with code %d (goal %zu)",
                           static_cast<int>(wrapped.code), planner_index_);
            }

            advanceToNextGoal();
            continue;
          }

          if (std::chrono::steady_clock::now() > planner_deadline_)
          {
            RCLCPP_WARN(node_->get_logger(),
                        "[SelectExplorationGoal] Planner result timeout (goal %zu)",
                        planner_index_);
            if (planner_goal_handle_)
            {
              planner_client_->async_cancel_goal(planner_goal_handle_);
            }
            goals_[planner_index_].path_cost = std::numeric_limits<double>::infinity();
            advanceToNextGoal();
            continue;
          }

          return BT::NodeStatus::RUNNING;
        }

        case Stage::Finalizing:
        {
          const auto status = emitBestGoal();
          stage_            = Stage::Finished;
          final_status_     = status;
          return status;
        }

        case Stage::Idle:
        case Stage::Finished:
          break;
        }
      }

      return final_status_.value_or(BT::NodeStatus::FAILURE);
    }

    void onHalted() override
    {
      if (planner_goal_handle_)
      {
        planner_client_->async_cancel_goal(planner_goal_handle_);
      }
      reset();
    }

  private:
    enum class Stage
    {
      Idle,
      WaitingForService,
      WaitingForCostmap,
      PreparingGoals,
      PlanningDispatch,
      PlanningWaitingHandle,
      PlanningWaitingResult,
      Finalizing,
      Finished
    };

    template <typename T> T getOr(const char* port_name, const T& def)
    {
      T value{};
      return getInput<T>(port_name, value) ? value : def;
    }

    void reset()
    {
      stage_ = Stage::Idle;
      final_status_.reset();
      costmap_future_ = {};
      costmap_request_id_.reset();
      planner_goal_future_   = GoalHandleFuture{};
      planner_result_future_ = ResultFuture{};
      planner_goal_handle_.reset();
      artifacts_ = Artifacts{};
      ig_scratch_.clear();
      goals_.clear();
      planner_index_ = 0;
      costmap_msg_   = nav2_msgs::msg::Costmap{};
    }

    bool prepareGoals()
    {
      try
      {
        grid_ = costmapToGrid(costmap_msg_);
      }
      catch (const std::exception& e)
      {
        RCLCPP_WARN(node_->get_logger(), "[SelectExplorationGoal] Costmap conversion failed: %s",
                    e.what());
        return false;
      }

      auto robot_pose =
          lookupRobotPose(*tf_buffer_, global_frame_, robot_frame_, node_->get_logger());
      if (!robot_pose)
      {
        return false;
      }
      robot_ = *robot_pose;

      params_                         = Params{};
      params_.connectivity            = Conn::Eight;
      params_.min_unknown_neighbors   = 1;
      params_.min_component_size      = 15;
      params_.min_goal_spacing_cells  = 10;
      params_.max_goals_per_component = 30;
      params_.max_total_goals         = 64;
      params_.top_k_goals             = top_k_;
      params_.w_information           = 1.0;
      params_.w_distance_cost         = 1.0;
      params_.ray_cast_params         = raycast_params_;

      const int    w = grid_.width;
      const int    h = grid_.height;
      const size_t n = static_cast<size_t>(w) * static_cast<size_t>(h);

      artifacts_        = Artifacts{};
      artifacts_.width  = w;
      artifacts_.height = h;
      artifacts_.frontier_mask.assign(n, 0u);
      artifacts_.component_labels.assign(n, -1);
      artifacts_.candidate_goals.clear();
      artifacts_.ranked_goals.clear();
      artifacts_.selected_goal.reset();

      compute_frontier_mask(grid_, params_, artifacts_.frontier_mask.data(), n);
      auto components = label_components(grid_, params_, artifacts_.frontier_mask.data(), n,
                                         artifacts_.component_labels.data());

      Callbacks cb{};
      cb.information_gain = [&, this](const Pose2D& q) -> double
      { return static_cast<double>(count_unknown_ig(grid_, params_, q, &ig_scratch_)); };

      ig_scratch_.assign(n, 0u);

      auto candidates = select_component_goals(grid_, params_, components);
      if (candidates.empty())
      {
        publishDebugFrontiersAndCandidates();
        RCLCPP_INFO(node_->get_logger(), "[SelectExplorationGoal] No frontier candidates found");
        return false;
      }

      compute_goal_information(params_, cb, candidates);
      select_top_k_by_information(candidates, top_k_);
      for (auto& goal : candidates)
      {
        goal.path_cost = std::numeric_limits<double>::infinity();
        goal.score     = 0.0;
      }
      goals_                     = std::move(candidates);
      artifacts_.candidate_goals = goals_;
      publishDebugFrontiersAndCandidates();

      return true;
    }

    void advanceToNextGoal()
    {
      planner_goal_future_   = GoalHandleFuture{};
      planner_result_future_ = ResultFuture{};
      planner_goal_handle_.reset();
      ++planner_index_;
      planner_deadline_ = std::chrono::steady_clock::now() + planner_timeout_;
      stage_            = Stage::PlanningDispatch;
    }

    void ensureDebugPublishers()
    {
      if (!debug_frontier_pub_)
      {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        debug_frontier_pub_ =
            node_->create_publisher<nav_msgs::msg::OccupancyGrid>("omniseer/debug/frontiers", qos);
      }
      if (!debug_candidates_pub_)
      {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        debug_candidates_pub_ =
            node_->create_publisher<geometry_msgs::msg::PoseArray>("omniseer/debug/candidates",
                                                                   qos);
      }
      if (!debug_selected_goal_pub_)
      {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        debug_selected_goal_pub_ =
            node_->create_publisher<geometry_msgs::msg::PoseStamped>("omniseer/debug/selected_goal",
                                                                     qos);
      }
    }

    void publishDebugFrontiersAndCandidates()
    {
      if (!debug_enabled_ || !debug_frontier_pub_ || !debug_candidates_pub_)
        return;
      if (artifacts_.frontier_mask.empty())
        return;

      nav_msgs::msg::OccupancyGrid frontier_msg;
      frontier_msg.header.frame_id           = global_frame_;
      frontier_msg.header.stamp              = node_->now();
      frontier_msg.info.resolution           = grid_.resolution;
      frontier_msg.info.width                = static_cast<uint32_t>(grid_.width);
      frontier_msg.info.height               = static_cast<uint32_t>(grid_.height);
      frontier_msg.info.origin.position.x    = grid_.origin_x;
      frontier_msg.info.origin.position.y    = grid_.origin_y;
      frontier_msg.info.origin.orientation.w = 1.0;
      frontier_msg.data.resize(artifacts_.frontier_mask.size());
      std::transform(artifacts_.frontier_mask.begin(), artifacts_.frontier_mask.end(),
                     frontier_msg.data.begin(),
                     [](std::uint8_t v) -> int8_t { return v ? static_cast<int8_t>(100) : 0; });
      debug_frontier_pub_->publish(std::move(frontier_msg));

      geometry_msgs::msg::PoseArray candidate_msg;
      candidate_msg.header.frame_id = global_frame_;
      candidate_msg.header.stamp    = node_->now();
      candidate_msg.poses.reserve(artifacts_.candidate_goals.size());
      for (const auto& goal : artifacts_.candidate_goals)
      {
        geometry_msgs::msg::Pose pose;
        pose.position.x    = goal.pose.x;
        pose.position.y    = goal.pose.y;
        pose.position.z    = 0.0;
        pose.orientation.w = 1.0;
        candidate_msg.poses.push_back(pose);
      }
      debug_candidates_pub_->publish(std::move(candidate_msg));
    }

    void publishDebugSelectedGoal(const geometry_msgs::msg::PoseStamped& goal_msg)
    {
      if (!debug_enabled_ || !debug_selected_goal_pub_)
        return;
      debug_selected_goal_pub_->publish(goal_msg);
    }

    BT::NodeStatus emitBestGoal()
    {
      score_goals_with_precomputed_cost(params_, goals_);

      if (goals_.empty())
      {
        artifacts_.ranked_goals.clear();
        artifacts_.selected_goal.reset();
        RCLCPP_INFO(node_->get_logger(), "[SelectExplorationGoal] No reachable frontier goals");
        return BT::NodeStatus::FAILURE;
      }

      artifacts_.ranked_goals  = goals_;
      artifacts_.selected_goal = goals_.front();

      const auto& best_goal = goals_.front();

      geometry_msgs::msg::PoseStamped out;
      out.header.frame_id    = global_frame_;
      out.header.stamp       = node_->now();
      out.pose.position.x    = best_goal.pose.x;
      out.pose.position.y    = best_goal.pose.y;
      out.pose.orientation.w = 1.0;

      setOutput("goal", out);
      RCLCPP_DEBUG(
          node_->get_logger(),
          "[SelectExplorationGoal] Goal (%.2f, %.2f) score=%.3f comp=%d cost=%.2f info=%.2f",
          best_goal.pose.x, best_goal.pose.y, best_goal.score, best_goal.component_id,
          best_goal.path_cost, best_goal.info_gain);

      publishDebugSelectedGoal(out);

      return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus finish(BT::NodeStatus status)
    {
      final_status_ = status;
      stage_        = Stage::Finished;
      return status;
    }

    Stage                                       stage_{Stage::Idle};
    std::optional<BT::NodeStatus>               final_status_;
    rclcpp::Node::SharedPtr                     node_;
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string global_frame_;
    std::string robot_frame_;
    std::string costmap_service_;
    std::string planner_action_;
    std::string planner_client_action_name_;
    int         top_k_{8};
    bool        warned_disable_plan_cost_{false};
    bool        debug_enabled_{false};

    std::chrono::milliseconds             costmap_timeout_{200};
    std::chrono::milliseconds             planner_timeout_{800};
    std::chrono::steady_clock::time_point costmap_deadline_;
    std::chrono::steady_clock::time_point planner_deadline_;

    rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
    rclcpp_action::Client<ComputePathToPose>::SharedPtr   planner_client_;

    nav2_msgs::msg::Costmap                                  costmap_msg_;
    rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedFuture costmap_future_;
    std::optional<int64_t>                                   costmap_request_id_;
    GoalHandleFuture                                         planner_goal_future_;
    ResultFuture                                             planner_result_future_;
    GoalHandle::SharedPtr                                    planner_goal_handle_;

    GridU8                    grid_;
    Pose2D                    robot_;
    Params                    params_;
    RaycastParams             raycast_params_;
    Artifacts                 artifacts_;
    std::vector<std::uint8_t> ig_scratch_;
    std::vector<FrontierGoal> goals_;
    std::size_t               planner_index_{0};

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr    debug_frontier_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr   debug_candidates_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr debug_selected_goal_pub_;
  };

} // namespace omniseer

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<omniseer::SelectExplorationGoal>("SelectExplorationGoal");
}
