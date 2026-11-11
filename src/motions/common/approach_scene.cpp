#include "demo_1f/motions/common/cup_scene.hpp"
#include <moveit/collision_detection/collision_matrix.hpp>

namespace demo_1f {
moveit_msgs::msg::PlanningScene make_approach_scene(const Config & robot,
    const ApproachConfig & config,
    const geometry_msgs::msg::Pose & body,
    double uncertainty) {
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;
  moveit_msgs::msg::CollisionObject cup;
  cup.id = "demo_place_cup"; // Keep the existing object ID so updates replace the previous cup.
  cup.header.frame_id = robot.planning_frame;
  cup.operation = cup.ADD;
  shape_msgs::msg::SolidPrimitive cylinder;
  cylinder.type = cylinder.CYLINDER;
  // Explicit test assumption: BODY Z is the cup middle. Keep lateral uncertainty.
  cylinder.dimensions = {
      config.cup_height, config.inner_radius + config.wall_thickness + uncertainty};
  cup.primitives.push_back(cylinder);
  cup.primitive_poses.push_back(body);
  scene.world.collision_objects.push_back(cup);
  // Restore normal cup/finger collision checks after an unfinished pick setup.
  scene.allowed_collision_matrix.entry_names = {cup.id};
  for (const std::string side : {"l", "r"}) {
    for (const std::string finger : {"r1", "r2", "l1", "l2"}) {
      scene.allowed_collision_matrix.entry_names.push_back(
          "gripper_" + side + "_rh_p12_rn_" + finger);
    }
  }
  const size_t entries = scene.allowed_collision_matrix.entry_names.size();
  scene.allowed_collision_matrix.entry_values.resize(entries);
  for (auto & row : scene.allowed_collision_matrix.entry_values) {
    row.enabled.resize(entries, false);
  }
  // Remove only the old demo plane; retain all externally registered obstacles/attachments.
  moveit_msgs::msg::CollisionObject old_table;
  old_table.id = "demo_place_table";
  old_table.header.frame_id = robot.planning_frame;
  old_table.operation = old_table.REMOVE;
  scene.world.collision_objects.push_back(old_table);
  return scene;
}

moveit_msgs::msg::PlanningScene make_cup_release_scene(const Config & robot) {
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;
  // The hand is about to open: detach the held cup and delete it; its new place is unknown.
  moveit_msgs::msg::AttachedCollisionObject held;
  held.object.id = "demo_place_cup";
  held.object.operation = held.object.REMOVE;
  scene.robot_state.attached_collision_objects.push_back(held);
  moveit_msgs::msg::CollisionObject cup;
  cup.id = held.object.id;
  cup.header.frame_id = robot.planning_frame;
  cup.operation = cup.REMOVE;
  scene.world.collision_objects.push_back(cup);
  return scene;
}

namespace {
std::vector<std::string> hand_links(const std::string & arm, bool with_base) {
  const std::string prefix = arm == "left" ? "gripper_l_" : "gripper_r_";
  std::vector<std::string> links = {prefix + "rh_p12_rn_r1",
      prefix + "rh_p12_rn_r2",
      prefix + "rh_p12_rn_l1",
      prefix + "rh_p12_rn_l2"};
  if (with_base) {
    links.push_back(prefix + "rh_p12_rn_base");
  }
  return links;
}
} // namespace

moveit_msgs::msg::PlanningScene make_cup_contact_scene(
    const Config & robot, const std::string & receiver) {
  (void)robot;
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;
  auto & acm = scene.allowed_collision_matrix;
  acm.entry_names = {"demo_place_cup"};
  // The cup model is wider than the real cup, so a deep grasp touches it with the palm too.
  for (const auto & link : hand_links(receiver, true)) {
    acm.entry_names.push_back(link);
  }
  acm.entry_values.resize(acm.entry_names.size());
  for (size_t i = 0; i < acm.entry_names.size(); ++i) {
    auto & row = acm.entry_values[i].enabled;
    row.resize(acm.entry_names.size());
    for (size_t j = 0; j < row.size(); ++j) {
      row[j] = (i == 0) != (j == 0); // Cup-hand pairs only.
    }
  }
  return scene;
}

moveit_msgs::msg::PlanningScene make_cup_detach_scene(
    const Config & robot, const std::string & giver) {
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;
  moveit_msgs::msg::AttachedCollisionObject held;
  held.link_name = robot.arms.at(giver).tcp;
  held.object.id = "demo_place_cup";
  held.object.operation = held.object.REMOVE; // Detach only; the cup returns to the world.
  scene.robot_state.attached_collision_objects.push_back(held);
  return scene;
}

moveit_msgs::msg::PlanningScene make_cup_attach_scene(
    const Config & robot, const std::string & receiver, const std::string & giver) {
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;
  moveit_msgs::msg::AttachedCollisionObject held;
  held.link_name = robot.arms.at(receiver).tcp;
  held.object.id = "demo_place_cup";
  held.object.header.frame_id = robot.planning_frame;
  held.object.operation = held.object.ADD; // No geometry: MoveIt moves the world cup to the hand.
  held.touch_links = hand_links(receiver, true);
  scene.robot_state.attached_collision_objects.push_back(held);
  // Giver fingers and palm still touch the cup until the giver opens.
  auto & acm = scene.allowed_collision_matrix;
  acm.entry_names = {held.object.id};
  for (const auto & link : hand_links(giver, true)) {
    acm.entry_names.push_back(link);
  }
  acm.entry_values.resize(acm.entry_names.size());
  for (size_t i = 0; i < acm.entry_names.size(); ++i) {
    auto & row = acm.entry_values[i].enabled;
    row.resize(acm.entry_names.size());
    for (size_t j = 0; j < row.size(); ++j) {
      row[j] = (i == 0) != (j == 0); // Cup-giver pairs only.
    }
  }
  return scene;
}

ApproachScene::ApproachScene(rclcpp::Node & node) {
  query_client_ = node.create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  client_ = node.create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
}

bool ApproachScene::ready() const {
  return client_->service_is_ready() && query_client_->service_is_ready() && !query_.valid() &&
         !future_.valid() && !timed_out_;
}

bool ApproachScene::apply(const moveit_msgs::msg::PlanningScene & scene) {
  if (!ready()) {
    return false;
  }
  pending_scene_ = scene;
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components =
      moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
      moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
      moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
  query_ = query_client_->async_send_request(request).future.share();
  started_ = Clock::now();
  return true;
}

ExecutionResult ApproachScene::poll() {
  if (timed_out_) {
    return {ExecutionState::StopUnconfirmed,
        "SCENE_UNCONFIRMED",
        "Scene update timed out; restart only after inspecting the planning scene"};
  }
  if (query_.valid()) {
    if (query_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      if (elapsed(started_) > 5) {
        timed_out_ = true;
        return poll();
      }
      return {ExecutionState::Running, {}, {}};
    }
    try {
      const auto existing = query_.get()->scene;
      query_ = {};
      for (const auto & object : pending_scene_.world.collision_objects) {
        for (const auto & attached : existing.robot_state.attached_collision_objects) {
          if (object.operation == object.ADD && object.id == attached.object.id) {
            return {
                ExecutionState::Failed, "OBJECT_ALREADY_HELD", "Cup is attached to a robot link"};
          }
        }
      }
      if (!pending_scene_.allowed_collision_matrix.entry_names.empty()) {
        collision_detection::AllowedCollisionMatrix merged(existing.allowed_collision_matrix);
        const auto requested = pending_scene_.allowed_collision_matrix;
        for (size_t i = 0; i < requested.entry_names.size(); ++i) {
          for (size_t j = i + 1; j < requested.entry_names.size(); ++j) {
            // Only the requested cup/finger pairs change; retain all external entries.
            if (requested.entry_names[i] == "demo_place_cup") {
              merged.setEntry(requested.entry_names[i],
                  requested.entry_names[j],
                  requested.entry_values.at(i).enabled.at(j));
            }
          }
        }
        merged.getMessage(pending_scene_.allowed_collision_matrix);
      }
      auto & objects = pending_scene_.world.collision_objects;
      const auto held = [&](const std::string & id) {
        return std::any_of(existing.robot_state.attached_collision_objects.begin(),
            existing.robot_state.attached_collision_objects.end(),
            [&](const auto & old) { return old.object.id == id; });
      };
      const auto detaching = [&](const std::string & id) {
        return std::any_of(pending_scene_.robot_state.attached_collision_objects.begin(),
            pending_scene_.robot_state.attached_collision_objects.end(),
            [&](const auto & object) {
              return object.object.operation == object.object.REMOVE && object.object.id == id;
            });
      };
      objects.erase(std::remove_if(objects.begin(),
                        objects.end(),
                        [&](const auto & object) {
                          if (object.operation != object.REMOVE) return false;
                          // Detach + delete: only a held object is deleted; MoveIt detaches first.
                          if (detaching(object.id)) return !held(object.id);
                          return std::none_of(existing.world.collision_objects.begin(),
                              existing.world.collision_objects.end(),
                              [&](const auto & old) {
                                return old.id == object.id;
                              });
                        }),
          objects.end());
      auto & attached = pending_scene_.robot_state.attached_collision_objects;
      attached.erase(std::remove_if(attached.begin(),
                         attached.end(),
                         [&](const auto & object) {
                           return object.object.operation == object.object.REMOVE &&
                                  std::none_of(
                                      existing.robot_state.attached_collision_objects.begin(),
                                      existing.robot_state.attached_collision_objects.end(),
                                      [&](const auto & old) {
                                        return old.object.id == object.object.id;
                                      });
                         }),
          attached.end());
      auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
      request->scene = pending_scene_;
      future_ = client_->async_send_request(request).future.share();
    } catch (const std::exception & e) {
      query_ = {};
      return {ExecutionState::Failed, "SCENE_FAILED", e.what()};
    }
  }
  if (!future_.valid()) {
    return {ExecutionState::Idle, {}, {}};
  }
  if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    if (elapsed(started_) > 5) {
      timed_out_ = true;
      return poll();
    }
    return {ExecutionState::Running, {}, {}};
  }
  try {
    bool ok = future_.get()->success;
    future_ = {};
    return {ok ? ExecutionState::Succeeded : ExecutionState::Failed,
        ok ? "" : "SCENE_REJECTED",
        ok ? "" : "Planning scene update rejected"};
  } catch (const std::exception & e) {
    future_ = {};
    return {ExecutionState::Failed, "SCENE_FAILED", e.what()};
  }
}
} // namespace demo_1f
