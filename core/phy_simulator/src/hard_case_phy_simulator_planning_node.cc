#include <stdlib.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud.h>

#include "vehicle_msgs/ControlSignal.h"
#include "vehicle_msgs/decoder.h"
#include "vehicle_msgs/encoder.h"

#include "phy_simulator/basics.h"
#include "phy_simulator/phy_simulator.h"
#include "phy_simulator/ros_adapter.h"
#include "phy_simulator/visualizer.h"

#include "dataset/get_raw_data.h"
// #include "dataset

using namespace phy_simulator;

DECLARE_BACKWARD;
const double simulation_rate = 10.0;
const double gt_msg_rate = 10.0;
const double gt_static_msg_rate = 10.0;
const double visualization_msg_rate = 10.0;

common::VehicleControlSignalSet _signal_set;
std::vector<ros::Subscriber> _ros_sub;

Vec3f initial_state(0, 0, 0);
bool flag_rcv_initial_state = false;

Vec3f goal_state(0, 0, 0);
bool flag_rcv_goal_state = false;

void CtrlSignalCallback(const vehicle_msgs::ControlSignal::ConstPtr& msg,
                        int index) {
  common::VehicleControlSignal ctrl;
  vehicle_msgs::Decoder::GetControlSignalFromRosControlSignal(*msg, &ctrl);
  _signal_set.signal_set[index] = ctrl;
}

void InitialPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  common::VisualizationUtil::Get3DofStateFromRosPose(msg->pose.pose,
                                                     &initial_state);
  flag_rcv_initial_state = true;
}

void NavGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  common::VisualizationUtil::Get3DofStateFromRosPose(msg->pose, &goal_state);
  flag_rcv_goal_state = true;
}



int main(int argc, char** argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");

  // 1.read the case.json files
  std::string case_path;
  if (!nh.getParam("case_path", case_path)) {
    ROS_ERROR("Failed to get param case_path");
    assert(false);
  }
  int case_id;
  if (!nh.getParam("case_id", case_id)) {
    ROS_ERROR("Failed to get param case_id");
    assert(false);
  }
  int start_frame;
  int frame_now, max_frame;
  if (!nh.getParam("start_frame", start_frame)) {
    ROS_ERROR("Failed to get param start_frame");
    assert(false);
  }
  frame_now = start_frame;

  double init_angle;
  if (!nh.getParam("init_angle", init_angle)) {
    ROS_ERROR("Failed to get param init_angle");
    assert(false);
  }
  
  std::vector<Json> frames;
  get_raw_data(case_path, frames);
  max_frame = frames.size();



  // 2.deal with the .json files
  //  get agent_config.json, set init state
  std::string save_path_vehicle_set = "/home/rancho/2lidelun/EPSILON_ws/src/EPSILON/core/playgrounds/hard_cases/vehicle_set.json";
  ErrorType result_ego = save_ego_pos(frames[start_frame], save_path_vehicle_set, init_angle);
  if (result_ego != kSuccess) {
    std::cout << "Failed to save_path_vehicle_set." << std::endl;
    assert(false);
  }

  //  get lane_net.json 
  std::vector<Eigen::Vector2d> ego_path;
  std::string save_path_lane_net =    "/home/rancho/2lidelun/EPSILON_ws/src/EPSILON/core/playgrounds/hard_cases/lane_net_norm.json";
  ErrorType result1 = get_ego_path(frames, ego_path);
  if (result1 != kSuccess) {
    std::cout << "Failed to save_path_lane_net." << std::endl;
    assert(false);
  }
  ErrorType result2 = save_ego_path(ego_path, save_path_lane_net);
  if (result2 != kSuccess) {
    std::cout << "Failed to save_ego_path." << std::endl;
    assert(false);
  }

  // 3.read .json 
  std::string vehicle_info_path;
  if (!nh.getParam("vehicle_info_path", vehicle_info_path)) {
    ROS_ERROR("Failed to get param %s", vehicle_info_path.c_str());
    assert(false);
  }
  std::string map_path;
  if (!nh.getParam("map_path", map_path)) {
    ROS_ERROR("Failed to get param %s", map_path.c_str());
    assert(false);
  }
  std::string lane_net_path;
  if (!nh.getParam("lane_net_path", lane_net_path)) {
    ROS_ERROR("Failed to get param %s", lane_net_path.c_str());
    assert(false);
  }

  PhySimulation phy_sim(vehicle_info_path, map_path, lane_net_path);

  RosAdapter ros_adapter(nh);
  ros_adapter.set_phy_sim(&phy_sim);

  Visualizer visualizer(nh);
  visualizer.set_phy_sim(&phy_sim);

  auto vehicle_ids = phy_sim.vehicle_ids();
  int num_vehicles = static_cast<int>(vehicle_ids.size());
  num_vehicles = 1;
  _ros_sub.resize(num_vehicles);

  for (int i = 0; i < num_vehicles; i++) {
    auto vehicle_id = vehicle_ids[i];
    std::string topic_name =
        std::string("/ctrl/agent_") + std::to_string(vehicle_id);
    printf("subscribing to %s\n", topic_name.c_str());
    _ros_sub[i] = nh.subscribe<vehicle_msgs::ControlSignal>(
        topic_name, 10, boost::bind(CtrlSignalCallback, _1, vehicle_id));
  }

  for (auto& vehicle_id : vehicle_ids) {
    common::VehicleControlSignal default_signal;
    _signal_set.signal_set.insert(std::pair<int, common::VehicleControlSignal>(
        vehicle_id, default_signal));
  }

  ros::Subscriber ini_pos_sub =
      nh.subscribe("/initialpose", 10, InitialPoseCallback);
  ros::Subscriber goal_pos_sub =
      nh.subscribe("/move_base_simple/goal", 10, NavGoalCallback);

  ros::Rate rate(simulation_rate);
  ros::Time next_gt_pub_time = ros::Time::now();
  ros::Time next_gt_static_pub_time = next_gt_pub_time;
  ros::Time next_vis_pub_time = ros::Time::now();

  std::cout << "[PhySimulation] Initialization finished, waiting for callback"
            << std::endl;

  int gt_msg_counter = 0;

  common::VehicleSet vehicle_set;
  std::unordered_map<int, common::Vehicle> vehicles;
  common::VehicleParam param;
  common::State state;
  // state.vec_position << 2500, -5300;
  // common::Vehicle vehicle_1(1, "car", param, state);
  // Vehicle(const int &id, const std::string &subclass, const VehicleParam &param, const State &state);

  // vehicle_set.vehicles = vehicles;
  // vehicles.emplace(1, vehicle_1);
  common::Vehicle ego_vehicle;

  while (ros::ok()) {
    ros::spinOnce();

    phy_sim.UpdateSimulatorUsingSignalSet(_signal_set, 1.0 / simulation_rate);
    vehicle_set = phy_sim.vehicle_set();
    ego_vehicle = vehicle_set.vehicles[0];
    vehicles = vehicle_set.vehicles;
    // 4. update the vehicle_set, add vehicles from the dataset
    Json frame_data_now = frames[frame_now];
    // select the key objects
    std::vector<common::Vehicle> key_objects;
    auto time0 = ros::Time::now();
    get_key_objects(frame_data_now, ego_vehicle, key_objects);
    std::cout << "get_key_objects time:" << ros::Time::now() - time0 << std::endl;
    int sim_id = 1;
    // get key objects' states
    for(common::Vehicle obj: key_objects)
    {
      
      // double obj_x_relative = obj["obj_x_relative"].get<double>();
      // double obj_y_relative = obj["obj_y_relative"].get<double>();
      // double obj_v_map = obj["obj_v_map"].get<double>();
      // double obj_heading_relative = obj["obj_heading_relative"].get<double>();
      // // double obj_theta_map = obj["obj_theta_map"].get<double>();
      // double obj_width = obj["obj_width"].get<double>();
      // double obj_length = obj["obj_length"].get<double>();

      // state.vec_position << obj_x_relative, obj_y_relative;
      // state.angle << obj_heading_relative;
      // state.velocity << obj_v_map;

      // param.set_width(obj_width);
      // param.set_length(obj_length);
      // param.set_wheel_base(obj_length*0.6);

      // common::Vehicle vehicle_new(sim_id, "car", param, state);

      obj.set_id(sim_id);
      vehicles.emplace(sim_id, obj);
      sim_id += 1;
    }
    std::cout << "+++++++++++sim_id:" << sim_id << std::endl;
    std::cout << "+++++++++++frame_now:" << frame_now << ", max_frame" << max_frame << std::endl;

    vehicle_set.vehicles = vehicles;
    frame_now += 1;

    // UpdateVehicleInfo(vehicle_set, );
    ros::Time tnow = ros::Time::now();
    if (tnow >= next_gt_pub_time) {
      next_gt_pub_time += ros::Duration(1.0 / gt_msg_rate);

      ros_adapter.HC_PublishDynamicDataWithStamp(vehicle_set, tnow);
    }

    if (tnow >= next_gt_static_pub_time) {
      next_gt_static_pub_time += ros::Duration(1.0 / gt_static_msg_rate);
      vehicle_msgs::ArenaInfoStatic msg;
      ros_adapter.PublishStaticDataWithStamp(tnow);
    }


    if (tnow >= next_vis_pub_time) {
      next_vis_pub_time += ros::Duration(1.0 / visualization_msg_rate);
      visualizer.VisualizeDataWithStamp(tnow);
    }

    rate.sleep();
  }

  _ros_sub.clear();
  ros::shutdown();
  return 0;
}
