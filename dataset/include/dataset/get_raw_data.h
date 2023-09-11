#ifndef _DATASET_RAW_DATA_H_
#define _DATASET_RAW_DATA_H_

#include <assert.h>
#include <iostream>
#include <vector>
#include <filesystem>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <json/json.hpp>
#include "common/basics/basics.h"
#include "common/state/state.h"
#include "common/basics/semantics.h"

using Json = nlohmann::json;



// enum ErrorType { kSuccess = 0, kFailure, kWrongStatus, kIllegalInput, kUnknown };

ErrorType get_raw_data(const std::string & scene_path, std::vector<Json> & frames) {
    printf("\n[DATASET] Loading raw data\n");

    std::fstream fs(scene_path);
    Json raw_data;
    fs >> raw_data;
    // std::vector<Json> frames;
    
    for (int i = 0; i < static_cast<int>(raw_data.size()); i++)
    {
        Json frame = raw_data[i];
        frames.push_back(frame);

    }
    printf("\n[DATASET] Loading raw data done\n");
    fs.close();
    return kSuccess;
}

ErrorType get_ego_path(const std::vector<Json>& frames, std::vector<Eigen::Vector2d>& ego_path, const int prolong = 200) {
    printf("\n[DATASET] Geting ego path\n");
    // std::vector<Json> frames;
    ego_path.clear();
    Eigen::Vector2d pt;
    Eigen::Vector2d last_pt(-100000, -100000);
    double ego_x_relative, ego_y_relative;
    for (int i = 0; i < static_cast<int>(frames.size()); i++)
    {
        Json frame = frames[i];
        ego_x_relative = frame["ego_x_relative"].get<double>() - 0;
        // std::cout << ego_x_relative << std::endl;
        ego_y_relative = frame["ego_y_relative"].get<double>() - 0;
        // std::cout << ego_y_relative << std::endl;
        // std::cout << obj_x_relative, obj_y_relative << std::endl;
        Eigen::Vector2d pt(ego_x_relative, ego_y_relative);
        if ((last_pt - pt).norm() > 1.0)
        {
            ego_path.push_back(pt);
            last_pt = pt;
        }
    }
    // prolong the ego path
    const int n = static_cast<int>(ego_path.size());
    Eigen::Vector2d start_pt = ego_path[n-3];
    Eigen::Vector2d end_pt = ego_path[n-1];
    Eigen::Vector2d dir = (end_pt - start_pt).normalized();

    for (int i = 1; i < int(prolong/10); i++)
    {
        Eigen::Vector2d pt = end_pt + dir * i * 10;
        ego_path.push_back(pt);
    }

    start_pt = ego_path[3];
    end_pt = ego_path[0];
    Eigen::Vector2d back_dir = (end_pt - start_pt).normalized();

    for (int i = 1; i < 5; i++)
    {
        Eigen::Vector2d pt = end_pt + back_dir * i * 10;
        ego_path.emplace(ego_path.begin(),pt);
    }

    return kSuccess;
}


ErrorType save_ego_path(const std::vector<Eigen::Vector2d>& ego_path, const std::string& save_path) {

    // 创建 JSON 对象
    Json json_data;
    json_data["crs"]["type"] = "name";
    json_data["crs"]["properties"]["name"] = "hard_cases";
    json_data["type"] = "FeatureCollection";
    json_data["name"] = "hard_case";

    // 创建 coordinates 数组
    Json coordinates;
    for (int i = 0; i < ego_path.size(); i++) {
        coordinates.push_back(
            {ego_path[i].x(), ego_path[i].y()}
        );
    }
    json_data["features"][0]["geometry"]["type"] = "MultiLineString";
    json_data["features"][0]["geometry"]["coordinates"][0] = coordinates;
    
    // 替换 length 值
    double length = 0;  // 替换为新的长度值
    for (size_t i = 0; i < ego_path.size() - 1; ++i) {
        const Eigen::Vector2d& current_point = ego_path[i];
        const Eigen::Vector2d& next_point = ego_path[i + 1];
        double segment_distance = (next_point - current_point).norm();
        length += segment_distance;
    }

    json_data["features"][0]["type"] = "Feature";
    json_data["features"][0]["properties"]["length"] = length;
    json_data["features"][0]["properties"]["right_id"] = 0;
    json_data["features"][0]["properties"]["left_id"] = 0;
    json_data["features"][0]["properties"]["rchg_vld"] = 0;
    json_data["features"][0]["properties"]["father_id"] = "0";
    json_data["features"][0]["properties"]["behavior"] = "s";
    json_data["features"][0]["properties"]["child_id"] = "0";
    json_data["features"][0]["properties"]["lchg_vld"] = 0;
    json_data["features"][0]["properties"]["id"] = 1;

    // 将 JSON 写入文件
    std::ofstream output_file(save_path);
    if (!output_file.is_open()) {
        std::cout << "Failed to open file: " << save_path << std::endl;
        return kWrongStatus;
    }
    output_file << json_data.dump(2);  // 使用 4 个空格进行缩进
    output_file.close();

    return kSuccess;
}

ErrorType save_ego_pos(const Json& init_frame, const std::string& save_path, const double init_angle = -100) {
    double ego_x_relative = init_frame["ego_x_relative"].get<double>() - 0;
    double ego_y_relative = init_frame["ego_y_relative"].get<double>() - 0;
    
    double ego_theta_relative = init_frame["ego_theta_relative"].get<double>() - 0;
    double obj_v_map = init_frame["ego_v_relative"].get<double>() - 0;

    printf("\n[DATASET] Save ego pos\n");
    // 创建 JSON 对象
    Json json_data;
    Json vehicles;
    Json info;
    Json init_state;
    Json params;
    
    vehicles["_COMMENT"] = "[Vehicles info] - START";
    vehicles["__COMMENT"] = "Vehicle Subclass: car, van, truck.";
    vehicles["___COMMENT"] = "Vehicle Type: VehicleWithBicycleKinematics, VehicleDoubleIntegratorKinematics, VehicleSingleIntegratorKinematics";
    vehicles["num"] = 1;
    
    info["id"] = 0;
    info["subclass"] = "car";
    info["type"] = "VehicleWithBicycleKinematics";
    
    init_state["x"] = ego_x_relative;
    init_state["y"] = ego_y_relative;
    init_state["angle"] = ego_theta_relative;
    if (init_angle != -100) init_state["angle"] = init_angle;
    init_state["curvature"] = 0.0;
    init_state["velocity"] = obj_v_map;
    init_state["steer"] = 0.0;
    init_state["acceleration"] = 0.0;
    info["init_state"] = init_state;

    params["width"] = 1.9;
    params["length"] = 4.5;
    params["wheel_base"] = 2.85;
    params["front_suspension"] = 0.93;
    params["rear_suspension"] = 1.10;
    params["max_steering_angle"] = 45.0;
    params["max_longitudinal_acc"] = 2.0;
    params["max_lateral_acc"] = 2.0;
    info["params"] = params;
    vehicles["info"][0] = info;
    vehicles["COMMENT_"] = "[Vehicles info] - END";

    json_data["vehicles"] = vehicles;
    
    printf("\n[DATASET] Save ego pos\n");
    // 将 JSON 写入文件
    std::ofstream output_file(save_path);
    if (!output_file.is_open()) {
        std::cout << "Failed to open file: " << save_path << std::endl;
        return kWrongStatus;
    }
    output_file << json_data.dump(2);  // 使用 2 个空格进行缩进
    output_file.close();

    return kSuccess;
}

ErrorType get_key_objects(const Json& frame, const common::Vehicle& ego_vehicle, std::vector<common::Vehicle>& key_objects)
{
    
    printf("\n[DATASET] Get key objects\n");
    Json objs = frame["objects"];
    common::State ego_state = ego_vehicle.state();

    double ego_x_relative = ego_state.vec_position(0, 0);
    double ego_y_relative = ego_state.vec_position(1, 0);
    Eigen::Vector2d ego_pos(ego_x_relative, ego_y_relative);
    double ego_theta_relative = ego_state.angle;
    double ego_v = ego_state.velocity;
    key_objects.clear();
    for(auto item:objs.items())
    {
        // std::cout<<item.key()<<" "<<item.value()<<std::endl;
        Json obj = item.value();
        double obj_x_relative = obj["obj_x_relative"].get<double>();
        double obj_y_relative = obj["obj_y_relative"].get<double>();
        Eigen::Vector2d obj_pos(obj_x_relative, obj_y_relative);
        double obj_v_map = obj["obj_v_map"].get<double>();
        double obj_heading_relative = obj["obj_heading_absolute"].get<double>();
        // double obj_theta_map = obj["obj_theta_map"].get<double>();
        double obj_width = obj["obj_width"].get<double>();
        double obj_length = obj["obj_length"].get<double>();
        double dist = (obj_pos - ego_pos).norm();
        double d_angle = atan2(obj_pos.y() - ego_pos.y(), obj_pos.x() - ego_pos.x()) - ego_theta_relative;


        if(dist >= 50) {continue;}
        else if(0) {continue;}



        common::VehicleParam param;
        common::State state;
        state.vec_position << obj_x_relative, obj_y_relative;
        state.angle = obj_heading_relative;
        state.velocity = obj_v_map;

        param.set_width(obj_width);
        param.set_length(obj_length);
        param.set_wheel_base(obj_length*0.6);

        common::Vehicle vehicle_new(1, "car", param, state);
        key_objects.emplace_back(vehicle_new);
    }

    return kSuccess;
}

#endif

