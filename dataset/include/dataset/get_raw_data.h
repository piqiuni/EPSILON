#ifndef _DATASET_RAW_DATA_H_
#define _DATASET_RAW_DATA_H_

#include <assert.h>
#include <iostream>
#include <vector>
#include <filesystem>

#include <typeinfo>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <json/json.hpp>
#include "common/basics/basics.h"
#include "common/state/state.h"
#include "common/basics/semantics.h"

using Json = nlohmann::json;

std::vector<int> obs_type_list = {0, 12, 13, 14, 15, 16, 17, 18, 19, 10, 20, 24, 33};


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

ErrorType get_lanes(const std::vector<Json> & frames, std::unordered_map<std::string, std::vector<Eigen::Vector2d>> & lanes) {
    std::vector<Eigen::Vector2d> lane;
    lanes.clear();
    for (int i = 0; i < static_cast<int>(frames.size()); i++)
    {
        
        Json frame = frames[i];
        Json routes = frame["routes"];
        for (auto item:routes.items())
        {
            // std::cout<<item.key()<<" "<<item.value()<<std::endl;
            std::string id = item.key();
            if (lanes.find(id) != lanes.end())
            {
                continue;
            }
            lane.clear();
            Json route = item.value();
            for (auto item: route["points"])
            {
                Json route_pt = item;
                double x = route_pt["route_point_x_relative"].get<double>();
                double y = route_pt["route_point_y_relative"].get<double>();
                lane.emplace_back(Eigen::Vector2d(x, y));
            }
            lanes.emplace(id, lane);
        }        
    }
    return kSuccess;
}

ErrorType get_ego_lane(const Json & frame_now, const std::unordered_map<std::string, std::vector<Eigen::Vector2d>> & lanes, std::vector<Eigen::Vector2d> &ego_path) {
    ego_path.clear();
    // Json frame = frame_now;
    int ego_lane_id = frame_now["ego_current_route_id"].get<int>();
    // std::cout << frame["ego_current_route_id"] << std::endl;

    auto lane = lanes.find(std::to_string(ego_lane_id));
    // auto lane = lanes.find(ego_lane_id_str);
    ego_path = lane->second;
    // prolong the ego path
    int prolong = 200;
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

ErrorType get_obj_path(const std::vector<Json>& frames, std::vector<Eigen::Vector2d>& obj_path, const int id, const int prolong = 200) {
    printf("\n[DATASET] Geting ego path\n");
    // std::vector<Json> frames;
    obj_path.clear();
    Eigen::Vector2d last_pt(-100000, -100000);
    double obj_x_relative, obj_y_relative;
    for (int i = 0; i < static_cast<int>(frames.size()); i++)
    {
        Json frame = frames[i];
        Json objs = frame["objects"];
        // FIXME: use dict?
        for(auto item:objs.items())
        {
            Json obj = item.value();
            int obj_id = obj["obj_id"].get<int>();
            if (obj_id != id)
            {
                continue;
            }
            double obj_x_relative = obj["obj_x_relative"].get<double>();
            double obj_y_relative = obj["obj_y_relative"].get<double>();
            Eigen::Vector2d pt(obj_x_relative, obj_y_relative);
            if ((last_pt - pt).norm() > 1.0)
            {
                obj_path.push_back(pt);
                last_pt = pt;
            }
            break;
        }
    }
    if (obj_path.size() < 5)
    {
        obj_path.clear();
        return kWrongStatus;
    }

    // prolong the ego path
    const int n = static_cast<int>(obj_path.size());
    Eigen::Vector2d start_pt = obj_path[n-3];
    Eigen::Vector2d end_pt = obj_path[n-1];
    Eigen::Vector2d dir = (end_pt - start_pt).normalized();

    for (int i = 1; i < int(prolong/10); i++)
    {
        Eigen::Vector2d pt = end_pt + dir * i * 10;
        obj_path.push_back(pt);
    }

    start_pt = obj_path[3];
    end_pt = obj_path[0];
    Eigen::Vector2d back_dir = (end_pt - start_pt).normalized();

    for (int i = 1; i < 5; i++)
    {
        Eigen::Vector2d pt = end_pt + back_dir * i * 10;
        obj_path.emplace(obj_path.begin(),pt);
    }

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

ErrorType save_lanes(const std::vector<std::vector<Eigen::Vector2d>>& lanes, const std::string& save_path) {

    // 创建 JSON 对象
    Json json_data;
    json_data["crs"]["type"] = "name";
    json_data["crs"]["properties"]["name"] = "hard_cases";
    json_data["type"] = "FeatureCollection";
    json_data["name"] = "hard_case";

    for (int i = 0; i < lanes.size(); i++) {
        Json coordinates;
        
        for (int j = 0; j < lanes[i].size(); j++) {
            coordinates.push_back(
                {lanes[i][j].x(), lanes[i][j].y()}
            );
        }
        json_data["features"][i]["geometry"]["type"] = "MultiLineString";
        json_data["features"][i]["geometry"]["coordinates"][0] = coordinates;
        // 替换 length 值
        double length = 0;  // 替换为新的长度值
        for (size_t k = 0; k < lanes[i].size() - 1; ++k) {
            const Eigen::Vector2d& current_point = lanes[i][k];
            const Eigen::Vector2d& next_point = lanes[i][k + 1];
            double segment_distance = (next_point - current_point).norm();
            length += segment_distance;
        }

        json_data["features"][i]["type"] = "Feature";
        json_data["features"][i]["properties"]["length"] = length;
        json_data["features"][i]["properties"]["right_id"] = 0;

        // if (i == 0) {json_data["features"][i]["properties"]["right_id"] = 2;}

        json_data["features"][i]["properties"]["left_id"] = 0;

        // if (i == 1) {json_data["features"][i]["properties"]["left_id"] = 1;}

        json_data["features"][i]["properties"]["rchg_vld"] = 0;

        json_data["features"][i]["properties"]["father_id"] = "0";

        json_data["features"][i]["properties"]["behavior"] = "s";
        json_data["features"][i]["properties"]["child_id"] = "0";
        json_data["features"][i]["properties"]["lchg_vld"] = 0;

        // if (i == 1) {json_data["features"][i]["properties"]["lchg_vld"] = 1;}

        json_data["features"][i]["properties"]["id"] = i+1;
    }


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
    
    printf("[DATASET] Get key objects\t");
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
        Json obj = item.value();
        // std::cout<<item.key()<<" "<<item.value()<<std::endl;
        int obj_type = obj["obj_class"].get<int>();
        if (std::count(obs_type_list.begin(), obs_type_list.end(), obj_type))
        {
            printf("obj_type: %d\t", obj_type);
            continue;
        }
        
        
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


        if(dist >= 100) {continue;}
        else if(0) {continue;}



        common::VehicleParam param;
        common::State state;
        state.vec_position << obj_x_relative, obj_y_relative;
        state.angle = obj_heading_relative;
        state.velocity = obj_v_map;

        param.set_width(obj_width);
        param.set_length(obj_length);
        param.set_wheel_base(obj_length*0.6);
        // std::cout << std::stoi(item.key()) << ", " << (item.key()) << std::endl;
        common::Vehicle vehicle_new(std::stoi(item.key()), "car", param, state);
        key_objects.emplace_back(vehicle_new);
    }

    return kSuccess;
}

#endif

