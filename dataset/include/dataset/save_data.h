#ifndef _SAVE_DATA_H_
#define _SAVE_DATA_H_

#include <iostream>
#include <fstream>
#include <assert.h>
#include <json/json.hpp>
#include "common/basics/basics.h"
#include "common/state/state.h"
#include "common/basics/semantics.h"


using Json = nlohmann::json;

void save_data(const std::string & save_path, Json& data, const std::string name)
{
    std::ofstream file(save_path);
    data["scene_name"] = name;
    int first_frame = data["frames"][0]["timestamp"].get<int>();
    int frame_num = data["frames"].size();
    int max_frame = data["frames"][data["frames"].size() - 1]["timestamp"].get<int>();
    data["first_frame"] = first_frame;
    data["frame_num"] = frame_num;
    data["max_frame"] = max_frame;


    if (file.is_open()) {
        file << data;
        file.close();
        std::cout << "JSON data saved to file." << std::endl;
        std::cout << "save_path: " << save_path << std::endl;
    } else {
        std::cout << "Failed to open file for writing." << std::endl;
        assert(false);
    }
}

void save_data_frame(const std::unordered_map<int, common::Vehicle>& objects, const int timestamp, Json& save_this_frame) {
    save_this_frame.clear();
    save_this_frame["timestamp"] = timestamp;

    auto ego_vehecle = objects.at(0);
    auto ego_state = ego_vehecle.state();
    auto ego_param = ego_vehecle.param();
    Json ego;
    ego["id"] = 0;
    ego["x"] = ego_state.vec_position(0, 0);
    ego["y"] = ego_state.vec_position(1, 0);
    ego["theta"] = ego_state.angle;
    ego["v"] = ego_state.velocity;
    ego["a"] = ego_state.acceleration;
    ego["jerk"] = 0;
    ego["width"] = ego_param.width();
    ego["length"] = ego_param.length();
    save_this_frame["ego_state"] = ego;

    for (const auto& pair : objects) {
        // std::cout << "Key: " << pair.first << ", Value: " << pair.second << std::endl;
        auto vehecle = pair.second;
        int id = vehecle.id();
        if (id == 0) { continue;}

        auto state = vehecle.state();
        auto param = vehecle.param();
        double x = state.vec_position(0, 0);
        double y = state.vec_position(1, 0);
        double theta = state.angle;
        double v = state.velocity;
        double a = state.acceleration;
        double jerk = 0;
        double width = param.width();
        double length = param.length();
        Json obj;
        obj["id"] = id;
        obj["x"] = x;
        obj["y"] = y;
        obj["theta"] = theta;
        obj["v"] = v;
        obj["a"] = a;
        obj["jerk"] = jerk;
        obj["width"] = width;
        obj["length"] = length;
        save_this_frame["objects"].push_back(obj);
    }
}


#endif