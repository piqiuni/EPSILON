
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include "dataset/get_raw_data.h"

using Json = nlohmann::json;

int main() {
    // std::vector<Eigen::Vector2d> ego_path = {
    //     Eigen::Vector2d(-25.117972490959801, -13.722607914824039),
    //     Eigen::Vector2d(-21.048880392219871, -19.214927807915956),
    //     Eigen::Vector2d(-21.048880392219871, -19.214927807915956),
    //     Eigen::Vector2d(-21.048880392219871, -19.214927807915956),
    //     Eigen::Vector2d(-21.048880392219871, -19.214927807915956)
    // };

    std::string save_path_lane_net =    "/home/rancho/2lidelun/EPSILON_ws/src/EPSILON/core/playgrounds/hard_cases/lane_net_norm.json";
    std::string save_path_vehicle_set = "/home/rancho/2lidelun/EPSILON_ws/src/EPSILON/core/playgrounds/hard_cases/vehicle_set.json";

    std::vector<Json> frames;
    std::string scene_path;
    // scene_path = "/home/rancho/2lidelun/Huawei-dataset/DATA/case_0901/segment24##goal_pre#source_202308251731#generate#dataVector.json-1688353978";
    // scene_path = "/home/rancho/2lidelun/Huawei-dataset/DATA/case_0706/segment3##goal_pre#source_202305#generate#dataVector.json-1686731982";
    scene_path = "/home/rancho/2lidelun/Huawei-dataset/DATA/case_old/3964-2022-06-25-161842";
    get_raw_data(scene_path, frames);

    std::vector<Eigen::Vector2d> ego_path;
    ErrorType result1 = get_ego_path(frames, ego_path);

    ErrorType result2 = save_ego_path(ego_path, save_path_lane_net);
    ErrorType result_ego = save_ego_pos(frames[0], save_path_vehicle_set);
    
    if (result_ego == kSuccess) {
        std::cout << "Ego path saved successfully." << std::endl;
    } else {
        std::cout << "Failed to save ego path." << std::endl;
    }

    return 0;
}