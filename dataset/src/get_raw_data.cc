// #include "get_raw_data.h"

// using Json = nlohmann::json;

// ErrorType get_raw_data(const std::string & scene_path, std::vector<Json> & frames) {
//     printf("\n[DATASET] Loading raw data\n");

//     std::fstream fs(scene_path);
//     Json raw_data;
//     fs >> raw_data;
//     // std::vector<Json> frames;
    
//     for (int i = 0; i < static_cast<int>(raw_data.size()); i++)
//     {
//         Json frame = raw_data[i];
//         frames.push_back(frame);

//     }

//   fs.close();
//   return kSuccess;
// }

// ErrorType get_ego_path(const std::string & scene_path, std::vector<Json> & frames, const double prolong) {

//     // std::vector<Json> frames;
    
//     for (int i = 0; i < static_cast<int>(raw_data.size()); i++)
//     {
//         Json frame = raw_data[i];
//         frames.push_back(frame);

//     }

//   return kSuccess;
// }