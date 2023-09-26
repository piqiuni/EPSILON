#!/usr/bin/env python3.8

import os
import subprocess
import numpy as np
import yaml
import rospy
import time
import json
from tqdm import tqdm

old_case_file_names = ['1216-2022-06-18-112437', '1586-2021-08-03-170858', '1670-2022-07-15-164141',
                       '1969-2021-08-03-92532', '2651-2021-07-30-93210', '3401-2022-06-18-114355',
                       '3412-2022-08-04-170759', '3695-2022-06-25-165731', '3964-2022-06-25-161842',
                       '4070-2022-08-05-184153', '4079-2022-07-08-172812', '4684-2022-07-25-170643',
                       '4812-2022-07-08-172800', '4916-2022-07-28-165113', '5848-2021-08-19-101042',
                       '7275-2022-08-05-174640', '7393-2021-07-22-150446', '7580-2022-07-09-194412',
                       '7643-2021-08-06-144023', '8782-2022-08-15-170839', '9292-2022-08-10-183505',
                       '9491-2021-08-02-194800']

def get_case_config(case_folder_absolute_path):
    # store the case config
    config_file = 'case_config.yaml'
    config_path = os.path.join(case_folder_absolute_path, config_file)
    with open(config_path, 'r') as f:
        content = f.read()
    yamlData = yaml.load(content, Loader=yaml.FullLoader)
    PredModel = yamlData['PredModel']
    use_map_config = yamlData['use_map_config']
    return PredModel, use_map_config

class MapConfig():
    def __init__(self) -> None:
        self.scene_number = None
        self.d_a = None
        self.d_b = None
        self.remove_car_list = []
        self.start_frame = 0
        self.init_angle = -100.0
        self.use_ego_path = True
        self.start_speed = None
        self.start_acc = None

def get_map_param(case_folder_absolute_path, scene_number):
    map_config = MapConfig()
    config_file = 'map_config.yaml'
    config_path = os.path.join(case_folder_absolute_path, config_file)
    with open(config_path, 'r') as f:
        content = f.read()
    yamlData = yaml.load(content, Loader=yaml.FullLoader)
    scece_config = yamlData[scene_number]
    
    if not yamlData[scene_number]:
        print(f"[Sim]scene:{scene_number} has no map_config")
        return map_config
    
    # print(yamlData[scene_number]) 
    map_config.d_a = np.array(yamlData[scene_number]['d_a']) if yamlData[scene_number]['d_a'] else None
    # print(yamlData[scene_number]['d_a'], map_config.d_a, type(map_config.d_a))
    map_config.d_b = np.array(yamlData[scene_number]['d_b']) if yamlData[scene_number]['d_b'] else None
    map_config.remove_car_list = yamlData[scene_number]['remove_car_list']
    map_config.start_frame = yamlData[scene_number]['start_frame']
    map_config.init_angle = yamlData[scene_number]['init_angle']
    
    if (ego_path := yamlData[scene_number].get('use_ego_path', False)) is not False:
        map_config.use_ego_path = True if ego_path else False
    return map_config

def run_roslaunch():
    # roslaunch命令和参数
    command = ['roslaunch', 'phy_simulator', 'total_for_testing.launch']

    # 运行roslaunch进程
    # process = subprocess.Popen(command)
    # process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # 等待进程结束
    process.wait()

    # 获取进程的标准输出和错误输出
    # stdout, stderr = process.communicate()

    # 输出标准输出和错误输出内容
    # print("Standard Output:")
    # print(stdout.decode())

    # print("Error Output:")
    # print(stderr.decode())
    

    # 检查进程的返回值
    if process.returncode == 0:
        print("roslaunch process completed successfully.")
    else:
        print("roslaunch process failed.")


if __name__ == "__main__":
    rospy.init_node("sim_testing", anonymous=True)
    
    data_folder_absolute_path = '/home/rancho/2lidelun/Huawei-dataset/DATA'
    log_folder_absolute_path = '/home/rancho/2lidelun/Huawei-dataset/LOG'
    
    test_cases = ["case_old", "case_0601", "case_0706"]
    
    # test_cases = ["case_old"]
    
    test_paths = []
    
    for case_name in test_cases:
        case_folder_absolute_path = data_folder_absolute_path + '/' + case_name
        full_file_names = [f for f in os.listdir(case_folder_absolute_path) if not f.endswith(('.yaml', '.md'))]
        if case_name == 'case_old':
            full_file_names = old_case_file_names
        file_names = [name for name in full_file_names]
        for file_name in file_names:
            index = file_names.index(file_name)
            
            # if index != 12:
            #     continue
            
            test_paths.append([case_name, case_folder_absolute_path, file_name, index])
    print(test_paths)
    for i in tqdm(range(0, len(test_paths))):
        case_name, case_folder_absolute_path, file_name, index = test_paths[i]
        case_path = case_folder_absolute_path + '/' + file_name
        # print(index, case_path)
        PredModel, use_map_config = get_case_config(case_folder_absolute_path)
        
        # print(PredModel, use_map_config)
        if use_map_config:
            map_config = get_map_param(case_folder_absolute_path, index)
        else:
            map_config = MapConfig()

        start_frame = map_config.start_frame
        init_angle = map_config.init_angle if map_config.init_angle else -100.0
        use_ego_path = map_config.use_ego_path
        folder_name = 'EPSILON/' + case_name
        folder_name = 'EPSILON_0918_fix_egopath/' + case_name
        # folder_name = 'EPSILON_0918_fix_egopath'
        
        rospy.set_param('/hard_case_phy_simulator_planning_node/case_path', case_path)
        rospy.set_param('/hard_case_phy_simulator_planning_node/case_id', index)
        rospy.set_param('/hard_case_phy_simulator_planning_node/folder_name', folder_name)
        rospy.set_param('/hard_case_phy_simulator_planning_node/init_angle', init_angle)
        rospy.set_param('/hard_case_phy_simulator_planning_node/start_frame', start_frame)
        rospy.set_param('/hard_case_phy_simulator_planning_node/use_ego_path', use_ego_path)
        
        # print(case_name, start_frame, init_angle)
        
        run_roslaunch()
        
        # break
        
        
        
        

        
        