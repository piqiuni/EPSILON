import math
import os
import json
from typing import Dict

import numpy as np
import pandas as pd
from shapely.geometry import Polygon

class Obj_State(object):
    def __init__(self, id, x, y, yaw, v, a, jerk, length, width) -> None:
        self.id = id
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.a = a
        self.jerk = jerk
        self.length = length
        self.width = width

class Test_get_info(object):
    def __init__(self) -> None:
        self.scene_info_dict = {}
        self._init_info_dict()
        
    def read_json(self, data_path):
        with open(data_path, 'r') as file:
            json_data = json.load(file)
        index = data_path.split('/')[-1].split('_')[0]
        self.data = {}
        self.data['scene_id'] = index
        self.data['scene_name'] = json_data['scene_name']
        self.data['first_frame'] = json_data['first_frame']
        self.data['frame_num'] = json_data['frame_num']
        self.data['max_frame'] = json_data['max_frame']
        self.data['frame'] = {}
        for frame in json_data['frames']:
            frame_data = {}
            timestamp = frame['timestamp']
            id = frame['ego_state']['id']
            x = frame['ego_state']['x']
            y = frame['ego_state']['y']
            yaw = frame['ego_state']['theta']
            v = frame['ego_state']['v']
            a = frame['ego_state']['a']
            jerk = frame['ego_state']['jerk']
            length = frame['ego_state']['length']
            width = frame['ego_state']['width']
            
            ego_state = Obj_State(id, x, y, yaw, v, a, jerk, length, width)
            frame_data['timestamp'] = timestamp
            frame_data['ego_state'] = ego_state
            obj_dict = {}
            # print(frame.keys())
            if 'objects' in frame.keys():
                for obj in frame['objects']:
                    id = obj['id']
                    x = obj['x']
                    y = obj['y']
                    yaw = obj['theta']
                    v = obj['v']
                    a = obj['a']
                    jerk = obj['jerk']
                    length = obj['length']
                    width = obj['width']
                    obj_state = Obj_State(id, x, y, yaw, v, a, jerk, length, width)
                    obj_dict[obj_state.id] = obj_state
                
            frame_data['obj_dict'] = obj_dict
            self.data['frame'][timestamp] = frame_data
        self.run_sim_data()
    
    def run_sim_data(self):
        self.data['ego_speed'] : Dict[int, float] = {}
        self.data['ego_acc'] : Dict[int, float] = {}
        self.data['ego_jerk'] : Dict[int, float] = {}
        self.data['result'] = 'no collision'
        self.data['collision_frame'] = []
        self.data['collision_state'] = ''
        for time_stamp, frame_data in self.data['frame'].items():
            frame = self.data['frame'][time_stamp]
            self.data['ego_speed'][time_stamp] = frame['ego_state'].v
            self.data['ego_acc'][time_stamp] = frame['ego_state'].a
            self.data['ego_jerk'][time_stamp] = frame['ego_state'].jerk
            ego_x = frame['ego_state'].x
            ego_y = frame['ego_state'].y
            ego_yaw = frame['ego_state'].yaw
            ego_polygon = self.get_polygon(frame['ego_state'])
            for obj in frame['obj_dict'].values():
                obj_x = obj.x
                obj_y = obj.y
                obj_yaw = obj.yaw
                obj_length = obj.length
                obj_width = obj.width
                obj_polygon = self.get_polygon(obj)
                is_collision, collision_state = self.is_collision(ego_polygon, ego_yaw, obj_polygon)
                if is_collision:
                    self.data['result'] = 'collision'
                    self.data['collision_frame'].append(time_stamp)
                    self.data['collision_state'] = collision_state
                    return
            
            
        
    
    def get_polygon(self, ego_state: Obj_State):
        x, y, yaw, l, w = ego_state.x, ego_state.y, ego_state.yaw, ego_state.length, ego_state.width
        polygon = Polygon([[x - l/2 * math.cos(yaw) + w/2 * math.sin(yaw),
                                                y - l/2 * math.sin(yaw) - w/2 * math.cos(yaw)],
                                                [x - l/2 * math.cos(yaw) - w/2 * math.sin(yaw),
                                                y - l/2 * math.sin(yaw) + w/2 * math.cos(yaw)],
                                                [x +l/2 * math.cos(yaw) - w/2 * math.sin(yaw),
                                                y + l/2 * math.sin(yaw) + w/2 * math.cos(yaw)],
                                                [x + l/2 * math.cos(yaw) + w/2 * math.sin(yaw),
                                                y + l/2 * math.sin(yaw) - w/2 * math.cos(yaw)]])
        return polygon
    
    def is_collision(self, ego_polygon : Polygon, ego_heading, polygon):
        res = ego_polygon.intersects(polygon)
        if res == False:
            return False, 'None'
        else:
            ego_center = ego_polygon.centroid
            social_center = polygon.centroid
            vector = np.array((ego_center.x, ego_center.y)) - np.array((social_center.x, social_center.y))
            angle = np.arctan2(vector[1], vector[0])
            angle_diff = angle - ego_heading
            angle_diff = np.mod(angle_diff + np.pi, 2*np.pi) - np.pi
            #front
            if abs(angle_diff) > np.pi - 0.5:
                result = 'front'
            # backward
            elif abs(angle_diff) < 0.5:
                result = 'backward'
            else:
                result = 'side'
            return True, result
        
    
    def _init_info_dict(self):
        self.scene_info_dict['scene_id'] = []
        self.scene_info_dict['scene_name'] = []
        self.scene_info_dict['dataset timestamp'] = []
        self.scene_info_dict['sim timestamp'] = []
        self.scene_info_dict['stop time(ms)'] = []
        self.scene_info_dict['result'] = []
        self.scene_info_dict['collision'] = []
        self.scene_info_dict['collision state'] = []
        self.scene_info_dict['start speed(m/s)'] = []
        self.scene_info_dict['average speed(m/s)'] = []
        self.scene_info_dict['max acc +(m/s2)'] = []
        self.scene_info_dict['max acc -(m/s2)'] = []
        self.scene_info_dict['max jerk(m/s3)'] = []
        self.scene_info_dict['mid state'] = []
        self.scene_info_dict['v_std_dev'] = []
        self.scene_info_dict['a_std_dev'] = []
        self.scene_info_dict['UD'] = []
        # self.scene_info_dict
        
    def add_info_dict(self, ):
        self.scene_info_dict['scene_id'].append(self.data['scene_id'])
        self.scene_info_dict['scene_name'].append(self.data['scene_name'])
        self.scene_info_dict['dataset timestamp'].append(f"0-{self.data['max_frame']}")
        self.scene_info_dict['sim timestamp'].append(f"{self.data['first_frame']}-{self.data['max_frame']}")
        self.scene_info_dict['stop time(ms)'].append(-1)
        self.scene_info_dict['result'].append(self.data['result'])
        self.scene_info_dict['collision'].append(self.data['collision_frame'])
        self.scene_info_dict['collision state'].append(self.data['collision_state'])
        self.scene_info_dict['start speed(m/s)'].append(self.data['ego_speed'][self.data['first_frame']])
        self.scene_info_dict['average speed(m/s)'].append(np.mean(list(self.data['ego_speed'].values())))
        self.scene_info_dict['max acc +(m/s2)'].append(max(list(self.data['ego_acc'].values())))
        self.scene_info_dict['max acc -(m/s2)'].append(min(list(self.data['ego_acc'].values())))
        self.scene_info_dict['max jerk(m/s3)'].append(max(np.abs(list(self.data['ego_jerk'].values()))))
        self.scene_info_dict['mid state'].append('')
        self.scene_info_dict['v_std_dev'].append(self.cal_std_deviation(self.data['ego_speed'].values()))
        self.scene_info_dict['a_std_dev'].append(self.cal_std_deviation(self.data['ego_acc'].values()))
        self.scene_info_dict['UD'].append(self.cal_uncomfortable_deceleration(list(self.data['ego_acc'].values())))
        
    def cal_std_deviation(self, values):
        value_list = list(values)
        values = np.array(value_list)
        return np.std(values)
    
    def cal_uncomfortable_deceleration(self, acc_list):
        # count continuous deceleration < -1.6 as one time
        accs = np.array(acc_list)
        uncomfortable_deceleration_count = 0
        i = 0
        while i < len(accs):
            if accs[i] < -1.6:
                uncomfortable_deceleration_count += 1
                while 1:
                    if i == len(accs)-1:
                        break
                    if accs[i+1] > -1.6:
                        break
                    i += 1
            i += 1
        return uncomfortable_deceleration_count
    
    def save_to_excel(self, saved_name):
                
        # 将数据转换为DataFrame对象
        df = pd.DataFrame(self.scene_info_dict)
        
        # 将DataFrame写入Excel文件
        df.to_excel(saved_name, index=False)
        pass

def get_excel(path):
    case = path.split('/')[-1]
    excel_path = os.path.join(path, case + '_result.xlsx')
    test_excel = Test_get_info()
    files = os.listdir(path)
    json_files = [file for file in files if file.endswith('.json')]
    json_files.sort(key = lambda x: int(x.split('_')[0]))
    for item in json_files:
        print(item)
        json_path = os.path.join(path, item)
        # print(json_path)
        test_excel.read_json(json_path)
        
        test_excel.add_info_dict()
    
    test_excel.save_to_excel(excel_path)

if __name__ == "__main__":
    log_folder_absolute_path = '/home/rancho/2lidelun/Huawei-dataset/LOG/EPSILON_0918_fix_egopath'
    subfolders = []
    for item in os.listdir(log_folder_absolute_path):
        if item == 'test':
            continue
        item_path = os.path.join(log_folder_absolute_path, item)
        if os.path.isdir(item_path):
            subfolders.append(item)
    # print(subfolders)
    
    
    for folder in subfolders:
        # folder = 'test'
        path = os.path.join(log_folder_absolute_path, folder)
        get_excel(path)
        # print(path)
        # break
        
    