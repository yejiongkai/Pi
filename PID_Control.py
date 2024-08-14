import numpy as np


class Gesture_PID_Control:
    def __init__(self, video_width, video_height):
        self.video_width = video_width
        self.video_height = video_height

        # PID 控制参数
        self.kp = 0.1
        self.ki = 0.01
        self.kd = 0

        # PID 控制变量
        self.integral = [0, 0]
        self.last_error = [0, 0]
        self.delta_output = [0, 0]  # 用于增量式PID

        #其他
        self.max_ratio = 5
        self.dead_space = 90
        self.min_ratio = 2 

        # 初始化目标中心坐标为相机中心
        self.last_object_center = [self.video_width / 2, self.video_height / 2]
        self.camera_center = [self.video_width / 2, self.video_height / 2]

    def pos_pid_control(self, camera_center, target_center, integral, last_error):
        error = [target_center[i] - camera_center[i] for i in range(2)]
        integral = [integral[i] + error[i] for i in range(2)]
        derivative = [error[i] - last_error[i] for i in range(2)]
        output = [self.kp * error[i] + self.ki * integral[i] + self.kd * derivative[i] for i in range(2)]
        return output, integral, error

    # 写了 没用
    def incremental_pid_control(self, camera_center, target_center, integral, last_error, delta_output):
        error = [target_center[i] - camera_center[i] for i in range(2)]
        integral = [integral[i] + error[i] for i in range(2)]
        derivative = [error[i] - last_error[i] for i in range(2)]
        delta_output = [self.kp * (error[i] - last_error[i]) + self.ki * error[i] + self.kd * (derivative[i]) for i in
                        range(2)]
        output = [delta_output[i] + delta_output[i] for i in range(2)]
        return output, integral, error, delta_output

    def get_Control_Value(self, is_detect, object_x, object_y):
        if not is_detect:
            object_center = self.last_object_center
        else:
            object_center = [object_x, object_y]
            self.last_object_center = object_center
            
        print(object_center, self.camera_center)

        if np.linalg.norm(np.array(object_center)-np.array(self.camera_center)) < self.dead_space:
            return 0, 0

        else:
            outputs, self.integral, self.last_error,self.delta_output  = self.incremental_pid_control(self.camera_center,
                                                                           object_center,
                                                                           self.integral,
                                                                           self.last_error,
                                                                           self.delta_output)
            # 将PID输出转换为舵机控制角度
            degree_per_pixel_v = 300 / self.video_height  # 每像素对应的角度--肩部--对应纵
            degree_per_pixel_h = 300 / self.video_width  # 每像素对应的角度--腰部--对应横]
            
            value_v = outputs[1] * degree_per_pixel_v
            value_h = outputs[0] * degree_per_pixel_h
            
            if value_v > 0:
                value_v = min(self.max_ratio, max(self.min_ratio, outputs[1] * degree_per_pixel_v))  # 肩部舵机角度计算
            else:
                value_v = min(-self.min_ratio, max(-self.max_ratio, outputs[1] * degree_per_pixel_v))  # 肩部舵机角度计算
                
            if value_h > 0:
                value_h = min(self.max_ratio, max(self.min_ratio, outputs[0] * degree_per_pixel_h))  # 腰部舵机角度计算
            else:
                value_h = min(-self.min_ratio, max(-self.max_ratio, outputs[0] * degree_per_pixel_h))  # 腰部舵机角度计算
                

            #value_v = min(self.max_ratio, max(-self.max_ratio, outputs[1] * degree_per_pixel_v))  # 肩部舵机角度计算
            #value_h = min(self.max_ratio, max(-self.max_ratio, outputs[0] * degree_per_pixel_h))  # 腰部舵机角度计算
            
            
            

            return value_v, value_h


