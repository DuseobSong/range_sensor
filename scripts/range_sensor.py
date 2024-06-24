#!/usr/bin/env python3

import rospy
import os, sys, json

import rospkg

from sensor_msgs.msg import Range
from range_sensor.msg import RangeDetection
from std_msgs.msg import Bool, UInt8

ERR_NO_ERROR = 0
ERR_MAL_FUNCTION = 1
ERR_ERROR_CODE_LOADING_FAILED = 2
ERR_TOPIC_LIST_LOADING_FAILED = 3

DEFAULT_RANGE_THRESHOLD = 0.8 # [ m ]

CAM_DIR_LEFT = 1
CAM_DIR_RIGHT = 3

STATE_PAUSE = 1
STATE_MOVE = 3

RADIATION_SOUND = 0
RADIATION_IR = 1
RADIATION_ETC = 2

EMPTY_SPACE_DETECTION_COUNT_THRESHOLD = 20 # 0.2m by 100Hz
DEAD_END_COUNT_THRESHOLD = 1000 # 10 sec 
NOISE_CNT_THRESHOLD = 20


class RangeDetection:
    def __init__(self, 
                 error_code_list_dir, 
                 topic_list_dir,
                 cam_dir=CAM_DIR_LEFT,
                 range_threshold=DEFAULT_RANGE_THRESHOLD):
        self.rate = rospy.Rate(100)
        
        self.error_code_list_dir = error_code_list_dir
        self.topic_list_dir = topic_list_dir
        self.load_error_code_list()
        self.load_topic_list()
        
        self.cam_dir = cam_dir
        self.range_threshold = range_threshold
        
        self.error_code_list = None
        self.topic_list = None
        
        self.err_code = ERR_NO_ERROR
    
        self.operation = True
        self.activation = False
        self.pause = False
        
        self.cam_dir = CAM_DIR_LEFT
        
        self.blocked_space_detected = False
        self.empty_space_detected = False
        self.empty_space_detection_count = 0
        self.noise_cnt = 0
        
        self.detection_state = False
        
        self.sensor_radiation_type_left = None
        self.snesor_radiation_type_right = None
        self.range_left = None
        self.range_right = None
        
        self.sensor_connection_left = False
        self.sensor_connection_right = False
        self.sensor_connection_count = 0
        self.connect_count_limit = 100 # sensor working frequency : 100 Hz
        
        self.range_detection_msg = None
        
        self.lp_capture_status = False
        
        # Dead-end count
        self.dead_end_count = 0
        
        self.activate_topic = self.topic_list["activate_range_sensor"]["NAME"]
        self.range_sensor_left_topic = self.topic_list["range_sensor_left_topic"]["NAME"]
        self.range_sensor_right_topic = self.topic_list["range_sensor_right_topic"]["NAME"]
        self.range_detection_topic = self.topic_list["range_detection_topic"]["NAME"]
        self.camera_direction_topic = self.topic_list["camera_direction_topic"]["NAME"]
        self.lp_capture_status_topic = self.topic_list["lp_capture_status_topic"]["NAME"]
        self.lp_capture_start_topic = self.topic_list["lp_capture_start_topic"]["NAME"]
        
        self.activate_pub = rospy.Subscriber(self.activate_topic, Bool, self.activation_callback)
        self.range_sensor_left_sub = rospy.Subscriber(self.range_sensor_left_topic, Range, self.range_sensor_left_callback)
        self.range_sensor_right_sub = rospy.Subscriber(self.range_sensor_right_topic, Range, self.range_sensor_right_callback)
        self.camera_direction_sub = rospy.Subscriber(self.camera_direction_topic, UInt8, self.camera_direction_callback)
        self.lp_capture_status_pub = rospy.Subscriber(self.lp_capture_status_topic, Bool, self.lp_capture_status_callback)
        
        self.range_detection_pub = rospy.Publisher(self.range_detection_topic, RangeDetection, queue_size=2)
        self.lp_capture_start_pub = rospy.Publisher(self.lp_capture_start_topic, Bool, queue_size=2)
    
    def reset_flag(self):
        self.blocked_space_detected = False
        self.empty_space_detected = False
        self.empty_space_detection_count = 0
        self.noise_cnt = 0
    
    def load_error_code_list(self):
        try:
            with open(self.error_code_list_dir) as f:
                self.error_code_list = json.loads(f.read())
        except:
            print('[RANGE SENSOR] Failed to read error code list.')
            exit(ERR_ERROR_CODE_LOADING_FAILED)
            
    def load_topic_list(self):
        try:
            with open(self.topic_list_dir) as f:
                self.topic_list = json.loads(f.read())
        except:
            self.raise_error(ERR_TOPIC_LIST_LOADING_FAILED)
                
    def raise_error(self, error_code, exception=None):
        self.err_code = error_code 
        if exception is None:
            print('Error code: {} - {}'.format(error_code, self.error_code_list["{}".format(error_code)]))
        else:
            print('Error code: {} - {}'.format(error_code, exception))
        self.operation = False
    
    def activation_callback(self, msg):
        self.activation = msg.data
    
    def range_sensor_left_callback(self, msg: Range):
        self.sensor_radiation_type_left = msg.radiation_type
        self.range_left = msg.range
        self.sensor_connection_left = True
    
    def range_sensor_right_callback(self, msg: Range):
        self.sensor_radiation_type_right = msg.radiation_type
        self.range_right = msg.range
        self.sensor_connection_right = True
        
    def camera_direction_callback(self, msg: UInt8):
        self.cam_dir = msg.data
        
    def lp_capture_status_callback(self, msg: Bool):
        self.lp_capture_status = msg.data
        
        if not self.lp_capture_status:
            self.blocked_space_detected = False
            self.empty_space_detected = False
        
    def publish_detection_msg(self):
        msg = RangeDetection()
        msg.header.stamp = rospy.Time.now()
        msg.radiation_type_left = self.sensor_radiation_type_left
        msg.radiation_type_right = self.sensor_radiation_type_right
        msg.range_left = self.range_left
        msg.range_right = self.range_right
        msg.camera_direction = self.cam_dir
        msg.state = self.detection_state
        
        self.range_detection_pub.publish(msg)
    
    def start_lp_capture(self):
        self.lp_capture_status = True
        self.pause = True
        
        self.reset_flag()
        # self.empty_space_detected = False
        # self.blocked_space_detected = False
        # self.empty_space_detection_count = 0
        
        msg = Bool()
        msg.data = self.lp_capture_status
        self.lp_capture_start_pub.publish(msg)
    
    def range_filter(self):
        cur_range_val = 0.0
        detection_flag = False
        
        if cam_dir == CAM_DIR_LEFT:
            cur_range_val = self.range_left
        elif cam_dir == CAM_DIR_RIGHT:
            cur_range_val = self.range_right
        
        if cur_range_val <= self.range_threshold:
            detection_flag = True
        
        if not self.pause:    
            if detection_flag:
                if self.blocked_space_detected:
                    if self.empty_space_detected: 
                        if self.noise_cnt < NOISE_CNT_THRESHOLD:
                            self.noise_cnt += 1
                        else:
                            self.noise_cnt = 0
                            self.empty_space_detected = False
                    elif not self.empty_space_detected:
                        # robot is beside a car
                        pass
                elif not self.blocked_space_detected:
                    # Car detection started
                    self.blocked_space_detected = True
            elif not detection_flag:
                if self.blocked_space_detected:
                    if self.empty_space_detected:
                        # start counting: empty space between cars or end of parking space
                        # Robot moves forward a little bit and stops, so that the robot can take picture 
                        # of license plate.
                        self.empty_space_detection_count += 1
                        if self.empty_space_detection_count == EMPTY_SPACE_DETECTION_COUNT_THRESHOLD:
                            self.start_lp_capture()
                            
                    elif not self.empty_space_detected:
                        # robot entered into area beside empty space between cars or end of parking space
                        self.empty_space_detected = True
                        self.empty_space_detection_count += 1
                        
                elif not self.blocked_space_detected:
                    # robot entering into parking space or continue driving after license plate capture
                    pass        
    
    def check_detection(self):
        if not self.sensor_connection_left or not self.sensor_connection_right:
            self.sensor_connection_count += 1
            if self.sensor_connection_count >= self.connect_count_limit:
                self.operation = False
                if not self.sensor_connection_left and not self.sensor_connection_right:
                    self.raise_error(13)
                elif not self.sensor_connection_left and self.sensor_connection_right:
                    self.raise_error(11)
                elif self.sensor_connection_left and not self.sensor_connection_right:
                    self.raise_error(12)
        else:
            self.range_filter()
            self.publish_detection_msg()
            self.sensor_connection_left = False
            self.sensor_connection_right = False
            self.sensor_connection_count = 0

    def run(self):
        try:
            while self.operation:
                # To reduce computing resource uasge, this node works only by 
                # drive-control-by-side-obstacle-detection-mode
                if self.activation:
                    self.check_detection()
                else:
                    pass
                self.rate.sleep()
        except Exception as e:
            self.raise_error(1, e)
        finally:
            return self.err_code
    

if __name__ == "__main__":
    rospy.init_node("~", anonymous=True)
    rp = rospkg.RosPack()
    root_dir = rp.get_path("line_following_ros")
    error_code_list = root_dir + "/config/error_code_list.json"
    topic_list_dir = root_dir + "/config/topic_list.json"
    
    cam_dir = CAM_DIR_LEFT
    range_threshold = DEFAULT_RANGE_THRESHOLD
    
    if rospy.has_param("camera_direction"):
        tmp_string = rospy.get_param("camera_direction")
        if tmp_string == "left":
            cam_dir = CAM_DIR_LEFT
        elif tmp_string == "right":
            cam_dir = CAM_DIR_RIGHT
        else:
            print("[RANGE DETECTION] Invalid camera direction: {}\r\nSet to default camera direction - LEFT".format(tmp_string))
            cam_dir = CAM_DIR_LEFT
    
    if rospy.has_param("range_threshold"):
        tmp_val = float(rospy.get_param("range_threshold"))
        if tmp_val >= 1.5:
            print("[RANGE DETECTION] Range threshold exceeds 1.5m. It will be set to 1.5m.")
            range_threshold = 1.5
        elif tmp_val <= 0.3: 
            print("[RANGE DETECTION] Range threshold should be greater than 0.3m. It will be set to 0.3m.")
            range_threshold = 0.3
        else:
            range_threshold = tmp_val
            
        
    node = RangeDetection(
        error_code_list_dir=error_code_list,
        topic_list_dir=topic_list_dir,
        cam_dir=cam_dir,
        range_threshold=range_threshold
    )
    
    node.run()
    
    