#!/usr/bin/env python

import sys
import time

from tkinter import * 

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import threading

import math
import numpy as np

from autoware_auto_msgs.msg import AdjustXY
import yaml

##################################
##  GETTING CONFIGS FROM YAMLS  ##
##################################

def load_configs():
    global slider_params
    global camera_names_config_file
    global adjust_defaults_config_file

    with open(camera_names_config_file, 'r') as stream:
        slider_params = dict(yaml.safe_load(stream))

    with open(adjust_defaults_config_file, 'r') as stream:
        slider_params.update(dict(yaml.safe_load(stream)))

def get_param(name):
    return slider_params[name]

def get_param_with_default(name, default):
    val = None
    try:
        val = get_param(name)
    except KeyError:
        val = default
    
    return val


def save_to_file():

    global dump_defaults_file
    global adjust_defaults

    dicto = {'adjust_defaults' : adjust_defaults}
    with open(dump_defaults_file, 'w') as file:
        yaml.dump(dicto, file)
    

def change_val_by_id(id, name, axis):
    def set_val(val):
        list_of_xy[id][axis] = val
        adjust_defaults[name][axis] = val

    return set_val

def change_val(scale, change):
    def change_scale():
        scale.set(scale.get() + change)

    return change_scale

def talker():
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        data_x = list()
        data_y = list()

        for point in list_of_xy:
            data_x.append(int(point[0]))
            data_y.append(int(point[1]))

        data = AdjustXY()
        data.x = data_x
        data.y = data_y

        pub.publish(data)
        rate.sleep()

def scaler():
    if (list_of_xy == None):
        raise AssertionError("List of XY is empty")
    
    parent_widget = Tk(className="AdjustXY")

    for i in range(scale_count):
        for shift in range(1, 4, 2):
            axis = 'x' if shift // 2 == 0 else 'y'

            short_camera_name = camera_name_list[i]

            lab = Label(parent_widget, text="{} {}".format(short_camera_name, axis), font="Arial 14")
            lab.grid(row = 4*i + shift - 1, column=1)

            scale_widget = Scale(parent_widget, from_=bounds[0], to=bounds[1],
                                        orient=HORIZONTAL,
                                        command = change_val_by_id(i, short_camera_name, shift // 2))
            
            default = adjust_defaults[short_camera_name][shift // 2]
            scale_widget.set(default)
            scale_widget.grid(column=1, row=4*i + shift)

            btn_column_1 = Button(parent_widget, text="-", command=change_val(scale_widget, -1))
            btn_column_1.grid(column=0, row=4*i + shift)

            btn_column_2 = Button(parent_widget, text="+", command=change_val(scale_widget, 1))
            btn_column_2.grid(column=2, row=4*i + shift)

    save_button = Button(parent_widget, text="save", command=save_to_file)
    save_button.grid(column=2, row=4*scale_count)
    
    mainloop()

slider_params = None
camera_names_config_file = str()
adjust_defaults_config_file = str()

list_of_xy = None
camera_name_list = None
scale_count = None
bounds = None
dump_defaults_file = None
adjust_defaults = None
pub = None

class AdjustPublisher(Node):
    def __init__(self):
        super().__init__('adjust_xy_distributor')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('default_dumps_file', None),
                ('camera_names_config_file', None),
                ('adjust_defaults_config_file', None)
            ]
        )

        global dump_defaults_file
        global camera_names_config_file
        global adjust_defaults_config_file

        dump_defaults_file = self.get_parameter('default_dumps_file').get_parameter_value().string_value
        self.get_logger().info("dump_defaults_file: {}".format(dump_defaults_file))
        camera_names_config_file = self.get_parameter('camera_names_config_file').get_parameter_value().string_value
        self.get_logger().info("camera_names_config_file: {}".format(camera_names_config_file))
        adjust_defaults_config_file = self.get_parameter('adjust_defaults_config_file').get_parameter_value().string_value
        self.get_logger().info("adjust_defaults_config_file: {}".format(adjust_defaults_config_file))

        load_configs()

        global slider_params

        self.get_logger().debug("Slider params: {}".format(slider_params))

        try:
            global camera_name_list
            global scale_count
            global bounds
            global adjust_defaults

            camera_name_list = get_param('camera_name_list')
            scale_count = len(camera_name_list)

            bounds = get_param_with_default('adjust_bounds', [0, 0])
            adjust_defaults = get_param('adjust_defaults')

        except Exception as e:
            rclpy.logging.get_logger().debug("Failed to obtain parameters: {}".format(e))
            raise rclpy.exceptions.ParameterException("Failed to obtain parameters from Parameter Server")
            
        print("CAMNAME: {}".format(camera_name_list))
        global list_of_xy
        
        list_of_xy = list()
        for i in range(scale_count):
            list_of_xy.append([0, 0])

        self.publisher_ = self.create_publisher(AdjustXY, '/config/adjust_xy', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        data_x = list()
        data_y = list()

        for point in list_of_xy:
            data_x.append(int(point[0]))
            data_y.append(int(point[1]))

        data = AdjustXY()
        data.x = data_x
        data.y = data_y

        self.publisher_.publish(data)
        self.get_logger().debug('Publishing: "%s"' % data)


# The following is just to start the node
def main(args=None):
    rclpy.init(args=args)

    try:
        node = AdjustPublisher()
    except yaml.YAMLError as exc:
        print("YAML Parsing error: {}".format(exc))
        node.destroy_node()
        rclpy.shutdown()
    
    thread = threading.Thread(target=scaler, args=[])
    thread.start()
    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()