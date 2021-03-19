import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion

import math
import numpy as np

from pyquaternion import Quaternion as pyqQuaternion

'''
 
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

# Calculates Rotation Matrix given euler angles.
def eulerAnglesToRotationMatrix(theta) :
    
    R_x = np.array([
        [1,         0,                   0                   ],
        [0,         math.cos(theta[0]), -math.sin(theta[0])  ],
        [0,         math.sin(theta[0]),  math.cos(theta[0])  ]
    ])
        
        
                    
    R_y = np.array([
        [ math.cos(theta[1]),    0,      math.sin(theta[1])  ],
        [ 0,                     1,      0                   ],
        [-math.sin(theta[1]),    0,      math.cos(theta[1])  ]
    ])
                
    R_z = np.array([
        [math.cos(theta[2]),    -math.sin(theta[2]),     0],
        [math.sin(theta[2]),     math.cos(theta[2]),     0],
        [0,                      0,                      1]
    ])
                    
                    
    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R

def full_transform_matrix(pose, orientation):
    M = eulerAnglesToRotationMatrix(orientation)
    return np.array(
        [
            [M[0][0], M[0][1], M[0][2], pose[0]],
            [M[1][0], M[1][1], M[1][2], pose[1]],
            [M[2][0], M[2][1], M[2][2], pose[2]],
            [      0,       0,       0,       1]
        ]
    )

'''

def normalize(quaternion):
    norm = math.sqrt(
        quaternion.x ** 2 + 
        quaternion.y ** 2 + 
        quaternion.z ** 2 + 
        quaternion.w ** 2
    )

    qw = Quaternion()
    qw.x = quaternion.x / norm
    qw.y = quaternion.y / norm
    qw.z = quaternion.z / norm
    qw.w = quaternion.w / norm

    return qw


tf_msg = TFMessage()

class TFPublisher(Node):

    def __init__(self):
        super().__init__('tf_fixed_publisher')
        self.publisher_ = self.create_publisher(TFMessage, '/tf', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global tf_msg

        self.publisher_.publish(tf_msg)
        self.get_logger().info('Publishing: "%s"' % tf_msg)


class OdometrySubscriber(Node):

    def __init__(self):
        super().__init__('odometry_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/lgsvl/gnss_odom',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: Odometry):
        global tf_msg 

       #self.get_logger().info('I heard: "%s"' % (msg))

        #s = str(msg.pose) + " " + str(msg.twist)
        """
        position_v = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y, 
            msg.pose.pose.position.z
        ]
        orientation_v = list(
            euler_from_quaternion(
                msg.pose.pose.orientation.x, 
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            )
        )
        """
        ts = self.get_clock().now().to_msg()

        stamped_tf = TransformStamped()
        stamped_tf.header.stamp = ts
        stamped_tf.header.frame_id = "map"
        stamped_tf.child_frame_id = "base_link"
        
        position_v3 = Vector3()
        position_v3.x = msg.pose.pose.position.x
        position_v3.y = msg.pose.pose.position.y
        position_v3.z = msg.pose.pose.position.z

        stamped_tf.transform.translation = position_v3
        stamped_tf.transform.rotation = normalize(msg.pose.pose.orientation)

        stamped_tf2 = TransformStamped()
        stamped_tf2.header.stamp = ts
        stamped_tf2.header.frame_id = "earth"
        stamped_tf2.child_frame_id = "map"
        
        position_v3 = Vector3()
        position_v3.x = -2686535.79222737
        position_v3.y = -4307199.67675597
        position_v3.z = 3848553.85931299

        rot = Quaternion()
        rot_matrix = np.array([
            [ 0.84487019,  0.33047596, -0.42068991],
            [-0.53489876,  0.50888832, -0.6744746 ],
            [-0.00881346,  0.79487,     0.60671575]
        ])
        util_quat = pyqQuaternion(matrix=rot_matrix)

        rot.x = util_quat.x
        rot.y = util_quat.y
        rot.z = util_quat.z
        rot.w = util_quat.w
        

        stamped_tf2.transform.translation = position_v3
        stamped_tf2.transform.rotation = rot
        tf_msg.transforms = [
            stamped_tf, 
            stamped_tf2
        ]

        #self.get_logger().info('I heard: "%s"' % (msg))


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = OdometrySubscriber()
    minimal_publisher = TFPublisher()

    while (rclpy.ok()):
        rclpy.spin_once(minimal_subscriber)
        rclpy.spin_once(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()