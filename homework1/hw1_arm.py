import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from matplotlib import pyplot as plt
import os 
import sympy as sp

from time import clock_gettime_ns, CLOCK_MONOTONIC, sleep

from std_msgs.msg import String


class DiffDriveControl(Node):

    def __init__(self, v = 0.0, w = 0.0, w_l = 0.0, w_r = 0.0):
        super().__init__('diff_drive_control')
        
        # Constants
        self.T_sample = 0.033
        self.r = 0.04
        self.L = 0.08
        self.t_end = 5 * 1e9

        self.publisher = self.create_publisher(Twist, '/hagen/cmd_vel', 30)
        self.odometry = self.create_subscription(Odometry, "/hagen/odom", self.set_pose, 20) 
        # self.subscription = self.create_subscription(Twist, '/hagen/cmd_vel', self.listener_callback, 10)
        
        self.timer = self.create_timer(self.T_sample, self.timer_callback)

        if w_l != 0 or w_r != 0:            
            self.v = (w_r * self.r + w_l * self.r) / 2
            self.w = (w_r * self.r - w_l * self.r) / self.L
        else:
            self.v = v
            self.w = w

        self.q = np.zeros(3)

        self.q_history = []
        self.t_history = []

        self.t_init = clock_gettime_ns(CLOCK_MONOTONIC)

        self.is_working = True
        
    

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


    def set_pose(self, msg):
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.q = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]) 
        
        self.q_history.append(self.q)
        self.t_history.append(clock_gettime_ns(CLOCK_MONOTONIC) - self.t_init)


    def timer_callback(self):
        msg = Twist()
        
        if self.t_end < clock_gettime_ns(CLOCK_MONOTONIC) - self.t_init:
            self.is_working = False
            self.v = 0.0
            self.w = 0.0
            self.timer.cancel()

        msg.linear.x = self.v
        msg.angular.z = self.w

        self.publisher.publish(msg)


def wrap_to_pi(x):
    x = np.array([x])
    xwrap = np.remainder(x, 2*np.pi)
    mask = np.abs(xwrap)>np.pi
    xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
    return xwrap[0]


def kinematics_solution(v = 0.0, w = 0.0, w_l = 0.0, w_r = 0.0, t_span = None):
    r = 0.04
    L = 0.08
    
    if w_l != 0 or w_r != 0:            
        v = np.round((w_r * r + w_l * r) / 2, 3)
        w = np.round((w_r * r - w_l * r) / L, 3)

    t = sp.symbols("t")
    v_vec = sp.Matrix([v*sp.cos(w*t), v*sp.sin(w*t), 0])
    
    x = sp.integrate(v_vec[0], t)
    y = sp.integrate(v_vec[1], t)

    theta = w*t
    
    x_range = sp.lambdify(t, x, "numpy")(t_span) if x != 0 else np.zeros(t_span.shape)
    x_range -= x_range[0]
    
    y_range = sp.lambdify(t, y, "numpy")(t_span) if y != 0 else np.zeros(t_span.shape)
    y_range -= y_range[0]
    
    theta_range = wrap_to_pi( 
        sp.lambdify(t, theta, "numpy")(t_span) if theta != 0 else np.zeros(t_span.shape)
    )

    
    return np.array([x_range, y_range, theta_range])


def main(args=None):
    r = 0.04
    L = 0.08
    
    sim_parameters = (
        (0.5, 0.0, 0.0, 0.0),
        (1.0, 2.0, 0.0, 0.0),
        (0.0, 2.0, 0.0, 0.0),
        (0.0, 0.0, 20.0, 18.0)
    )          

    fig, axs = plt.subplots(4, 3, figsize=(16, 16))

    coords_names = ("x", "y", "theta")
    
    rclpy.init(args=args)

    for i in range(4):
        minimal_publisher = DiffDriveControl(*(sim_parameters[i]))

        while minimal_publisher.is_working:
            rclpy.spin_once(minimal_publisher)
        
        minimal_publisher.destroy_node()
        os.system("ros2 service call /reset_simulation std_srvs/srv/Empty")
        sleep(0.1)

        t_span = np.array(minimal_publisher.t_history) / 1e9
        
        q_sol = kinematics_solution(*(sim_parameters[i]), t_span)
        q_sim = np.array(minimal_publisher.q_history)
        
        for k in range(3):    
            axs[i][k].plot(t_span, q_sim[:, k])
            axs[i][k].plot(t_span, q_sol[k, :])
            axs[i][k].set_xlabel("t, s")
            axs[i][k].set_ylabel(coords_names[k])
            axs[i][k].legend(["simulation", "solution"])
            if sim_parameters[i][2] * sim_parameters[i][3] != 0:    
                w_l, w_r = sim_parameters[i][2:4]
                v = np.round((w_r * r + w_l * r) / 2, 3)
                w = np.round((w_r * r - w_l * r) / L, 3)
                axs[i][k].set_title(f"v = {v}, w = {w}, coordinate: {coords_names[k]}")
            else:
                axs[i][k].set_title(f"v = {sim_parameters[i][0]}, w = {sim_parameters[i][1]}, coordinate: {coords_names[k]}")
        
    plt.tight_layout()
    plt.savefig("hw1_arm.png")
    rclpy.shutdown()


if __name__ == '__main__':
    main()