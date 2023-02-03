import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time 
import numpy as np 

class ControlStrategy(Node):
    def __init__(self, delta_t,):
        super().__init__('control_strategy')
        self.publisher_ = self.create_publisher(Twist, '/hagen/cmd_vel', 30)
        self.vel_sub = self.create_subscription(Twist, '/hagen/cmd_vel', self.listener_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/hagen/odom", self.set_pose, 20)
        self.vel_sub
        self.odom_sub
        self.i = 0
        self.set_q_init = None
        self.q = None 
        self.r = 0.3 # Wheel radius
        self.L = 1.25 # Axle length
        self.D = 0.07 # Distance between the front front whell and rear axle
        self.Ts = delta_t # Sampling time
        self.t = np.arange(0, 10, self.Ts) # Simulation time
        self.end_controller = False
        self.timer = self.create_timer(self.Ts, self.timer_callback)


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


    def listener_callback(self, msg):
        pass


    def stop_vehicle(self, ):
        self.send_vel(0.0, 0.0)    
    

    def wrap_to_pi(self, x):
        x = np.array([x])
        xwrap = np.remainder(x, 2*np.pi)
        mask = np.abs(xwrap)>np.pi
        xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
        return xwrap[0]
    

    def diff_drive_init(self, duration=10):
        self.duration = duration
        self.wL = 12 # Left wheel velocity
        self.wR = 12 # Right wheel velocity
        self.time_utilized = 0.0
        

    def inter_point_diff_drive_init(self, duration=30, r_distance=1, refPose=np.array([3,6,np.pi/6]), k_p=0.5, k_w=0.9, dmin=0.2):        
        self.duration = duration        # Duration of motion
        self.r_distance = r_distance    # Distance (radius) between the reference and intermediate points
        self.refPose = refPose          # Reference pose
        self.k_p = k_p                  # Proportional gain for the distance error
        self.k_w = k_w                  # Proportional gain for the orientation error
        self.dmin = dmin                # Tolerance for intermediate point
        self.time_utilized = 0.0        # Time utilized for the motion

        self.interPose = np.array([       # Intermediate point pose
            self.refPose[0] - self.r_distance*np.cos(self.refPose[2]),
            self.refPose[1] - self.r_distance*np.sin(self.refPose[2]),
            0
        ])

        self.approach_ref = False
            

    def inter_point_diff_drive(self, ):
        if(self.q is not None):
            # Calculating distance to the reference
            self.D = np.sqrt((self.q[0]-self.refPose[0])**2 + (self.q[1]-self.refPose[1])**2)

            if self.duration < self.time_utilized or self.D < self.dmin:
                self.stop_vehicle()
                print(f"End of simulation, angle is {self.q[2]}")
                self.end_controller = True


            self.theta_r = np.arctan2(self.refPose[1]-self.q[1], self.refPose[0]-self.q[0])
            self.alpha = self.theta_r - self.refPose[2]
            self.beta = np.arctan2((-1)**(self.alpha <= 0) * self.r_distance, self.D)

            self.err_Phi = self.theta_r - self.q[2] + np.minimum(np.abs(self.alpha), np.abs(self.beta))
            
            v = self.k_p*self.D
            w = self.k_w*self.err_Phi

            print("Distance to the goal: ", self.D)
            
            # dq = np.array([v*np.cos(self.q[2]+self.Ts*w/2), v*np.sin(self.q[2]+self.Ts*w/2), w])
            # self.q = self.q + self.Ts*dq # Integration
            # self.q[2] = self.wrap_to_pi(self.q[2]) # Map orientation angle to [-pi, pi]
            
            self.send_vel(v, w)
            self.time_utilized  =  self.time_utilized + self.Ts 


    def perform_action_diff_drive_one_step(self):
        v = self.r/2*(self.wR+self.wL) # Robot velocity
        w = self.r/self.L*(self.wR-self.wL) # Robot angular velocity
        dq = np.array([v*np.cos(self.q[2]+self.Ts*w/2), v*np.sin(self.q[2]+self.Ts*w/2), w])
        self.q = self.q + self.Ts*dq # Integration
        self.q[2] = self.wrap_to_pi(self.q[2]) # Map orientation angle to [-pi, pi]
        self.send_vel(v, w)
        # rate.sleep()
        # time.sleep(self.Ts)
        self.time_utilized  =  self.time_utilized + self.Ts


    def timer_callback(self, ):
        # self.perform_action_diff_drive_one_step()
        self.inter_point_diff_drive()
        return 


    def set_pose(self, msg):
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        print(f"Angle is pi / {(np.pi / yaw)}")
        # if(self.set_q_init is None):
        self.set_q_init = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])
        self.q = self.set_q_init


    def send_vel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    control_strategy = ControlStrategy(delta_t=0.03)
    # control_strategy.diff_drive_init()
    control_strategy.inter_point_diff_drive_init()
    while control_strategy.end_controller is False and rclpy.ok():
        try:
            rclpy.spin_once(control_strategy)
        except KeyboardInterrupt:
            break
    control_strategy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# To reset ros2 service call /reset_simulation std_srvs/srv/Empty