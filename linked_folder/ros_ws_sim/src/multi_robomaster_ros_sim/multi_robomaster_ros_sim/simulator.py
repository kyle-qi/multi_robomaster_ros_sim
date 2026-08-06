import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import ColorRGBA
from robomaster_msgs.action import GripperControl
from math import cos, sin, pi
import time
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.patches as patches
import matplotlib.pyplot as plt

class MultiRoboMasterSim(Node):
    def __init__(self):
        super().__init__('multi_robomaster_sim')

        # constants
        # robots
        self.ROBOT_IDS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        self.N = len(self.ROBOT_IDS)
        # time
        self.TIMEOUT_SET_MOBILE_BASE_SPEED = 20 # milliseconds
        self.TIMEOUT_GET_POSES = 10 # milliseconds
        self.TIMEOUT_CHASSIS_SPEED = 500 # milliseconds
        self.DT = (self.TIMEOUT_SET_MOBILE_BASE_SPEED + self.TIMEOUT_GET_POSES) / 1000.
        # robot control
        self.MAX_LINEAR_SPEED = 1.0 # meters / second
        self.MAX_ANGULAR_SPEED = 360 * np.pi / 180 # radians / second
        # dimensions
        self.ENV = [-2., -2., 4., 4.] # (x, y) can vary from (ENV[0], ENV[1]) to (ENV[0]+ENV[2], ENV[1]+ENV[3])
        self.ROBOT_SIZE = [0.24, 0.32] # [w, l]
        self.GRIPPER_SIZE = 0.1
        
        # Gripper / arm constants
        self.GRIPPER_OPEN = 1
        self.GRIPPER_CLOSED = 2
        self.ARM_MAX_EXTENSION = 0.2  # max forward extension in meters
        self.GRIPPER_OPEN_WIDTH = self.GRIPPER_SIZE  # visual width when open
        self.GRIPPER_CLOSED_WIDTH = self.GRIPPER_SIZE * 0.3  # visual width when closed

        # State: [x, y, theta]
        self.states = {}
        self.leds = {}
        self.velocities = {rid: np.array([0.0, 0.0, 0.0]) for rid in self.ROBOT_IDS}
        self.last_cmd_time = {rid: self.get_clock().now() for rid in self.ROBOT_IDS}

        # Gripper and arm state per robot
        self.gripper_states = {rid: self.GRIPPER_OPEN for rid in self.ROBOT_IDS}
        self.arm_positions = {rid: np.array([0.0, 0.0, 0.0]) for rid in self.ROBOT_IDS}  # x, y, z
        
        # Initialize robots randomly
        for i, rid in enumerate(self.ROBOT_IDS):
            x = np.random.uniform(self.ENV[0], self.ENV[0] + self.ENV[2])
            y = np.random.uniform(self.ENV[1], self.ENV[1] + self.ENV[3])
            theta = np.random.random() * 2 * np.pi
            
            self.states[rid] = np.array([x, y, theta])
            self.leds[rid] = np.array([0., 0., 0.])

        # Pubs and Subs
        self.pubs = {}
        self.subs_vel = {}
        self.subs_led = {}
        self.subs_arm = {}
        self.gripper_action_servers = {}
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self._action_group = ReentrantCallbackGroup()

        for rid in self.ROBOT_IDS:
            # Publisher: Mimics VRPN motion capture system
            self.pubs[rid] = self.create_publisher(
                PoseStamped, f'/vrpn_mocap/dji_robot_{rid}/pose', qos)
            
            # Subscriber: Listen to the controller's cmd_vel
            self.subs_vel[rid] = self.create_subscription(
                Twist, f'/robot{rid}/cmd_vel', 
                lambda msg, rid=rid: self.vel_callback(msg, rid), qos)
            
            # Subscriber: Listen to the controller's leds
            self.subs_led[rid] = self.create_subscription(
                ColorRGBA, f'/robot{rid}/leds/color', 
                lambda msg, rid=rid: self.led_callback(msg, rid), qos)

            # Subscriber: Listen to the controller's target arm position
            self.subs_arm[rid] = self.create_subscription(
                Point, f'/robot{rid}/target_arm_position',
                lambda msg, rid=rid: self.arm_callback(msg, rid), qos)

            # Action Server: Gripper control
            self.gripper_action_servers[rid] = ActionServer(
                self,
                GripperControl,
                f'/robot{rid}/gripper',
                lambda goal_handle, rid=rid: self.gripper_execute_callback(goal_handle, rid),
                callback_group=self._action_group)

        self.timer = self.create_timer(self.DT, self.update_and_publish)
        self.get_logger().info(f"Simulator started for robots: {self.ROBOT_IDS}")

        # Plots
        self.figure = []
        self.axes = []
        self.patches_robots = {rid: [] for rid in self.ROBOT_IDS}
        self.patches_grippers = {rid: [] for rid in self.ROBOT_IDS}
        self.text_ids = {rid: [] for rid in self.ROBOT_IDS}
        self.__init_plot()
        self.__update_plot()
    
    def __init_plot(self):
        self.figure, self.axes = plt.subplots()
        p_env = patches.Rectangle(np.array([self.ENV[0], self.ENV[1]]), self.ENV[2], self.ENV[3], edgecolor=(0, 0, 0, 1), fill=False, linewidth=4)
        self.axes.add_patch(p_env)

        for i, rid in enumerate(self.ROBOT_IDS):
            R = np.array([[cos(self.states[rid][2]), -sin(self.states[rid][2])], [sin(self.states[rid][2]), cos(self.states[rid][2])]])
            t = np.array([self.states[rid][0], self.states[rid][1]])
            p_robot = patches.Polygon(t + (np.array([[self.ROBOT_SIZE[1] / 2.0, self.ROBOT_SIZE[0] / 2.0],
                                                     [-self.ROBOT_SIZE[1] / 2.0, self.ROBOT_SIZE[0] / 2.0],
                                                     [-self.ROBOT_SIZE[1] / 2.0, -self.ROBOT_SIZE[0] / 2.0],
                                                     [self.ROBOT_SIZE[1] / 2.0, -self.ROBOT_SIZE[0] / 2.0]]) @ R.T),
                                                     facecolor='k')
            p_gripper = patches.Polygon(t + (np.array([[self.ROBOT_SIZE[1] / 2.0, -self.GRIPPER_SIZE / 2.0],
                                                       [self.ROBOT_SIZE[1] / 2.0, self.GRIPPER_SIZE / 2.0],
                                                       [self.ROBOT_SIZE[1] / 2.0 + self.GRIPPER_SIZE, self.GRIPPER_SIZE / 2.0],
                                                       [self.ROBOT_SIZE[1] / 2.0 + self.GRIPPER_SIZE, 0.8 * self.GRIPPER_SIZE / 2.0],
                                                       [self.ROBOT_SIZE[1] / 2.0, 0.8 * self.GRIPPER_SIZE / 2.0],
                                                       [self.ROBOT_SIZE[1] / 2.0, -0.8 * self.GRIPPER_SIZE / 2.0],
                                                       [self.ROBOT_SIZE[1] / 2.0 + self.GRIPPER_SIZE, -0.8 * self.GRIPPER_SIZE / 2.0],
                                                       [self.ROBOT_SIZE[1] / 2.0 + self.GRIPPER_SIZE, -self.GRIPPER_SIZE / 2.0],
                                                       [self.ROBOT_SIZE[1] / 2.0, -self.GRIPPER_SIZE / 2.0]]) @ R.T),
                                                       facecolor='k')
            text_id = plt.text(self.states[rid][0] + max(self.ROBOT_SIZE) / 2.0, self.states[rid][1] + max(self.ROBOT_SIZE) / 2.0, s=str(self.ROBOT_IDS[i]), color="red")
            self.patches_robots[rid] = p_robot
            self.patches_grippers[rid] = p_gripper
            self.text_ids[rid] = text_id
            self.axes.add_patch(p_robot)
            self.axes.add_patch(p_gripper)
        
        self.axes.set_xlim(self.ENV[0] - max(self.ROBOT_SIZE), self.ENV[0] + self.ENV[2] + max(self.ROBOT_SIZE))
        self.axes.set_xlim(self.ENV[1] - max(self.ROBOT_SIZE), self.ENV[1] + self.ENV[3] + max(self.ROBOT_SIZE))
        self.axes.grid()
        # self.axes.set_axis_off()
        self.axes.axis('equal')

        plt.ion()
        plt.show()
    
    def __update_plot(self):
        for rid in self.ROBOT_IDS:
            R = np.array([[cos(self.states[rid][2]), -sin(self.states[rid][2])], [sin(self.states[rid][2]), cos(self.states[rid][2])]])
            t = np.array([self.states[rid][0], self.states[rid][1]])
            xy_robot = t + (np.array([[self.ROBOT_SIZE[1] / 2.0, self.ROBOT_SIZE[0] / 2.0],
                                      [-self.ROBOT_SIZE[1] / 2.0, self.ROBOT_SIZE[0] / 2.0],
                                      [-self.ROBOT_SIZE[1] / 2.0, -self.ROBOT_SIZE[0] / 2.0],
                                      [self.ROBOT_SIZE[1] / 2.0, -self.ROBOT_SIZE[0] / 2.0]]) @ R.T)

            # Gripper visual depends on arm extension and gripper state
            arm_ext = self.arm_positions[rid][0]  # forward extension from arm x
            gripper_offset = self.ROBOT_SIZE[1] / 2.0 + arm_ext

            if self.gripper_states[rid] == self.GRIPPER_CLOSED:
                # Closed gripper: narrow gap between fingers
                gap = self.GRIPPER_SIZE * 0.1
            else:
                # Open gripper: full gap
                gap = self.GRIPPER_SIZE * 0.8

            half_gap = gap / 2.0
            finger_width = (self.GRIPPER_SIZE - gap) / 2.0

            # Draw a U-shaped gripper with two fingers
            xy_gripper = t + (np.array([
                [gripper_offset, -self.GRIPPER_SIZE / 2.0],
                [gripper_offset, -half_gap],
                [gripper_offset + self.GRIPPER_SIZE, -half_gap],
                [gripper_offset + self.GRIPPER_SIZE, -self.GRIPPER_SIZE / 2.0 - finger_width + self.GRIPPER_SIZE / 2.0],
                [gripper_offset + self.GRIPPER_SIZE, -self.GRIPPER_SIZE / 2.0],
                [gripper_offset, -self.GRIPPER_SIZE / 2.0],
                [gripper_offset, self.GRIPPER_SIZE / 2.0],
                [gripper_offset + self.GRIPPER_SIZE, self.GRIPPER_SIZE / 2.0],
                [gripper_offset + self.GRIPPER_SIZE, half_gap],
                [gripper_offset, half_gap],
                [gripper_offset, -self.GRIPPER_SIZE / 2.0],
            ]) @ R.T)
        
            self.patches_robots[rid].xy = xy_robot
            self.patches_grippers[rid].xy = xy_gripper

            self.patches_robots[rid].set_facecolor(self.leds[rid])

            # Color gripper based on state
            if self.gripper_states[rid] == self.GRIPPER_CLOSED:
                self.patches_grippers[rid].set_facecolor('red')
            else:
                self.patches_grippers[rid].set_facecolor('green')

            self.text_ids[rid].set_position((self.states[rid][0] + max(self.ROBOT_SIZE) / 2.0, self.states[rid][1] + max(self.ROBOT_SIZE) / 2.0))

        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()
    
    @staticmethod
    def transform_velocity_local_to_global(robots_speeds, theta):
        # robots_speeds : list of 3
        # theta : scalar
        robots_speeds_global = [0] * 3
        x_dot = robots_speeds[0]
        y_dot = robots_speeds[1]
        th_dot = robots_speeds[2]
        c_th = cos(theta)
        s_th = sin(theta)
        robots_speeds_global[0] = c_th * x_dot - s_th * y_dot
        robots_speeds_global[1] = s_th * x_dot + c_th * y_dot
        robots_speeds_global[2] = robots_speeds[2]
        return robots_speeds_global

    def vel_callback(self, msg, rid):
        # Store commanded velocities
        robot_speeds = MultiRoboMasterSim.transform_velocity_local_to_global([msg.linear.x, msg.linear.y, msg.angular.z], self.states[rid][2])
        self.velocities[rid] = np.array(robot_speeds)
        self.last_cmd_time[rid] = self.get_clock().now() # Update heartbeat

    def led_callback(self, msg, rid):
        # Store commanded velocities
        self.leds[rid] = np.array([msg.r, msg.g, msg.b])

    def arm_callback(self, msg, rid):
        # Store arm target position (x forward extension, z height — y ignored)
        self.arm_positions[rid] = np.array([msg.x, msg.y, msg.z])
        self.get_logger().debug(f'Robot {rid} arm target: x={msg.x:.3f}, z={msg.z:.3f}')

    def gripper_execute_callback(self, goal_handle, rid):
        """Execute a GripperControl action — simulates gripper open/close."""
        target_state = goal_handle.request.target_state
        power = goal_handle.request.power

        state_str = 'Closed' if target_state == self.GRIPPER_CLOSED else 'Open'
        self.get_logger().info(
            f'Robot {rid} gripper command: {state_str} (power={power:.2f})')

        # Simulate gripper movement delay proportional to power (faster = more power)
        delay = max(0.3, 1.0 - power)  # 0.3s to 1.0s depending on power
        time.sleep(delay)

        # Update gripper state
        self.gripper_states[rid] = target_state

        goal_handle.succeed()
        result = GripperControl.Result()
        self.get_logger().info(f'Robot {rid} gripper now: {state_str}')
        return result
        
    def update_and_publish(self):
        current_time = self.get_clock().now()

        for rid in self.ROBOT_IDS:
            elapsed_time_since_last_command_received = (current_time - self.last_cmd_time[rid]).nanoseconds / 1e9    
            if elapsed_time_since_last_command_received > self.TIMEOUT_CHASSIS_SPEED / 1e3:
                v_cmd = np.array([0.0, 0.0, 0.0])
            else:
                v_cmd = self.velocities[rid]
                
            # Integrate velocity
            # Global X/Y update
            # The controller sends local velocities, which is what the robots are expected to receive.
            # These are then converted to global in the velocity callback in the simulator
            self.states[rid][0] += v_cmd[0] * self.DT
            self.states[rid][1] += v_cmd[1] * self.DT
            self.states[rid][2] += v_cmd[2] * self.DT

            # Create PoseStamped message
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            
            msg.pose.position.x = self.states[rid][0]
            msg.pose.position.y = self.states[rid][1]
            msg.pose.position.z = 0.0
            
            # Euler to Quaternion (simplified for 2D Z-axis rotation)
            half_yaw = self.states[rid][2] * 0.5
            msg.pose.orientation.z = sin(half_yaw)
            msg.pose.orientation.w = cos(half_yaw)
            
            self.pubs[rid].publish(msg)

        self.__update_plot()

def main(args=None):
    rclpy.init(args=args)
    node = MultiRoboMasterSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
