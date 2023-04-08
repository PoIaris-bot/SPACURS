#!/usr/bin/env python
import tf
import rospy
import numpy as np
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.msg import RCIn, State, ActuatorControl
from nav_msgs.msg import Odometry
from tools import remap_angle, constraint, ParticleSwarmOptimizer, PIDController


def compute_path_param(param, x, y, z, theta, k, r_min, search=False):
    param = param.reshape(-1, 2)
    r_ep = param[:, 0].reshape(-1, 1)
    theta_ep = param[:, 1].reshape(-1, 1)

    xo1 = x + r_min * np.cos(theta - np.pi / 2)
    yo1 = y + r_min * np.sin(theta - np.pi / 2)

    xo2 = (r_ep - r_min) * np.cos(theta_ep)
    yo2 = (r_ep - r_min) * np.sin(theta_ep)

    xbc = xo2 - xo1
    ybc = yo2 - yo1

    angle_bc = np.zeros_like(r_ep)

    idx = np.where(xbc == 0)[0].tolist()
    if len(idx):
        angle_bc[idx, 0] = np.sign(ybc[idx, 0]) * np.pi / 2
    idx = np.where(xbc != 0)[0].tolist()
    if len(idx):
        angle_bc[idx, 0] = ((1 - np.sign(xbc[idx, 0])) / 2) * np.sign(ybc[idx, 0]) * np.pi + np.arctan(
            ybc[idx, 0] / xbc[idx, 0])

    beta1 = remap_angle(-(angle_bc - theta), "0_2pi")
    beta2 = remap_angle(-(theta_ep - angle_bc - np.pi / 2), "0_2pi")
    beta3 = remap_angle(-(np.arcsin(r_min / (r_ep - r_min)) - theta_ep), "0_2pi")
    beta4 = np.arcsin(r_min / (r_ep - r_min)) + np.pi / 2

    if search:
        return abs(r_min * (beta1 + beta2 + beta4) + r_ep * beta3 + np.sqrt(xbc ** 2 + ybc ** 2) + np.sqrt(
            (r_ep - 2 * r_min) * r_ep) - z * k)
    else:
        return list(map(float, [xo1, yo1, xo2, yo2, xbc, ybc, beta1, beta2, beta3, beta4]))


def parafoil_navigator(x, y, z, theta, x_goal, y_goal, z_goal, k=3, r_min=5, r_max=50, num_particle=50, max_iter=1000):
    def fitness_func(param):
        return compute_path_param(param, x - x_goal, y - y_goal, z - z_goal, theta, k, r_min, search=True)

    dim = 2
    lower_bound = [2 * r_min, -np.pi]
    upper_bound = [r_max, np.pi]
    pso = ParticleSwarmOptimizer(dim, num_particle, lower_bound, upper_bound, max_iter, fitness_func)
    parameter, _ = pso.run()

    r_ep = parameter[0]
    theta_ep = parameter[1]
    xo1, yo1, xo2, yo2, xbc, ybc, beta1, beta2, beta3, beta4 = compute_path_param(parameter, x, y, z, theta, k, r_min,
                                                                                  search=False)
    delta_z = 0.5
    n1 = round(beta1 * r_min / k / delta_z)
    gamma = theta + np.pi / 2
    gamma1 = np.linspace(gamma - beta1, gamma, n1)
    x1 = xo1 + r_min * np.cos(gamma1)
    y1 = yo1 + r_min * np.sin(gamma1)
    z1 = (z - z_goal) + r_min * (gamma1 - gamma) / k

    xb = xo1 + r_min * np.cos(gamma - beta1)
    yb = yo1 + r_min * np.sin(gamma - beta1)
    zb = (z - z_goal) + r_min * (-beta1) / k
    xc = xo2 + r_min * np.cos(beta2 + theta_ep)
    yc = yo2 + r_min * np.sin(beta2 + theta_ep)
    zc = zb - np.sqrt(xbc ** 2 + ybc ** 2) / k
    n2 = round((zb - zc) / delta_z)
    x2 = np.linspace(xc, xb, n2)
    y2 = np.linspace(yc, yb, n2)
    z2 = np.linspace(zc, zb, n2)

    zd = zc + r_min * (-beta2) / k
    n3 = round(r_min * beta2 / k / delta_z)
    gamma3 = np.linspace(gamma - beta1 - beta2, gamma - beta1, n3)
    x3 = xo2 + r_min * np.cos(gamma3)
    y3 = yo2 + r_min * np.sin(gamma3)
    z3 = zd + r_min * (gamma3 - (gamma - beta1 - beta2)) / k

    n4 = round(r_ep * beta3 / k / delta_z)
    gamma4 = np.linspace(gamma - beta1 - beta2 - beta3, gamma - beta1 - beta2, n4)
    x4 = r_ep * np.cos(gamma4)
    y4 = r_ep * np.sin(gamma4)
    z4 = zd + r_ep * (gamma4 - (gamma - beta1 - beta2)) / k

    ze = zd + r_ep * (-beta3) / k
    n5 = round(r_min * beta4 / k / delta_z)
    gamma5 = np.linspace(gamma - beta1 - beta2 - beta3 - beta4, gamma - beta1 - beta2 - beta3, n5)
    x5 = ((r_ep - r_min) * np.cos(beta4 - np.pi / 2)) + r_min * np.cos(gamma5)
    y5 = ((r_ep - r_min) * np.sin(beta4 - np.pi / 2)) + r_min * np.sin(gamma5)
    z5 = ze + r_min * (gamma5 - (gamma - beta1 - beta2 - beta3)) / k

    zf = ze + r_min * (-beta4) / k
    xf = ((r_ep - r_min) * np.cos(beta4 - np.pi / 2)) + r_min * np.cos(gamma - beta1 - beta2 - beta3 - beta4)
    yf = ((r_ep - r_min) * np.sin(beta4 - np.pi / 2)) + r_min * np.sin(gamma - beta1 - beta2 - beta3 - beta4)
    zg = zf - np.sqrt(xf ** 2 + yf ** 2) / k
    n6 = round((zf - zg) / delta_z)
    x6 = np.linspace(0, xf, n6)
    y6 = np.linspace(0, yf, n6)
    z6 = np.linspace(zg, zf, n6)

    path = np.block([
        [x6, x5, x4, x3, x2, x1],
        [y6, y5, y4, y3, y2, y1],
        [z6, z5, z4, z3, z2, z1]
    ]) + np.array([[x_goal, y_goal, z_goal]]).T

    return np.flip(path, axis=1)


class ParafoilAutopilot:
    def __init__(self):
        rospy.init_node("parafoil_autopilot", anonymous=True)

        self.state = State()
        self.path = None
        self.goal = None
        self.k = 3
        self.r_min = 5
        self.roll_controller = None
        self.pitch_controller = None

        self.last_pub = rospy.get_time()
        self.control_rate = 10

        rospy.Subscriber("/mavros/state", State, callback=self.state_callback, queue_size=1)
        rospy.Subscriber("/mavros/rc/in", RCIn, callback=self.rc_in_callback, queue_size=1)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, callback=self.odom_callback, queue_size=1)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.actuator_control_publisher = rospy.Publisher("/mavros/actuator_control", ActuatorControl, queue_size=1)
        rospy.spin()

    def state_callback(self, message):  # 1Hz
        self.state = message
        rospy.loginfo(f"mode: {message.mode}, armed: {message.armed}")

    def rc_in_callback(self, message):  # 10Hz
        mode = message.channels[4]
        if abs(mode - 1065) < 100 and self.state.mode != "OFFBOARD":
            if self.set_mode_client.call(SetModeRequest(custom_mode="OFFBOARD")).mode_sent:
                rospy.loginfo("OFFBOARD enabled")

    def odom_callback(self, message):  # 30Hz
        pose = message.pose.pose
        position = pose.position
        x, y, z = position.x, position.y, position.z
        orientation = pose.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        linear_velocity = message.twist.twist.linear
        v_x, v_y, v_z = linear_velocity.x, linear_velocity.y, linear_velocity.z
        angular_velocity = message.twist.twist.angular
        roll_rate, pitch_rate, yaw_rate = angular_velocity.x, angular_velocity.y, angular_velocity.z

        if self.goal is None:
            self.goal = [x, y, z]
        z = z + 30
        actuator_control = ActuatorControl()
        if self.state.mode != "OFFBOARD":
            self.actuator_control_publisher.publish(actuator_control)
            self.last_pub = rospy.get_time()
            self.path = None
        else:
            if self.path is None:
                self.path = parafoil_navigator(x, y, z, yaw, *self.goal, k=self.k, r_min=self.r_min)
                self.roll_controller = PIDController(1, 0, 0)
                self.pitch_controller = PIDController(1, 0, 0)
                np.savetxt("/home/polaris/catkin_ws/src/spacurs/scripts/path.txt", self.path, delimiter=',')
            else:
                lower_idx = np.where(self.path[2, :] < z)[0]
                distance = np.linalg.norm(np.array([[x, y, z]]).T - self.path[:, lower_idx], axis=0)
                closest_idx = lower_idx[0] + np.argmin(distance)
                target_idx = closest_idx + 1
                if target_idx > self.path.shape[1] - 1:
                    target_idx = self.path.shape[1] - 1

                x_t, y_t, z_t = self.path[:, target_idx]
                l1 = np.linalg.norm(np.array([[x - x_t, y - y_t]]).T, axis=0)
                eta = remap_angle(np.arctan2(y_t - y, x_t - x) - yaw)
                v_s = np.sqrt(v_x ** 2 + v_y ** 2)
                a_n_cmd = -2 * v_s ** 2 / l1 * np.sin(eta)
                a_n = v_s * yaw_rate

                gamma = -np.arctan(1 / self.k)

                roll_output = constraint(self.roll_controller.output(a_n_cmd - a_n), -1, 1)
                pitch_output = constraint(self.pitch_controller.output(gamma - pitch), -1, 1)
                actuator_control.controls = [roll_output, pitch_output, 0, 0, 0, 0, 0, 0]
            if rospy.get_time() - self.last_pub > 1 / self.control_rate:
                self.actuator_control_publisher.publish(actuator_control)
                self.last_pub = rospy.get_time()


if __name__ == "__main__":
    try:
        ParafoilAutopilot()
    except rospy.ROSInterruptException:
        pass
