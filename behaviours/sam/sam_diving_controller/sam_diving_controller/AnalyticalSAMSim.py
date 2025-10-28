#!/usr/bin/python3

import time

from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
from sam_diving_controller.IDivePub import MissionStates, ActuatorStates
from sam_diving_controller.controllers.DiveControllerInterface import DiveControllerInterface
from scipy.spatial.transform import Rotation as R
from smarc_control_msgs.msg import ControlInput, ControlReference

from smarc_modelling.control.control import *
from smarc_modelling.vehicles.SAM_casadi import SAM_casadi


class AnalyticalSAMSim(DiveControllerInterface):

    def __init__(self, node, dive_pub, dive_sub, param, rate=0.1):

        self._node = node
        self._dive_sub = dive_sub
        self._dive_pub = dive_pub
        self.param = param
        self._dt = rate

        super().__init__(self._node, self._dive_pub, self._dive_sub, self.param, self._dt)

        # Convenience Topics
        self._current_state = None
        self._current_state_in_odom = None
        self._current_state_in_mocap = None
        self._current_control = None
        self._ref = None
        self._error = None
        self._input = None
        self.waypoint = None

        self.pred_mpc = []

        # Declare counter
        self.ref_is_traj = False
        self.i = 0
        self.traj_len = 0

        self._initialized = False

        # Extract the CasADi model
        sam = SAM_casadi(dt=self._dt)
        self.dynamics = sam.dynamics()


    def update(self):
        """
        This is where all the magic happens.
        """
        # FIXME: This doesn't quite work. Replacing it with checking if
        # mission_state == RUNNING blocked the whole controller and it wouldn't
        # get the waypoint either.

        # Get the current states
        convert_state = True  # Flag to convert states
        self._current_state_in_odom = self._dive_sub.get_states()
        self._current_state_in_mocap = self._dive_sub.get_states_in_mocap()

        if self._current_state_in_mocap is None:
            self._loginfo(f"No state available yet.")
            return

        if not self._initialized:
            self._current_state = self.convert_flu_to_frd(self._current_state_in_mocap, convert_state)
            self._initialized = True


        self._current_control = self._dive_sub.get_control_input()

        if self.i % 100 == 0:
            self._current_state = self.convert_flu_to_frd(self._current_state_in_mocap, convert_state)



        x_current = self.get_state_array(self._current_state,
                                         self._current_control,
                                         is_init_state=False,
                                         is_trajectory=self.ref_is_traj)
        # DEBUG

        # States
        #x_current[3] = 1 
        #x_current[4:6] = 0 

        # Actuators
        #x_current[13] = 0
        #x_current[14] = 100
        #x_current[15:] = 0

        #x_current[15] *= -1

        sim_res, debug_val = self.rk4(x_current, x_current[13:], self._dt, self.dynamics)
        debug_array = np.array(debug_val)

        np.set_printoptions(precision=3)
        s = f"\nSIM INFO\n"  # {self._dive_sub.current_idx}/{self.traj_len}:\n"
        # s += f"NMPC solve time: {(end_time - start_time)*1000:.1f} ms\n"
        # s += f"Traj. index: {self._dive_sub.current_idx}/{self.traj_len}:\n" if self.ref_is_traj else f""
        s += f"position: x: {sim_res[0]:.3f}, y: {sim_res[1]:.3f}, z: {sim_res[2]:.3f}\n"
        s += f"orientation: x: {sim_res[4]:.3f}, y: {sim_res[5]:.3f}, z: {sim_res[6]:.3f} , w: {sim_res[3]:.3f}\n"
        s += f"actuators: vbs: {sim_res[13]:.3f}, lcg: {sim_res[14]:.3f}, rpm: {sim_res[17]:.3f}\n"
        s += f"   rudder: {sim_res[16]:.3f}, stern: {sim_res[15]:.3f}\n"
        s += f"g: {debug_array}\n"
            #s += f"W: {debug_array[0]}, B: {debug_array[1]}\n"

        self._loginfo(s)

        self.set_publisher(sim_res)

        self.i += 1

        return

    def rk4(self, x, u, dt, fun):
        k1, J_total = fun(x, u)
        k2, _ = fun(x+dt/2*k1, u)
        k3, _ = fun(x+dt/2*k2, u)
        k4, _ = fun(x+dt*k3, u)

        x_t = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)

        return x_t.full().flatten(), J_total

    def set_publisher(self, sim_res):
        state_msg = Odometry()
        state_msg.header.frame_id = 'mocap'
        state_msg.pose.pose.position.x = sim_res[0]
        state_msg.pose.pose.position.y = sim_res[1]
        state_msg.pose.pose.position.z = sim_res[2]
        state_msg.pose.pose.orientation.w = sim_res[3]
        state_msg.pose.pose.orientation.x = sim_res[4]
        state_msg.pose.pose.orientation.y = sim_res[5]
        state_msg.pose.pose.orientation.z = sim_res[6]
        state_msg.twist.twist.linear.x = sim_res[7]
        state_msg.twist.twist.linear.y = sim_res[8]
        state_msg.twist.twist.linear.z = sim_res[9]
        state_msg.twist.twist.angular.x = sim_res[10]
        state_msg.twist.twist.angular.y = sim_res[11]
        state_msg.twist.twist.angular.z = sim_res[12]

        self._dive_pub.set_state(state_msg)

        self._current_state = state_msg


    def get_reference(self):
        # TODO: refactor this if-statement as function.
        if self.ref_is_traj and not self._initialized:
            self.trajectory = self._dive_sub.get_path()

            if self.trajectory is None:
                self._loginfo_once("No trajectory received")
                return False
            else:
                self.trajectory = np.array(self.trajectory)  # Convert/make sure it is a numpy array

            # Declare duration of sim.
            self.traj_len = self.trajectory.shape[0]

            # Augment the trajectory and control input reference
            Uref = np.zeros((self.trajectory.shape[0], self.nu))  # Derivative reference - set to 0 to penalize large rate of change
            self.trajectory = np.concatenate((self.trajectory, Uref), axis=1)


        elif not self.ref_is_traj:

            if not self._dive_sub.has_waypoint():
                self._loginfo(f"No waypoint available")
                return False

            # Get Waypoint information
            waypoint_in_mocap = self._dive_sub.get_waypoint()

            # FIXME: This might be useless.
            if waypoint_in_mocap is None:
                self._loginfo(f"waypoint_in_mocap is None")
                return False

            self.waypoint = self.convert_wp_to_odometry(waypoint_in_mocap)

            self.wp_array = self.get_wp_array(self.waypoint)

        return True

    def convert_flu_to_frd(self, flu_msg, convert_state=True):
        """
        If convert_state, it converts an odometry message from FLU to FRD

        """
        frd_odometry = Odometry()
        frd_odometry.header.frame_id = flu_msg.header.frame_id
        frd_odometry.header.stamp = flu_msg.header.stamp
        if convert_state:
            frd_odometry.pose.pose.position.x = flu_msg.pose.pose.position.x
            frd_odometry.pose.pose.position.y = flu_msg.pose.pose.position.y
            frd_odometry.pose.pose.position.z = flu_msg.pose.pose.position.z
            quat = self.quat_flu_to_frd([flu_msg.pose.pose.orientation.w,
                                         flu_msg.pose.pose.orientation.x,
                                         flu_msg.pose.pose.orientation.y,
                                         flu_msg.pose.pose.orientation.z])
            frd_odometry.pose.pose.orientation.x = quat[1]
            frd_odometry.pose.pose.orientation.y = quat[2]
            frd_odometry.pose.pose.orientation.z = quat[3]
            frd_odometry.pose.pose.orientation.w = quat[0]

            frd_odometry.twist.twist.linear.x = flu_msg.twist.twist.linear.x
            frd_odometry.twist.twist.linear.y = -flu_msg.twist.twist.linear.y
            frd_odometry.twist.twist.linear.z = -flu_msg.twist.twist.linear.z
            frd_odometry.twist.twist.angular.x = flu_msg.twist.twist.angular.x
            frd_odometry.twist.twist.angular.y = -flu_msg.twist.twist.angular.y
            frd_odometry.twist.twist.angular.z = -flu_msg.twist.twist.angular.z

        else:
            frd_odometry = flu_msg

        return frd_odometry

    def quat_flu_to_frd(self, q_flu):
        """
        quat_flu = [q0, q1, q2, q3], with q0 the scalar part
        """
        quat_flu = np.array([q_flu[1], q_flu[2], q_flu[3], q_flu[0]])

        rot = R.from_euler('x', 180, degrees=True)
        r_flu = R.from_quat(quat_flu)  # Convert ENU quaternion to rotation object, assumes scalar last
        r_frd = r_flu.as_matrix() @ rot.as_matrix()
        quat_frd = R.from_matrix(r_frd).as_quat()  # Convert back to quaternion with scalar last
        quat_frd_right_order = np.array([quat_frd[3],  # w
                                         quat_frd[0],  # x
                                         quat_frd[1],  # y
                                         quat_frd[2]  # z
                                         ])
        return quat_frd_right_order

    def convert_enu_to_ned(self, enu_msg, convert_state=True):
        """
        If convert_state, it converts an odometry message from ENU to NED

        """
        ned_odometry = Odometry()
        ned_odometry.header.frame_id = "/mocap"  # state_msg.header.frame_id# + "_conv"
        ned_odometry.header.stamp = enu_msg.header.stamp
        if convert_state:
            ned_odometry.pose.pose.position.x = enu_msg.pose.pose.position.y
            ned_odometry.pose.pose.position.y = enu_msg.pose.pose.position.x
            ned_odometry.pose.pose.position.z = -enu_msg.pose.pose.position.z
            ned_odometry.pose.pose.orientation = enu_msg.pose.pose.orientation

            quat = self.quat_enu_to_ned([enu_msg.pose.pose.orientation.w,
                                         enu_msg.pose.pose.orientation.x,
                                         enu_msg.pose.pose.orientation.y,
                                         enu_msg.pose.pose.orientation.z])
            ned_odometry.pose.pose.orientation.x = quat[1]
            ned_odometry.pose.pose.orientation.y = quat[2]
            ned_odometry.pose.pose.orientation.z = quat[3]
            ned_odometry.pose.pose.orientation.w = quat[0]

            ned_odometry.twist.twist.linear.x = enu_msg.twist.twist.linear.y
            ned_odometry.twist.twist.linear.y = enu_msg.twist.twist.linear.x
            ned_odometry.twist.twist.linear.z = -enu_msg.twist.twist.linear.z
            ned_odometry.twist.twist.angular.x = enu_msg.twist.twist.angular.y
            ned_odometry.twist.twist.angular.y = enu_msg.twist.twist.angular.x
            ned_odometry.twist.twist.angular.z = -enu_msg.twist.twist.angular.z

        else:
            ned_odometry = enu_msg

        return ned_odometry

    def quat_enu_to_ned(self, quat_enu):
        """
        Transform quaternion from ENU to NED.
        q = [q0, q1, q2, q3], where q0 is the scalar part.
        """

        q = quat_enu

        q_ned = 1 / np.sqrt(2) * np.array([q[0] + q[3], q[1] + q[2], q[1] - q[2], q[0] - q[3]])
        q_ned /= np.linalg.norm(q_ned)

        return q_ned

    def convert_wp_to_odometry(self, wp_msg):
        """
        Returns waypoint as Odometry
        """
        odom_wp = Odometry()

        if isinstance(wp_msg, PoseStamped):
            odom_wp.header.frame_id = wp_msg.header.frame_id
            odom_wp.header.stamp = wp_msg.header.stamp

            odom_wp.pose.pose = wp_msg.pose

        elif isinstance(wp_msg, Pose):
            odom_wp.header.frame_id = '/mocap'
            odom_wp.header.stamp = self._node.get_clock().now().to_msg()
            odom_wp.pose.pose.position = wp_msg.position
            odom_wp.pose.pose.orientation = wp_msg.orientation

        elif isinstance(wp_msg, Odometry):
            odom_wp = wp_msg

        else:
            return None

        return odom_wp

    def get_state_array(self, state_msg, control_msg,
                        is_init_state=False,
                        is_trajectory=False):
        """
        Merges states and controls into numpy array and returns the state of
        the controller as the state vector x (numpy array)

        convert_state: state_msg is in ENU, x will be in NED

        Note: The MPC wants the quaternion scalar part first, [w, x, y, z]!
        """
        x = np.zeros(19)

        x[0] = state_msg.pose.pose.position.x
        x[1] = state_msg.pose.pose.position.y
        x[2] = state_msg.pose.pose.position.z
        x[3:7] = [state_msg.pose.pose.orientation.w,
                  state_msg.pose.pose.orientation.x,
                  state_msg.pose.pose.orientation.y,
                  state_msg.pose.pose.orientation.z]
        x[7] = state_msg.twist.twist.linear.x
        x[8] = state_msg.twist.twist.linear.y
        x[9] = state_msg.twist.twist.linear.z
        x[10] = state_msg.twist.twist.angular.x
        x[11] = state_msg.twist.twist.angular.y
        x[12] = state_msg.twist.twist.angular.z
        x[13] = control_msg['vbs']
        x[14] = control_msg['lcg']
        x[15] = control_msg['stern']
        x[16] = control_msg['rudder']
        x[17] = control_msg['rpm1']
        x[18] = control_msg['rpm2']

        # Due to numerical reasons, we add a small noise to the rpms in
        # waypoint following mode
        if is_init_state:
            if is_trajectory:
                x[17] = control_msg['rpm1']
                x[18] = control_msg['rpm2']
            else:
                x[17] = 1e-6
                x[18] = 1e-6

        return x

