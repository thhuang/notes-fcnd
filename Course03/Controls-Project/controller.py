"""
PID Controller

components:
    follow attitude commands
    gps commands and yaw
    waypoint following
"""
import numpy as np
from frame_utils import euler2RM

DRONE_MASS_KG = 0.5
GRAVITY = -9.81
MOI = np.array([0.005, 0.005, 0.01])  # moment of inertia (N*m*s^2/rad)
MAX_THRUST = 10.0
MAX_TORQUE = 1.0

class NonlinearController(object):

    def __init__(self,
                 k_p_x=2.5, k_p_y=2.5, k_p_z=12.0,
                 k_d_x=2.5, k_d_y=2.5, k_d_z=5.0,
                 k_p_roll=1.9, k_p_pitch=1.9, k_p_yaw=0.0,
                 k_p_p=13.0, k_p_q=13.0, k_p_r=0.0):

        """Initialize the controller object and control gains"""
        self.k_p_x = k_p_x
        self.k_p_y = k_p_y
        self.k_p_z = k_p_z
        self.k_d_x = k_d_x
        self.k_d_y = k_d_y
        self.k_d_z = k_d_z
        self.k_p_roll = k_p_roll
        self.k_p_pitch = k_p_pitch
        self.k_p_yaw = k_p_yaw
        self.k_p_p = k_p_p
        self.k_p_q = k_p_q
        self.k_p_r = k_p_r

        #print('x: k_p={:.2f}, k_d={:.2f}'.format(self.k_p_x, self.k_d_x))
        #print('y: k_p={:.2f}, k_d={:.2f}'.format(self.k_p_y, self.k_d_y))
        #print('z: k_p={:.2f}, k_d={:.2f}'.format(self.k_p_z, self.k_d_z))


    def trajectory_control(self, position_trajectory, yaw_trajectory, time_trajectory, current_time):
        """Generate a commanded position, velocity and yaw based on the trajectory

        Args:
            position_trajectory: list of 3-element numpy arrays, NED positions
            yaw_trajectory: list yaw commands in radians
            time_trajectory: list of times (in seconds) that correspond to the position and yaw commands
            current_time: float corresponding to the current time in seconds

        Returns: tuple (commanded position, commanded velocity, commanded yaw)

        """

        ind_min = np.argmin(np.abs(np.array(time_trajectory) - current_time))
        time_ref = time_trajectory[ind_min]


        if current_time < time_ref:
            position0 = position_trajectory[ind_min - 1]
            position1 = position_trajectory[ind_min]

            time0 = time_trajectory[ind_min - 1]
            time1 = time_trajectory[ind_min]
            yaw_cmd = yaw_trajectory[ind_min - 1]

        else:
            yaw_cmd = yaw_trajectory[ind_min]
            if ind_min >= len(position_trajectory) - 1:
                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min]

                time0 = 0.0
                time1 = 1.0
            else:

                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min + 1]
                time0 = time_trajectory[ind_min]
                time1 = time_trajectory[ind_min + 1]

        position_cmd = (position1 - position0) * \
                        (current_time - time0) / (time1 - time0) + position0
        velocity_cmd = (position1 - position0) / (time1 - time0)


        return (position_cmd, velocity_cmd, yaw_cmd)

    def lateral_position_control(self, local_position_cmd, local_velocity_cmd, local_position, local_velocity,
                               acceleration_ff = np.array([0.0, 0.0])):
        """Generate horizontal acceleration commands for the vehicle in the local frame

        Args:
            local_position_cmd: desired 2D position in local frame [north, east]
            local_velocity_cmd: desired 2D velocity in local frame [north_velocity, east_velocity]
            local_position: vehicle position in the local frame [north, east]
            local_velocity: vehicle velocity in the local frame [north_velocity, east_velocity]
            acceleration_cmd: feedforward acceleration command

        Returns: desired vehicle 2D acceleration in the local frame [north, east]
        """
        x_cmd, y_cmd = local_position_cmd
        x_dot_cmd, y_dot_cmd = local_velocity_cmd
        x_dot_dot_ff, y_dot_dot_ff = acceleration_ff
        x, y = local_position
        x_dot, y_dot = local_velocity
        # Calculate error terms (m)
        error_x = x_cmd - x
        error_y = y_cmd - y
        # Calculate derivative terms (m/s)
        derivative_x = x_dot_cmd - x_dot
        derivative_y = y_dot_cmd - y_dot
        # Calculate acceleration commands (m/s^2)
        x_dot_dot_cmd = self.k_p_x * error_x \
                      + self.k_d_x * derivative_x \
                      + x_dot_dot_ff
        y_dot_dot_cmd = self.k_p_y * error_y \
                      + self.k_d_y * derivative_y \
                      + y_dot_dot_ff

        return np.array([-x_dot_dot_cmd, -y_dot_dot_cmd])
        #return np.array([0, 0])

    def altitude_control(self, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, acceleration_ff=0.0):
        """Generate vertical acceleration (thrust) command

        Args:
            altitude_cmd: desired vertical position (+up)
            vertical_velocity_cmd: desired vertical velocity (+up)
            altitude: vehicle vertical position (+up)
            vertical_velocity: vehicle vertical velocity (+up)
            attitude: the vehicle's current attitude, 3 element numpy array (roll, pitch, yaw) in radians
            acceleration_ff: feedforward acceleration command (+up)

        Returns: thrust command for the vehicle (+up)
        """
        roll, pitch, yaw = attitude
        # Calculate b_z
        b_z = euler2RM(roll, pitch, yaw)[2, 2]
        # Calculate the error term (m)
        error_z = altitude_cmd - altitude
        # Calculate the derivative term (m/s)
        derivative_z = vertical_velocity_cmd - vertical_velocity
        # Calculate u_thrust_bar
        u_thrust_bar = self.k_p_z * error_z \
                     + self.k_d_z * derivative_z \
                     + acceleration_ff
        # Calculate and return thrust command (N)
        return DRONE_MASS_KG * (u_thrust_bar - GRAVITY) / b_z
        #return 0


    def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
        """ Generate the roll rate and pitch rate commands in the body frame

        Args:
            target_acceleration: 2-element numpy array (north_acceleration_cmd, east_acceleration_cmd) in m/s^2
            attitude: 3-element numpy array (roll, pitch, yaw) in radians
            thrust_cmd: vehicle thrusts command in Newton

        Returns: 2-element numpy array, desired roll rate (p_cmd) and pitch rate (q_cmd) commands in radians/s
        """
        roll, pitch, yaw = attitude  # (rad)
        R = euler2RM(roll, pitch, yaw)
        # Calculate b_x and b_y
        b_x, b_y = R[0:2, 2]
        # Calculate b_x_cmd and b_y_cmd
        c_cmd = thrust_cmd / DRONE_MASS_KG  # (m/s^2)
        b_x_cmd, b_y_cmd = acceleration_cmd / c_cmd
        # Calculate error terms
        error_b_x = b_x_cmd - b_x
        error_b_y = b_y_cmd - b_y
        # Calculate b_x_dot_cmd and b_y_dot_cmd (1/s)
        b_x_dot_cmd = self.k_p_roll * error_b_x
        b_y_dot_cmd = self.k_p_pitch * error_b_y
        # Calculate and return p_cmd and q_cmd (rad/s)
        rotation = np.array([[R[1,0], -R[0,0]], [R[1,1], -R[0,1]]]) / R[2,2]
        return rotation @ np.array([b_x_dot_cmd, b_y_dot_cmd])
        #return np.array([0, 0])


    def body_rate_control(self, body_rate_cmd, body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame

        Args:
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
            body_rate: 3-element numpy array (p,q,r) in radians/second^2

        Returns: 3-element numpy array, desired roll moment, pitch moment, and yaw moment commands in Newtons*meters
        """
        p_cmd, q_cmd, r_cmd = body_rate_cmd
        p, q, r = body_rate
        # Calculate error terms (rad/s)
        error_p = p_cmd - p
        error_q = q_cmd - q
        error_r = r_cmd - r
        # Calculate the u_bar (rad/s^2)
        u_bar_p = self.k_p_p * error_p
        u_bar_q = self.k_p_q * error_q
        u_bar_r = self.k_p_r * error_r
        # Calculate and return roll moment, pitch moment, and yaw moment commands (N*m)
        torque_x, torque_y, torque_z = MOI * np.array([u_bar_p, u_bar_q, u_bar_r])
        return np.clip([torque_x, torque_y, torque_z], -MAX_TORQUE, MAX_TORQUE)


    def yaw_control(self, yaw_cmd, yaw):
        """ Generate the yaw dot command

        Args:
            yaw_cmd: desired vehicle yaw in radians
            yaw: vehicle yaw in radians

        Returns: target yaw rate in radians/sec
        """
        # Calculate the error term (rad)
        error_yaw = yaw_cmd - yaw
        # Calculate the r command which is the yaw dot command (rad/s)
        r_cmd = self.k_p_yaw * error_yaw
        # Return the r_cmd = yaw_dot_cmd (rad/s)
        return r_cmd

