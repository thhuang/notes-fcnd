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


class PIDController(object):
    def __init__(self, k_p, k_i, k_d):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.integrated_error = 0.0

    def control(self, error, error_dot, feedforward=0.0):
        self.integrated_error += error
        return self.k_p * error + \
               self.k_d * error_dot + \
               self.k_i * self.integrated_error + \
               feedforward


class PDController(PIDController):
    def __init__(self, k_p, k_d):
        super().__init__(k_p, 0.0, k_d)

    def control(self, error, error_dot, feedforward=0.0):
        return self.k_p * error + \
               self.k_d * error_dot + \
               feedforward


class PController(PIDController):
    def __init__(self, k_p):
        super().__init__(k_p, 0.0, 0.0)

    def control(self, error):
        return self.k_p * error


class NonlinearController(object):
    def __init__(self,
                 k_p_x=5.0, k_p_y=5.0, k_p_z=30.0,
                 k_d_x=4.0, k_d_y=4.0, k_d_z=10.0,
                 k_p_roll=7.0, k_p_pitch=7.0, k_p_yaw=3.0,
                 k_p_p=20.0, k_p_q=20.0, k_p_r=5.0):

        """Initialize the controller object and control gains"""
        # Lateral position controller (PD controller)
        self.x_controller = PDController(k_p_x, k_d_x)
        self.y_controller = PDController(k_p_y, k_d_y)

        # Altitude controller (PD controller)
        self.z_controller = PDController(k_p_z, k_d_z)

        # Roll-pitch controller (P controller)
        self.roll_controller  = PController(k_p_roll)
        self.pitch_controller = PController(k_p_pitch)

        # Yaw controller (P controller)
        self.yaw_controller = PController(k_p_yaw)

        # Body rate controller (P controller)
        self.p_controller = PController(k_p_p)
        self.q_controller = PController(k_p_q)
        self.r_controller = PController(k_p_r)

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
        error = local_position_cmd - local_position
        error_dot = local_velocity_cmd - local_velocity

        return np.array([self.x_controller.control(error[0], error_dot[0], acceleration_ff[0]),
                         self.y_controller.control(error[1], error_dot[1], acceleration_ff[1])])

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
        # Calculate b_z
        b_z = euler2RM(*attitude)[2, 2]
        # Calculate the error term (m)
        error_z = altitude_cmd - altitude
        # Calculate the derivative term (m/s)
        error_dot_z = vertical_velocity_cmd - vertical_velocity
        # Calculate u_thrust_bar
        u_thrust_bar = self.z_controller.control(error_z, error_dot_z, acceleration_ff)
        # Calculate and return thrust command (N)
        return np.clip(DRONE_MASS_KG * (u_thrust_bar - GRAVITY) / b_z, 0.0, MAX_THRUST)

    def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
        """ Generate the roll rate and pitch rate commands in the body frame

        Args:
            target_acceleration: 2-element numpy array (north_acceleration_cmd, east_acceleration_cmd) in m/s^2
            attitude: 3-element numpy array (roll, pitch, yaw) in radians
            thrust_cmd: vehicle thrusts command in Newton

        Returns: 2-element numpy array, desired roll rate (p_cmd) and pitch rate (q_cmd) commands in radians/s
        """
        if thrust_cmd > 0:
            # Calculate rotation matrix
            R = euler2RM(*attitude)
            # Calculate b_x and b_y
            b_xy = R[0:2, 2]
            # Calculate b_x_cmd and b_y_cmd (value of the rotation matrix should be between -1 and 1)
            c_cmd = -thrust_cmd / DRONE_MASS_KG  # (m/s^2)
            b_xy_cmd = np.clip(acceleration_cmd/c_cmd, -1, 1)
            # Calculate error terms
            error = b_xy_cmd - b_xy
            # Calculate b_x_dot_cmd and b_y_dot_cmd (1/s)
            b_xy_dot_cmd = np.array([self.roll_controller.control(error[0]),
                                     self.pitch_controller.control(error[1])])
            # Calculate and return p_cmd and q_cmd (rad/s)
            rotation = np.array([[R[1,0], -R[0,0]], [R[1,1], -R[0,1]]]) / R[2,2]
            return rotation @ b_xy_dot_cmd
        else:
            return np.array([0, 0])

    def body_rate_control(self, body_rate_cmd, body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame

        Args:
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
            body_rate: 3-element numpy array (p,q,r) in radians/second^2

        Returns: 3-element numpy array, desired roll moment, pitch moment, and yaw moment commands in Newtons*meters
        """
        error = body_rate_cmd - body_rate
        u_body_bar = np.array([self.p_controller.control(error[0]),
                               self.q_controller.control(error[1]),
                               self.r_controller.control(error[2])])
        torque_cmd = MOI * u_body_bar
        return np.clip(torque_cmd, -MAX_TORQUE, MAX_TORQUE)

    def yaw_control(self, yaw_cmd, yaw):
        """ Generate the yaw dot command

        Args:
            yaw_cmd: desired vehicle yaw in radians
            yaw: vehicle yaw in radians

        Returns: target yaw rate in radians/sec
        """
        # Calculate the error term (rad)
        error_yaw = yaw_cmd - yaw
        while error_yaw > np.pi:
            error_yaw = error_yaw - 2.0*np.pi
        while error_yaw < -np.pi:
            error_yaw = error_yaw + 2.0*np.pi
        # Calculate the r command which is the yaw dot command (rad/s)
        r_cmd = self.yaw_controller.control(error_yaw)
        # Return the r_cmd = yaw_dot_cmd (rad/s)
        return r_cmd

