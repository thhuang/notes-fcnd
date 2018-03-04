import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection, route_size=10, route_altitude=3, waypoints_csv=None):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # set flying plan
        if waypoints_csv == None:
            self.route_size = route_size
            self.route_altitude = route_altitude

        # register all callbacks
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def update_target_position(self, north, east, altitude):
        self.target_position[0] = north
        self.target_position[1] = east
        self.target_position[2] = altitude
        print('>>> target position: ({:.2f}, {:.2f}, {:.2f})'.format(north, east, altitude))

    def local_position_callback(self):
        """
        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        if self.flight_state == States.TAKEOFF:
            # coordinate conversion
            altitude = -self.local_position[2]
            # check if the altitude is within 95% of the target
            if altitude > 0.95 * self.target_position[2]:
                self.all_waypoints = self.calculate_box()
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            # check if the distance to the target is within 0.5 m
            recent_position = self.local_position * [1, 1, -1]
            if np.linalg.norm(recent_position - self.target_position, ord=2) < 0.5:
                # check if all_waypoints is not empty
                if len(self.all_waypoints):
                    self.waypoint_transition()
                else:
                    # make sure the speed is not too fast (within 0.1 m/s)
                    if np.linalg.norm(self.local_velocity, ord=2) < 0.1:
                        self.landing_transition()

    def velocity_callback(self):
        """
        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        if self.flight_state == States.LANDING and abs(self.local_position[2]) < 0.05:  # (0,0,0) is the home position
            self.disarming_transition()

    def state_callback(self):
        """
        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                self.manual_transition()

    def calculate_box(self):
        """
        Return waypoints to fly a box
        """
        dist = self.route_size
        height = self.route_altitude
        local_waypoints = [[dist,  0.0, height],
                           [dist, dist, height],
                           [ 0.0, dist, height],
                           [ 0.0,  0.0, height]]
        return local_waypoints

    def arming_transition(self):
        """
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        # take control of the drone
        self.take_control()
        # arm the drone
        self.arm()
        # set the home location to the current position
        self.set_home_position(self.global_position[0],  # north
                               self.global_position[1],  # east
                               self.global_position[2])  # altitude
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """
        1. Set target_position altitude to 3.0 m
        2. Command a takeoff to 3.0 m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        # set the target to (0,0,3)
        self.update_target_position(0.0, 0.0, 3.0)
        # takeoff
        self.takeoff(self.target_position[2])
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")
        new_position = self.all_waypoints.pop(0)
        self.update_target_position(new_position[0],  # north
                                    new_position[1],  # east
                                    new_position[2])  # altitude
        self.cmd_position(self.target_position[0],
                          self.target_position[1],
                          self.target_position[2],
                          0.0)
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        """
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', help='Port number', type=int, default=5760)
    parser.add_argument('--host', help='Host address, i.e. \'127.0.0.1\'', type=str, default='127.0.0.1')
    parser.add_argument('--route_size', help='Size of the route', type=float, default=10)
    parser.add_argument('--route_altitude', help='Height of the route', type=float, default=3)
    parser.add_argument('--waypoints_csv', help='Path to the waypoints csv file', type=str, default=None)
    args = parser.parse_args()

    connection = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    # conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(connection, args.route_size, args.route_altitude, args.waypoints_csv)
    time.sleep(2)
    drone.start()
