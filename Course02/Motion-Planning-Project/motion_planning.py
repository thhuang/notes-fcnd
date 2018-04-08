import argparse
import msgpack
import time

import numpy as np
import planning_utils as pu

from enum import Enum, auto
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, map_name='colliders.csv', goal_position=None):
        super().__init__(connection)

        self.map_name = map_name
        if goal_position[0] is None or goal_position[1] is None:
            self.goal_position = None
        elif goal_position[2] is None:
            self.goal_position = [goal_position[0], goal_position[1], 5.0]
        else:
            self.goal_position = goal_position

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        MAX_ALTITUDE = 30
        SAFETY_DISTANCE = 5
        MIN_SAMPLES = 200
        MAX_SAMPLES = 500
        k = 8


        # Read (lon0, lat0, alt0) from self.map_name into floating point values
        # Default: alt0 is 0.0
        global_home = pu.get_global_home(self.map_name)

        # Set home position to (lon0, lat0, alt0)
        self.set_home_position(global_home[0], global_home[1], global_home[2])

        # Retrieve current global position.
        current_global_position = [self._longitude, self._latitude, self._altitude]

        # Convert to current local position using global_to_local()
        current_local_position = global_to_local(current_global_position, self.global_home)

        # Double check
        #print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position, self.local_position))
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, current_global_position, current_local_position))

        # Read in obstacle map
        data = np.loadtxt(self.map_name, delimiter=',', dtype='Float64', skiprows=2)
        polygons = pu.extract_polygons(data, SAFETY_DISTANCE)

        # Set goal position on the grid
        if self.goal_position is None:
            goal_position = pu.random_sample(data, polygons, num_samples=1, zmax=MAX_ALTITUDE).ravel()
        else:
            goal_local_position = global_to_local(self.goal_position, self.global_home)
            goal_position = [int(coord) for coord in goal_local_position]

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = pu.create_grid(data, goal_position[2], SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # Define starting point to current position
        start_position = [int(coord) for coord in self.local_position]

        pu.plot_map_2D(grid, start_position=start_position, goal_position=goal_position,
                       north_offset=north_offset, east_offset=east_offset)


        # Run A* to find a path from start to goal
        print('Local Start and Goal: ', start_position, goal_position)
        path, cost = pu.probabilistic_roadmap(data, polygons, start_position, goal_position, MAX_ALTITUDE,
                                              num_samples=np.random.randint(MIN_SAMPLES, MAX_SAMPLES),
                                              safety_distance=SAFETY_DISTANCE, k=k)

        # Prune the path
        path = pu.prune_path(polygons, path)
        pu.plot_map_2D(grid, start_position=start_position, goal_position=goal_position,
                       north_offset=north_offset, east_offset=east_offset, path=path,
                       plot_name='final_path_{}_{}'.format(tuple(start_position), tuple(goal_position)))

        # Set takeoff height to the altitude of the first waypoints
        self.target_position[2] = path[0][2]
        # Convert path to waypoints
        self.waypoints = [(int(p[0]), int(p[1]), int(p[2]), 0) for p in path]
        # Send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        #while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--goal_lon', type=float, default=None, help='goal longitude')
    parser.add_argument('--goal_lat', type=float, default=None, help='goal latitude')
    parser.add_argument('--goal_alt', type=float, default=5.0, help='goal altitude')
    args = parser.parse_args()

    pu.construct_graph('colliders.csv', num_samples=6000, k=10, zmax=30)

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn, goal_position=[args.goal_lon, args.goal_lat, args.goal_alt])
    time.sleep(1)

    drone.start()

