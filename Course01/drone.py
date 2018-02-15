import argparse
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
import time

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    drone = Drone(conn, tlog_name="TLog-manual.txt")
    time.sleep(2)
    drone.start()