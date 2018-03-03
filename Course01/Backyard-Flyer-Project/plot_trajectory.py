import numpy as np
import argparse
import matplotlib.pyplot as plt
from udacidrone import Drone

parser = argparse.ArgumentParser()
parser.add_argument('--logfile', help='Path to the log file', type=str, default='Logs/TLog-manual.txt')
parser.add_argument('--output', help='Path to the output figure', type=str, default='Logs/trajectory_manually_flying.png')
args = parser.parse_args()

filename = args.logfile
t_log = Drone.read_telemetry_data(filename)

# Time is always the first entry in the list
time = t_log['MsgID.LOCAL_POSITION'][0][:]
north = t_log['MsgID.LOCAL_POSITION'][1][:]
east = t_log['MsgID.LOCAL_POSITION'][2][:]

plt.plot(north, east)
plt.xlabel('North (m)')
plt.ylabel('Ease (m)')
plt.savefig(args.output)
print('Plot of {} is written to {}'.format(args.logfile, args.output))
plt.close()