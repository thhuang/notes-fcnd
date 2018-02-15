import numpy as np
import argparse
import matplotlib.pyplot as plt
from udacidrone import Drone

parser = argparse.ArgumentParser()
parser.add_argument('--logfile', help='Path to the log file', type=str, default='Logs/TLog.txt')
args = parser.parse_args()

filename = args.logfile
t_log = Drone.read_telemetry_data(filename)

# Time is always the first entry in the list
time = t_log['MsgID.LOCAL_POSITION'][0][:]
longitude = t_log['MsgID.LOCAL_POSITION'][1][:]
latitude = t_log['MsgID.LOCAL_POSITION'][2][:]

plt.plot(longitude, latitude)
plt.show()