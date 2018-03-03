# FCND - Backyard Flyer Project

## 1. Autonomously Flying

```bash
python backyard_flyer.py --route_size 10 --route_altitude 3
```
<div align = 'center'>
<a href="https://youtu.be/HIV8B7xQXEs"><img src="http://img.youtube.com/vi/YOUTUBE_VIDEO_ID_HERE/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>
</div>

## 2. Manually Flying Trajectory

```bash
python plot_trajectory.py --logfile Logs/TLog-manual.txt --output Logs/trajectory_manually_flying.png
```
<div align='center'>
<img src = 'Logs/trajectory_manually_flying.png' height="400px">
</div>

## 3. Autonomously Flying Trajectory

```bash
python plot_trajectory.py --logfile Logs/TLog.txt --output Logstrajectory_autonomously_flying.png
```
<div align='center'>
<img src = 'Logs/trajectory_autonomously_flying.png' height="400px">
</div>