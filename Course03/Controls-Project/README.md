## Project: Control of a 3D Quadrotor


### C++

#### Body rate and roll/pitch control (scenario 2)
- [X] Roll should less than 0.025 radian of nominal for 0.75 seconds (3/4 of the duration of the loop).
- [X] Roll rate should less than 2.5 radian/sec for 0.75 seconds.
1. implement the code in the function `GenerateMotorCommands()` in `C++/src/QuadControl.cpp`

2. implement the code in the function `BodyRateControl()`
3. Tune `Kp_pqr` in `QuadControlParams.txt` to get the vehicle to stop spinning quickly but not overshoot


#### Position/velocity and yaw angle control (scenario 3) 
- [X] X position of both drones should be within 0.1 meters of the target for at least 1.25 seconds.
- [X] Quad2 yaw should be within 0.1 of the target for at least 1 second.


#### Non-idealities and robustness (scenario 4)
- [X] Position error for all 3 quads should be less than 0.1 meters for at least 1.5 seconds.


#### Tracking trajectories (scenario 5)
- [X] Position error of the quad should be less than 0.25 meters for at least 3 seconds.


### Python (Optional)

#### 1. 