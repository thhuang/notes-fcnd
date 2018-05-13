## Project: Control of a 3D Quadrotor


### C++

#### Body rate and roll/pitch control (scenario 2)
- [X] Roll should less than 0.025 radian of nominal for 0.75 seconds (3/4 of the duration of the loop).
- [X] Roll rate should less than 2.5 radian/sec for 0.75 seconds.
1. Implement the code in the function [`GenerateMotorCommands()`](https://github.com/thhuang/NOTES-FCND/blob/master/Course03/Controls-Project/C%2B%2B/src/QuadControl.cpp#L58-L104)
2. Implement the code in the function `BodyRateControl()`
3. Tune `Kp_pqr` in `QuadControlParams.txt` to get the vehicle to stop spinning quickly but not overshoot
4. Implement the code in the function `RollPitchControl()`
5. Tune `Kp_bank` in `QuadControlParams.txt` to minimize settling time but avoid too much overshoot


#### Position/velocity and yaw angle control (scenario 3) 
- [X] X position of both drones should be within 0.1 meters of the target for at least 1.25 seconds.
- [X] Quad2 yaw should be within 0.1 of the target for at least 1 second.
1. Implement the code in the function `LateralPositionControl()`
2. Implement the code in the function `AltitudeControl()`
3. Tune parameters `Kp_pos_z` and `Kp_vel_z`
4. Tune parameters `Kp_vel_xy` and `Kp_vel_z`
5. Implement the code in the function `YawControl()`
6. Tune parameters `Kp_yaw` and the 3rd (z) component of `Kp_pqr`


#### Non-idealities and robustness (scenario 4)
- [X] Position error for all 3 quads should be less than 0.1 meters for at least 1.5 seconds.
1. Run your controller & parameter set from Step 3.  Do all the quads seem to be moving OK?  If not, try to tweak the controller parameters to work for all 3 (tip: relax the controller).
2. Edit `AltitudeControl()` to add basic integral control to help with the different-mass vehicle.
3. Tune the integral control, and other control parameters until all the quads successfully move properly.  Your drones' motion should look like this:


#### Tracking trajectories (scenario 5)
- [X] Position error of the quad should be less than 0.25 meters for at least 3 seconds.
1. Test the performance of the controller once again on a trajectory.

### Python (Optional)

#### 1. 