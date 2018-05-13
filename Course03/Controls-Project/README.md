## Project: Control of a 3D Quadrotor


### C++

#### Body rate and roll/pitch control (scenario 2)
- [X] Roll should less than 0.025 radian of nominal for 0.75 seconds (3/4 of the duration of the loop).
- [X] Roll rate should less than 2.5 radian/sec for 0.75 seconds.
1. Implement the code in the function [`GenerateMotorCommands()`](https://github.com/thhuang/NOTES-FCND/blob/master/Course03/Controls-Project/C%2B%2B/src/QuadControl.cpp#L58-L104)
2. Implement the code in the function [`BodyRateControl()`](https://github.com/thhuang/NOTES-FCND/blob/master/Course03/Controls-Project/C%2B%2B/src/QuadControl.cpp#L106-L130)
3. Tune [`Kp_pqr`](https://github.com/thhuang/NOTES-FCND/blob/0536cd91a4065d48bc62a7f67e4b9b8d254b686e/Course03/Controls-Project/C%2B%2B/config/QuadControlParams.txt#L35) to get the vehicle to stop spinning quickly but not overshoot
4. Implement the code in the function [`RollPitchControl()`](https://github.com/thhuang/NOTES-FCND/blob/master/Course03/Controls-Project/C%2B%2B/src/QuadControl.cpp#L133-L174)
5. Tune [`Kp_bank`](https://github.com/thhuang/NOTES-FCND/blob/0536cd91a4065d48bc62a7f67e4b9b8d254b686e/Course03/Controls-Project/C%2B%2B/config/QuadControlParams.txt#L31) to minimize settling time but avoid too much overshoot


#### Position/velocity and yaw angle control (scenario 3) 
- [X] X position of both drones should be within 0.1 meters of the target for at least 1.25 seconds.
- [X] Quad2 yaw should be within 0.1 of the target for at least 1 second.
1. Implement the code in the function [`LateralPositionControl()`](https://github.com/thhuang/NOTES-FCND/blob/master/Course03/Controls-Project/C%2B%2B/src/QuadControl.cpp#L220-L257)
2. Implement the code in the function [`AltitudeControl()`](https://github.com/thhuang/NOTES-FCND/blob/master/Course03/Controls-Project/C%2B%2B/src/QuadControl.cpp#L176-L217)
3. Tune parameters [`Kp_pos_xy`](https://github.com/thhuang/NOTES-FCND/blob/0536cd91a4065d48bc62a7f67e4b9b8d254b686e/Course03/Controls-Project/C%2B%2B/config/QuadControlParams.txt#L22) and [`Kp_pos_z`](https://github.com/thhuang/NOTES-FCND/blob/0536cd91a4065d48bc62a7f67e4b9b8d254b686e/Course03/Controls-Project/C%2B%2B/config/QuadControlParams.txt#L23)
4. Tune parameters [`Kp_vel_xy`](https://github.com/thhuang/NOTES-FCND/blob/0536cd91a4065d48bc62a7f67e4b9b8d254b686e/Course03/Controls-Project/C%2B%2B/config/QuadControlParams.txt#L27) and [`Kp_vel_z`](https://github.com/thhuang/NOTES-FCND/blob/0536cd91a4065d48bc62a7f67e4b9b8d254b686e/Course03/Controls-Project/C%2B%2B/config/QuadControlParams.txt#L28)
5. Implement the code in the function [`YawControl()`](https://github.com/thhuang/NOTES-FCND/blob/master/Course03/Controls-Project/C%2B%2B/src/QuadControl.cpp#L260-L279)
6. Tune parameters [`Kp_yaw`](https://github.com/thhuang/NOTES-FCND/blob/0536cd91a4065d48bc62a7f67e4b9b8d254b686e/Course03/Controls-Project/C%2B%2B/config/QuadControlParams.txt#L32) and the 3rd (z) component of [`Kp_pqr`](https://github.com/thhuang/NOTES-FCND/blob/0536cd91a4065d48bc62a7f67e4b9b8d254b686e/Course03/Controls-Project/C%2B%2B/config/QuadControlParams.txt#L35)


#### Non-idealities and robustness (scenario 4)
- [X] Position error for all 3 quads should be less than 0.1 meters for at least 1.5 seconds.
1. Tweak the controller parameters to work for all three quads.
2. Edit [`AltitudeControl()`](https://github.com/thhuang/NOTES-FCND/blob/master/Course03/Controls-Project/C%2B%2B/src/QuadControl.cpp#L176-L217) to add basic integral control to help with the different-mass vehicle.
3. Tune the integral control, and other control parameters until all the quads successfully move properly.


#### Tracking trajectories (scenario 5)
- [X] Position error of the quad should be less than 0.25 meters for at least 3 seconds.
1. Test the performance of the controller once again on a trajectory.

### Python (Optional)

#### 1. 