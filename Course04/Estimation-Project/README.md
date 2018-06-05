# Project: Estimation

#### Step 1: Sensor Noise (scenario `06_NoisySensors`)
[`MeasuredStdDev_GPSPosXY`](https://github.com/thhuang/NOTES-FCND/blob/master/Course04/Estimation-Project/config/06_SensorNoise.txt#L3) and [`MeasuredStdDev_AccelXY`](https://github.com/thhuang/NOTES-FCND/blob/master/Course04/Estimation-Project/config/06_SensorNoise.txt#L4) are tuned to capture approximate 68% of the respective measurements.

#### Step 2: Attitude Estimation (scenario `07_AttitudeEstimation`)
[`UpdateFromIMU()`](https://github.com/thhuang/NOTES-FCND/blob/master/Course04/Estimation-Project/src/QuadEstimatorEKF.cpp#L74-L121) is implemented with a complementary filter to reduce the attitude estimation errors. The quaternion
integration provided is used in the function. 

#### Step 3: Prediction Step
With section 7.2 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj#/54894644/) as a reference, [`Predict()`](https://github.com/thhuang/NOTES-FCND/blob/master/Course04/Estimation-Project/src/QuadEstimatorEKF.cpp#L225-L283), the first step of the EKF, as well as [`PredictState()`](https://github.com/thhuang/NOTES-FCND/blob/master/Course04/Estimation-Project/src/QuadEstimatorEKF.cpp#L146-L184) and [`GetRbgPrime()`](https://github.com/thhuang/NOTES-FCND/blob/master/Course04/Estimation-Project/src/QuadEstimatorEKF.cpp#L186-L223) are implemented. [`QPosXYStd`](https://github.com/thhuang/NOTES-FCND/blob/master/Course04/Estimation-Project/config/QuadEstimatorEKF.txt#L8) and [`QVelXYStd`](https://github.com/thhuang/NOTES-FCND/blob/master/Course04/Estimation-Project/config/QuadEstimatorEKF.txt#L10) are also tuned to capture the magnitude of the error.

#### Step 4: Magnetometer Update
With section 7.3.2 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj#/54894644/) as a reference, Measurement model for magnetometer is implemented in [`UpdateFromMag()`](https://github.com/thhuang/NOTES-FCND/blob/master/Course04/Estimation-Project/src/QuadEstimatorEKF.cpp#L309-L335). Besides, [`QYawStd`](https://github.com/thhuang/NOTES-FCND/blob/master/Course04/Estimation-Project/config/QuadEstimatorEKF.txt#L12) is tuned to capture the magnitude of the drift. 

#### Step 5: Closed Loop + GPS Update
With section 7.3.1 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj#/54894644/) as a reference, Measurement model for GPS is implemented in [`UpdateFromGPS()`](https://github.com/thhuang/NOTES-FCND/blob/master/Course04/Estimation-Project/src/QuadEstimatorEKF.cpp#L285-L307).

#### Step 6: Adding Your Controller
[`QuadController.cpp`](https://github.com/thhuang/NOTES-FCND/blob/master/Course04/Estimation-Project/src/QuadControl.cpp) is replaced with the controller written in the [Controls-Project](https://github.com/thhuang/NOTES-FCND/tree/master/Course03/Controls-Project) with more constant added. Moreover, parameters for the controller in [`QuadControlParams.txt
`](https://github.com/thhuang/NOTES-FCND/blob/master/Course04/Estimation-Project/config/QuadControlParams.txt) are re-tuned to pass every scenario test.