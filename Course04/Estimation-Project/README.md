# Project: Estimation

#### Step 1: Sensor Noise (scenario `06_NoisySensors`)
[`MeasuredStdDev_GPSPosXY`](https://github.com/thhuang/Estimation-Project/blob/master/config/06_SensorNoise.txt#L3) and [`MeasuredStdDev_AccelXY`](https://github.com/thhuang/Estimation-Project/blob/master/config/06_SensorNoise.txt#L4) are tuned to capture approximate 68% of the respective measurements.

#### Step 2: Attitude Estimation (scenario `07_AttitudeEstimation`)
[`UpdateFromIMU()`](https://github.com/thhuang/Estimation-Project/blob/master/src/QuadEstimatorEKF.cpp#L74) is implemented with a complementary filter to reduce the attitude estimation errors.

#### Step 3: Prediction Step

#### Step 4: Magnetometer Update

#### Step 5: Closed Loop + GPS Update

#### Step 6: Adding Your Controller