# Unscented Kalman Filter Project 

The goals / steps of this project are the following:

* Implement an Unscented Kalman Filter that reads lidar and radar measurement and performs sensor fusion for state estimates.

---

## Rubric points
#### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.


#### 1. Your code should compile. 
```
mkdir build; cd build; cmake ..; make && ./UnscentedKF
```

#### 2. px, py, vx, vy output coordinates must have an RMSE <=  [0.09, 0.10, 0.40, 0.30]
I ran the simulator and got
* On dataset 1 RMSE = [0.06, 0.10, 0.34, 0.22] < [0.098, 0.085, 0.409, 0.470] (Extended Kalman project results)
* On dataset 1 RMSE = [0.06, 0.10, 0.34, 0.22] < [0.09, 0.10, 0.40, 0.30] (rubric threshold)
* On dataset 2 RMSE = [0.09, 0.06, 0.66, 0.26] < [0.073, 0.097, 0.449, 0.464] (Extended Kalman project results)

#### 3. Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.
I have implemented the Unscented Kalman Filter following the logic from the lessons, buildin on the Extended Kalman Project.

#### 3. Your Kalman Filter algorithm handles the first measurements appropriately.
I initialize the first measurements in `UKF::InitializeState` function, called from `UKF::ProcessMeasurement`, based on receiving radar or lidar measurements.

#### 4. Your Kalman Filter algorithm first predicts then updates.
See the logic in `UKF::ProcessMeasurement`.

#### 5. Your Kalman Filter can handle radar and lidar measurements.
Based on the boolean flag you can use radar, lidar or both.

#### 6. Your algorithm should avoid unnecessary calculations.
Where applicable I cache the results.