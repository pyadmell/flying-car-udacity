# Project: Estimation

## Project description

## Required Steps for a Passing Submission:

1. Make sure you have cloned the repository and gotten familiar with the additional project repository files Introduction.
2. Implement all the necessary update and prediction steps required for your estimator to meet all the performance criteria of each step, outlined in detail in the project [README](https://github.com/udacity/FCND-Estimation-CPP/blob/master/README.md)
3. Tune your estimator, and re-tune your controller from your controls C++ project to successfully fly the desired trajectory with realistic sensors

## File description
- [QuadEstimatorEKF.txt](./config/QuadEstimatorEKF.txt): This files contains the estimator parameters.
- [QuadEstimatorEKF.cpp](./src/QuadEstimatorEKF.cpp): This file contains the estimator implementation. The original file with placeholders for the EKF implementaion was provided by Udacity. 
- [QuadControlParams.txt](./config/QuadControlParams.txt): This file contains the configuration for the controller. The parameters are re-tuned to work successfully with the estimator.
- [QuadControl.cpp](./src/QuadControl.cpp): This file contains the implementation of the controller by Udacity. 

## Tasks description

#### Step 1: Sensor Noise


#### Step 2: Attitude Estimation


#### Step 3: Prediction Step


#### Step 4: Magnetometer Update


#### Step 5: Closed Loop + GPS Update


#### Step 6: Adding Your Controller


## [Rubric Points](https://review.udacity.com/#!/rubrics/1807/view)

## Writeup
### Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.

The writeup is provided in README.md as you are reading it.

## Implement Estimator

### Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.

```
# Quad.GPS.X
Sample Standard Deviation, s	0.71611221969575
Variance (Sample Standard), s2	0.51281671119757
Population Standard Deviation, σ	0.71216666779826
Variance (Population Standard), σ2	0.50718136272287
Total Numbers, N	91
Sum:	-2.540099
Mean (Average):	-0.027913175824176
Standard Error of the Mean (SEx̄):	0.075068958129833
```

```
# Quad.IMU.AX
Sample Standard Deviation, s	0.48245032602929
Variance (Sample Standard), s2	0.23275831708577
Population Standard Deviation, σ	0.48196763423657
Variance (Population Standard), σ2	0.2322928004516
Total Numbers, N	500
Sum:	-3.991359
Mean (Average):	-0.007982718
Standard Error of the Mean (SEx̄):	0.021575834495369
```

### Implement a better rate gyro attitude integration scheme in the `UpdateFromIMU()` function.


### Implement all of the elements of the prediction step for the estimator.


### Implement the magnetometer update.


### Implement the GPS update.


## Flight Evaluation

### Meet the performance criteria of each step.

### De-tune your controller to successfully fly the final desired box trajectory with your estimator and realistic sensors.
