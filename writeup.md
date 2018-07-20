# **Unscented Kalman Filter Project**

The goals / steps of this project are the following:
* Understand how UKF(Unscented Kalman Filter) works in object detection
* Implement UKF in C++


[//]: # (Image References)

[image1]: ./writeup_images/best.png "Best RMSE"
[image2]: ./writeup_images/UKF_roadmap.png "UKF Roadmap"
[image3]: ./writeup_images/lidar_nis.png "Lidar NIS"
[image4]: ./writeup_images/radar_nis.png "Radar NIS"
[image5]: ./writeup_images/nis_before_tuning.png "NIS before tuning"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/783/view) individually and describe how I addressed each point in my implementation.  

### Compiling

#### 1. Your code should compile.
I completed the project under Ubuntu bash on Windows 10. I didn't modify CMakeLists.txt and other configuration files, so follow below to compile the code.  

~~~sh
cd build
cmake ..
make
./UnscentedKF
~~~

Then, launch Term 2 simulator.

### Accuracy
#### 1. RMSE must be less than  [.09, .10, .40, .30].  
Final result is as below.  
![image1]  


### Follows the Correct Algorithm
#### 1. Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.
In lesson, sensor fusion algorithm is described as below.  
![image2]  

Important logics are implemented in `main.cpp`. `Line 41 onMessage()` communicates with simulator and takes data input. After parsing data into proper format, it calls `ukf.ProcessMeasurement()` to process Kalman Filter Algorithm.  

In `ProcessMeasurement()`function, it processes initial measurement and `predict` / `measurement update` steps.  

For prediction, I first predicted sigma points. I followed process 'Generating Augmented Sigma Points -> Predict Sigma Points -> Calculate Mean and Covariance'. Specific code is as below.  

~~~cpp
// ukf.cpp : Line 148 - 224
/*
    Generate Augmented Sigma Points
 */
// Xk
VectorXd X_aug = VectorXd(n_aug_);
X_aug.head(n_x_) = x_;
X_aug(n_x_) = 0;
X_aug(n_x_+1) = 0;

MatrixXd P_aug(n_aug_, n_aug_);
P_aug.fill(0.0);
P_aug.topLeftCorner(n_x_, n_x_) = P_;
P_aug(n_x_, n_x_) = std_a_ * std_a_;
P_aug(n_x_+1, n_x_+1) = std_yawdd_ * std_yawdd_;

// Total 15 Sigma Points 2 * 7 + 1
for (int i = 0; i < n_aug_; i++) {
    Xsig_aug.col(i+1)               = X_aug + sqrt(lambda_+ n_aug_) * L.col(i);
    Xsig_aug.col(n_aug_+i+1) = X_aug - sqrt(lambda_ + n_aug_) * L.col(i);
}

/*
    Predict Sigma Points
*/
double px_p = px + v / yaw_rate * (sin(yaw + yaw_rate*delta_t) - sin(yaw));
double py_p = py + v / yaw_rate * (-cos(yaw + yaw_rate * delta_t) +cos(yaw));
double v_p = v;
double yaw_p = yaw + delta_t * yaw_rate;
double yawd_p = yaw_rate;

// Add noise
px_p += 0.5 * delta_t * delta_t * cos(yaw) * nu_a;
py_p += 0.5 * delta_t * delta_t * sin(yaw) * nu_a;
v_p += delta_t * nu_a;
yaw_p += 0.5 * delta_t * delta_t * nu_yawdd;
yawd_p += nu_yawdd * delta_t;

/*
    Calculate Mean and Covariance for Estimated Sigma Points
*/
for (int i = 0; i < 2*n_aug_+1; i++) {
    x_temp += weights_(i) * Xsig_pred_.col(i);
}

for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd x_diff = x_ - Xsig_pred_.col(i);
    x_diff(3) = Tools::ThetaValueCorrection(x_diff(3));
    P_temp += weights_(i) * (x_diff * x_diff.transpose());
}
~~~

After predict x and P in `UKF::Prediction()`, I updated measurements according to sensor type; `UKF::UpdateLidar()` and `UKF::UpdateRadar()`. Since Lidar sensor follows linear model, I used the same update calculation as EKF for Lidar.

For Radar sensor, I followed UKF process. First, I project estimated sigma points into measurement space with `RadarHFunction()` function that I defined. And then, adjusted UKF update state equation.


After x and P value update, it calculates NIS and RMSE, send the result back to the simulator.  

#### 2. Kalman Filter algorithm handles the first measurements appropriately.
After taking measurement data, measurements are processed in `ProcessMeasurement()` function.

If it is first measurement, use it to initialize `x` value. Lidar and Radar sensor data should be turned into `x` vector using different formula. Lidar data contains px and py, so just take them into x[0] and x[1].
~~~cpp
// ukf.cpp : Line 104 - 106
VectorXd measurements = meas_package.raw_measurements_;
x_[0] = measurements[0];
x_[1] = measurements[1];
~~~

Radar data contains rho, theta and rho_dot, so it should be converted into px, py value.  
~~~cpp
// FusionEKF.cpp : Line 95 - 99
VectorXd measurements = meas_package.raw_measurements_;
float rho = measurements[0];
float theta = measurements[1];
x_[0] = rho * cos(theta);
x_[1] = rho * sin(theta);
~~~

Because we have information to initialize px and py value, for covariance matrix P, starting from Identity matrix, I set σ​2px and σ​2py value lower . 
~~~cpp
  // initial state vector
  x_ = VectorXd(5);
  x_ << 1, 1, 1, 1, 1;

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 0.1, 0, 0, 0, 0,
            0, 0.1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
~~~

#### 3. Kalman Filter algorithm first predicts then updates.
In `ProcessMeasurement()` function, First predict x and P value as described in Rubric 1. And then, according to sensor type, call seperate update function; `UKF::UpdateLidar()` and `UKF::UpdateRadar()`.

~~~cpp
// ukf.cpp : Line 122 - 131
Prediction(delta_t);

if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) 
{
    UpdateRadar(meas_package);
} 
else if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) 
{
    UpdateLidar(meas_package);
}
~~~

#### 4. Kalman Filter can handle radar and lidar measurements.
In measurement update step, Lidar and Radar sensor follows different logic. As stated before, for Lidar measurement, I used same equation as EKF. Since state vector x is sized 5, H matrix is formed as [ 1, 0, 0, 0, 0 ;  0, 1, 0, 0, 0].
~~~cpp
// ukf.cpp : Line 230 - 249
MatrixXd H(2, 5);
H << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0;
MatrixXd R(2, 2);
R << std_laspx_ * std_laspx_, 0, 
        0, std_laspy_ * std_laspy_;

VectorXd z = meas_package.raw_measurements_ - H * x_;
S = H * P_ * H.transpose() + R;
K = P_ * H.transpose() * S.inverse();

MatrixXd I = MatrixXd::Identity(5,5);
x_ = x_ + (K * z);
P_ = (I - K * H) * P_;
~~~

But when dealing with Radar sensor value, matrix H should be totally different because it is made up of rho, theta and rho_dot values. And it needs to be handled with non-linear function. We need different process with Lidar sensor; `Calculate Mean Z and Covariance S in measurement space -> Update x and P`.

~~~cpp
// ukf.cpp : Line 257 - 308
/*
    Calculate Measurement Mean Z and Covariance S
*/
for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    Z += weights_(i) * Zsig.col(i);
}

for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd z_diff = Z - Zsig.col(i);
    z_diff(1) = Tools::ThetaValueCorrection(z_diff(1));
    S += weights_(i) * (z_diff * z_diff.transpose());
}

/*
    Matrix T
*/
for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd x_diff = x_ -  Xsig_pred_.col(i);
    x_diff(3) = Tools::ThetaValueCorrection(x_diff(3));

    VectorXd z_diff = Z - Zsig.col(i);
    z_diff(1) = Tools::ThetaValueCorrection(z_diff(1));

    T += weights_(i) * (x_diff * z_diff.transpose());
}

/*
    Update x and P
*/
MatrixXd K = T * S.inverse();
VectorXd measurements = meas_package.raw_measurements_;

MatrixXd z_diff = measurements - Z;
z_diff(1) = Tools::ThetaValueCorrection(z_diff(1));

// Update State
x_ = x_ + K * z_diff;
P_ = P_ - K * S * K.transpose();
~~~

---
To meet RMSE rubric, I needed to tune initial state of x and P, and standard deviation for process noise a and yaw acceleration; `std_a_` and `std_yawdd_`. Because we assume tracking bicycle, I start `std_a_ = 3` and `std_yawdd_ = 1`. In my case, RMSE for px and py were easily met, but vx, vy stays above 0.4, so I tried different values to reduce them. NIS of both Lidar and Raser sensor also seemed appropriate even before tuning.

![image5]  

Chaning `std_a_` and `std_yawdd_` didn't work for me, so I tried find proper initial value. I used Identity matrix for inital P, and I lowered σ​2px and σ​2py since I can calculate px and py value from first measurement.  

With P value [0.1, 0, 0, 0, 0;  0, 0.1, 0, 0, 0; 0, 0, 1, 0, 0;  0, 0, 0, 1, 0;  0, 0, 0, 0, 1;], I get best result as below.  

![image1]    

![image3]  
![image4]  

