## Introduction

- Data: lidar and radar measurements of a bicycle traveling around a car.

- Term 2 Simulator needs to be downloaded for this project. Further info in uWebSocketIO section.

- Simulator feeds the data to the c++ scripts and the script should give back the estimation and RMSE values.

- The github repo for the project has data description: `obj_pose-laser-radar-synthetic-input.txt`. The Simulator feeds this data to `main.cpp` one line at a time. Each line is a measurement where the first column tells the sensor type (R=radar, L=lidar)

 - For a row containing radar data, the columns are: `sensor_type`, `rho_measured`, `phi_measured`, `rhodot_measured`, `timestamp`, `x_groundtruth`, `y_groundtruth`, `vx_groundtruth`, `vy_groundtruth`, `yaw_groundtruth`, `yawrate_groundtruth`.

- For a row containing lidar data, the columns are: `sensor_type`, `x_measured`, `y_measured`, `timestamp`, `x_groundtruth`, `y_groundtruth`, `vx_groundtruth`, `vy_groundtruth`, `yaw_groundtruth`, `yawrate_groundtruth`.

- Whereas radar has three measurements (rho, phi, rhodot), lidar has two measurements (x, y).

- Initial codes are provided by Udacity. `main.cpp` will read and parse the data.

- The measurements for each line of data should be stored in `measurement_pack_list` and the ground truth onto `ground_truth`. Using these two RMSE can be calculated using `tool.cpp`.

Three main steps of KL:
1. initializing
2. predicting
3. updating
4. repeat 2&3

## Files in the Github src Folder
The files you need to work with are in the `src` folder of the github repository.

- `main.cpp` - communicates with the Term 2 Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE
- `FusionEKF.cpp` - initializes the filter, calls the predict function, calls the update function
- `kalman_filter.cpp` - defines the predict function, the update function for lidar, and the update function for radar
- `tools.cpp` - function to calculate RMSE and the Jacobian matrix
- The only files you need to modify are `FusionEKF.cpp`, `kalman_filter.cpp`, and `tools.cpp`.

## How the Files Relate to Each Other
Here is a brief overview of what happens when you run the code files:

- `main.cpp` reads in the data and sends a sensor measurement to `FusionEKF.cpp`.
- `FusionEKF.cpp` takes the sensor data and initializes variables and updates variables. The Kalman filter equations are not in this file. `FusionEKF.cpp` has a variable called `ekf_`, which is an instance of a KalmanFilter class. The `ekf_` will hold the matrix and vector values. You will also use the `ekf_` instance to call the predict and update equations.
- The KalmanFilter class is defined in `kalman_filter.cpp` and `kalman_filter.h`. You will only need to modify `kalman_filter.cpp`, which contains functions for the prediction and update steps.

## main.cpp

- You do not need to modify the `main.cpp`, but let's discuss what the file does.

- Term 2 Simulator acts as a client the the c++ program is the web server. There are several functions inside `main.cpp` for handling the `uWebsocketIO` communications to the Simulator.

- Here is the main protocol that main.cpp uses for `uWebSocketIO` in communicating with the simulator.

```c++
INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]
```

- All the main code loops in `h.onMessage()`, to have access to intial variables that we created at the beginning of `main()`, we pass pointers as arguments into the header of `h.onMessage()`. For example:

```c++
h.onMessage([&fusionEKF,&tools,&estimations,&ground_truth]
            (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
             uWS::OpCode opCode)
```

The rest of the arguments in `h.onMessage` are used to set up the server.

```c++
  // Create a Fusion EKF instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  //Call the EKF-based fusion
  fusionEKF.ProcessMeasurement(meas_package); 
```

The code is:

- Creating an instance of the `FusionEKF` class.
- Receiving the measurement data calling the `ProcessMeasurement()` function. `ProcessMeasurement()` is responsible for the initialization of the Kalman filter as well as calling the prediction and update steps of the Kalman filter. You will be implementing the `ProcessMeasurement()` function in `FusionEKF.cpp`:

Finally,

The rest of main.cpp will output the following results to the simulator:

- estimation position
- calculated RMSE
`main.cpp` will call a function to calculate root mean squared error:

```c++
  // compute the accuracy (RMSE)
  Tools tools;
  cout << "Accuracy - RMSE:" << endl << tools.CalculateRMSE(estimations, ground_truth) << endl;
```

You will implement an RMSE function in the `tools.cpp` file.

## Project Code

Let's discuss the three files that you will need to modify.

### `FusionEKF.cpp`
In `FusionEKF.cpp`, we have given some starter code for implementing sensor fusion. In this file, you won't need to include the actual Kalman filter equations; instead, you will be initializing variables, initializing the Kalman filters, and then calling functions that implement the prediction step or update step. You will see TODO comments indicating where to put your code.

You will need to:

1. Initialize variables and matrices (`x`, `F`, `H_laser`, `H_jacobian`, `P`, etc.)
2. Initialize the Kalman filter position vector with the first sensor measurements
3. Modify the `F` and `Q` matrices prior to the prediction step based on the elapsed time between measurements
4. Call the update step for either the lidar or radar sensor measurement. Because the update step for lidar and radar are slightly different, there are different functions for updating lidar and radar.

**Initializing Variables in `FusionEKF.cpp`**

```c++
  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
```

Every time main.cpp calls `fusionEKF.ProcessMeasurement(measurement_pack_list[k])`, the code in `FusionEKF.cpp` will run. If this is the first measurement, the Kalman filter will try to initialize the object's location with the sensor measurement.

**Initializing the Kalman Filter in `FusionEKF.cpp`**

```c++
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.

    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
```

**Predict and Update Steps in `FusionEKF.cpp`**

Once the Kalman filter gets initialized, the next iterations of the for loop will call the `ProcessMeasurement()` function to do the predict and update steps.

```c++
  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO: Use the sensor type to perform the update step.
   * TODO: Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates

  } else {
    // TODO: Laser updates

  }
```

In `FusionEKF.cpp`, you will see references to a variable called `ekf_`. The `ekf_` variable is an instance of the KalmanFilter class. You will use `ekf_` to store your Kalman filter variables `(x, P, F, H, R, Q)` and call the predict and update functions. Let's talk more about the `KalmanFilter` class.

**`KalmanFilter` Class**

`kalman_filter.h` defines the `KalmanFilter` class containing the x vector as well as the `P, F, Q, H` and `R` matrices. The `KalmanFilter` class also contains functions for the prediction step as well as the Kalman filter update step (lidar) and extended Kalman filter update step (radar).

You will need to add your code to `kalman_filter.cpp` to implement the prediction and update equations. You do not need to modify `kalman_filter.h`.

Because lidar uses linear equations, the update step will use the basic Kalman filter equations. On the other hand, radar uses non-linear equations, so the update step involves linearizing the equations with the Jacobian matrix. The `Update` function will use the standard Kalman filter equations. The `UpdateEKF` will use the extended Kalman filter equations:

```c++
void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
}
```

**`tools.cpp**

This file is relatively straight forward. You will implement functions to calculate root mean squared error and the Jacobian matrix:

```c++
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO: Calculate a Jacobian here.
   */
}
```

**Compiling and Running Your Code**

Take a look at the github repo [README file](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project/blob/master/README.md) for instructions on how to compile and run your code.

## Summary of What Needs to Be Done

1. In `tools.cpp`, fill in the functions that calculate root mean squared error (RMSE) and the Jacobian matrix.
2. Fill in the code in `FusionEKF.cpp`. You'll need to initialize the Kalman Filter, prepare the `Q` and `F` matrices for the prediction step, and call the radar and lidar update functions.
3. In i`kalman_filter.cpp`, fill out the `Predict()`, `Update()`, and `UpdateEKF()` functions.

### Tips and Tricks
**No Need to Tune Parameters**

- The `R` matrix values and `Q` noise values are provided for you. There is no need to tune these parameters for this project.

**Initializing the State Vector**

- You'll need to initialize the state vector with the first sensor measurement.
- Although radar gives velocity data in the form of the range rate (`ρ_dot`), a radar measurement does not contain enough information to determine the state variable velocities `v_x` and `v_y`. You can, however, use the radar measurements `ρ` and `ϕ` to initialize the state variable locations `p_x` and `p_y`.

**Calculating `y = z - H * x'`**

- For lidar measurements, the error equation is `y = z - H * x'`. For radar measurements, the functions that map the `x` vector `[px, py, vx, vy]` to polar coordinates are non-linear. Instead of using `H` to calculate `y = z - H * x'`, for radar measurements you'll have to use the equations that map from cartesian to polar coordinates: `y = z - h(x')`.

**Normalizing Angles**

- In C++, `atan2()` returns values between `-pi` and `pi`. When calculating phi in `y = z - h(x)` for radar measurements, the resulting angle phi in the `y` vector should be adjusted so that it is between `-pi` and `pi`. The Kalman filter is expecting small angle values between the range `-pi` and `pi`. **HINT**: when working in radians, you can add `2π` or subtract `2π` until the angle is within the desired range.

**Avoid Divide by Zero throughout the Implementation**

- Before and while calculating the Jacobian matrix `Hj`, make sure your code avoids dividing by zero. For example, both the `x` and `y` values might be zero or `px*px + py*py` might be close to zero. What should be done in those cases?

**Test Your Implementation**

- Test! We're giving you the ability to analyze your output data and calculate RMSE. As you make changes, keep testing your algorithm! If you are getting stuck, add print statements to pinpoint any issues. But please remove extra print statements before turning in the code.

### Ideas for Standing out!
The Kalman Filter general processing flow that you've learned in the preceding lessons gives you the basic knowledge needed to track an object. However, there are ways that you can make your algorithm more efficient!

- Dealing with the first frame, in particular, offers opportunities for improvement.
- Experiment and see how low your RMSE can go!
- Try removing radar or lidar data from the filter. Observe how your estimations change when running against a single sensor type! Do the results make sense given what you know about the nature of radar and lidar data?
- We give you starter code, but you are not required to use it! You may want to start from scratch if: You want a bigger challenge! You want to redesign the project architecture. There are many valid design patterns for approaching the Kalman Filter algorithm. Feel free to experiment and try your own! You want to use a different coding style, eg. functional programming. While C++ code naturally tends towards being object-oriented in nature, it's perfectly reasonable to attempt a functional approach. Give it a shot and maybe you can improve its efficiency!

## Resources for Completing the Project

The project's [GitHub repository](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project) (included within the workspace) contains all of the files that you will need. The github repository includes:

- starter code in the src folder
- a `README` file with instructions on compiling the code
- a `Docs` folder, which contains details about the structure of the code templates
- `CMakeLists.txt` file that will be used when compiling your code (you do not need to change this file)
- a data file for testing your extended Kalman filter which the simulator interface provides


