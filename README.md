## Extended Kalman Filter
[![C++](https://img.shields.io/badge/C++-Solutions-blue.svg?style=flat&logo=c%2B%2B)](https://www.python.org/downloads/release/python-360/)
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

In this project we utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. For validation position (px, py) and velocity (vx, vy) of the tracking object will be compared with their ground truth using RMSE.

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases). This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page for the required version and installation scripts.

<p align="center"> 
<img src="https://github.com/ajimenezjulio/P5_Extended_Kalman_Filter/blob/master/Docs/tracking.gif">
</p>

### Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)


### Build
Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.
```
$> mkdir build
$> cd build
$> cmake ..
$> make
$> ./ExtendedKF
```

### Implementation

Below the skeleton of the main program under the `src` directory.
```
.
├── Eigen
    ├── ...
├── FusionEKF.cpp
├── FusionEKF.h
├── json.hpp
├── kalman_filter.cpp
├── kalman_filter.h
├── main.cpp
├── measurement_package.h
├── tools.cpp
└── tools.h
```

The main changes were to the folowing files:

- `main.cpp` - Reads in data, runs the Kalman filter and calculates RMSE values after each measurement.
- `FusionEKF.cpp` - Initializes the filter, calls the `Predict` function and the `Update` function
- `kalman_filter.cpp`- Implementation of the `Predict` and `Update` function, for both lidar and radar.
- `tools.cpp` - Tool functions to calculate `RMSE` and the `Jacobian` matrix.
- `Eigen` - Math helper library for matrices oprations.

### Results

To asses the project RMSE values should met the next criteria. `RMSE < 0.11` for `Px` and `Py` and `RMSE < 0.52` for `Vx` and `Vy`.

| RMSE | Dataset 1 | Dataset 2 |
|------|-----------|-----------|
| Px  |  0.0979   |  0.0726    |
| Py  |  0.0864   |  0.0965    |
| Vx  |  0.4568   |  0.4216    |
| Vy  |  0.4458   |  0.4932    | 

### Deep down into the implementation

Kalman filters consists of two steps:

**Prediction**
    <p align="center" style="text-align: center;"><img align="center" src="https://tex.s2cms.ru/svg/%0A%5Cbegin%7Balign*%7D%20%0Ax'%20%26%3D%20Fx%2B%5Cnu%20%5C%5C%0AP'%20%26%3D%20FPF%5ET%20%2B%20Q%20%5C%5C%0Awhere%20%5C%3B%20F%20%26%3D%20Motion%20%5C%20model%20%3D%20%0A%5Cbegin%7Bpmatrix%7D%201%20%26%200%20%26%20%5CDelta%20t%20%26%200%20%5C%5C%5C%200%20%26%201%20%26%200%20%26%20%5CDelta%20t%20%5C%5C%5C%200%20%26%200%20%26%201%20%26%200%20%5C%5C%5C%200%20%26%200%20%26%200%20%26%201%20%5Cend%7Bpmatrix%7D%5C%5C%0Aand%20%5C%20Q%20%26%3D%20Covariance%20%5C%20matrix%20%3D%20GQ_%5Cnu%20G%5ET%20%3D%20%5Cbegin%7Bpmatrix%7D%20%5Cfrac%7B%5CDelta%20t%5E4%7D%7B%7B4%7D%7D%5Csigma_%7Bax%7D%5E2%20%26%200%20%26%20%5Cfrac%7B%5CDelta%20t%5E3%7D%7B%7B2%7D%7D%5Csigma_%7Bax%7D%5E2%20%26%200%20%5C%5C%200%20%26%20%5Cfrac%7B%5CDelta%20t%5E4%7D%7B%7B4%7D%7D%5Csigma_%7Bay%7D%5E2%20%26%200%20%26%20%5Cfrac%7B%5CDelta%20t%5E3%7D%7B%7B2%7D%7D%5Csigma_%7Bay%7D%5E2%20%5C%5C%20%5Cfrac%7B%5CDelta%20t%5E3%7D%7B%7B2%7D%7D%5Csigma_%7Bax%7D%5E2%20%26%200%20%26%20%5CDelta%20t%5E2%5Csigma_%7Bax%7D%5E2%20%26%200%20%5C%5C%200%20%26%20%5Cfrac%7B%5CDelta%20t%5E3%7D%7B%7B2%7D%7D%5Csigma_%7Bay%7D%5E2%20%26%200%20%26%20%5CDelta%20t%5E2%5Csigma_%7Bay%7D%5E2%20%5Cend%7Bpmatrix%7D%0A%5Cend%7Balign*%7D%0A" alt="
\begin{align*} 
x' &amp;= Fx+\nu \\
P' &amp;= FPF^T + Q \\
where \; F &amp;= Motion \ model = 
\begin{pmatrix} 1 &amp; 0 &amp; \Delta t &amp; 0 \\\ 0 &amp; 1 &amp; 0 &amp; \Delta t \\\ 0 &amp; 0 &amp; 1 &amp; 0 \\\ 0 &amp; 0 &amp; 0 &amp; 1 \end{pmatrix}\\
and \ Q &amp;= Covariance \ matrix = GQ_\nu G^T = \begin{pmatrix} \frac{\Delta t^4}{{4}}\sigma_{ax}^2 &amp; 0 &amp; \frac{\Delta t^3}{{2}}\sigma_{ax}^2 &amp; 0 \\ 0 &amp; \frac{\Delta t^4}{{4}}\sigma_{ay}^2 &amp; 0 &amp; \frac{\Delta t^3}{{2}}\sigma_{ay}^2 \\ \frac{\Delta t^3}{{2}}\sigma_{ax}^2 &amp; 0 &amp; \Delta t^2\sigma_{ax}^2 &amp; 0 \\ 0 &amp; \frac{\Delta t^3}{{2}}\sigma_{ay}^2 &amp; 0 &amp; \Delta t^2\sigma_{ay}^2 \end{pmatrix}
\end{align*}
" /></p>

**Update**
    <p align="center" style="text-align: center;"><img align="center" src="https://tex.s2cms.ru/svg/%0A%5Cbegin%7Balign*%7D%20%0Ay%20%26%3D%20z%20-%20Hx'%20%5C%5C%0AS%20%26%3D%20HP'H%5ET%2BR%20%5C%5C%0AK%20%26%3D%20P'H%5ET%20S%5E%7B-1%7D%20%5C%5C%0Ax%20%26%3D%20x'%20%2B%20Ky%20%5C%5C%0AP%20%26%3D%20(I%20-%20KH)P'%5C%5C%20%5C%5C%0Awhere%20%5C%20z%20%26%3D%20Measurement%20%5C%20function%2C%20%5C%5C%0AR%20%26%3D%20Measurement%20%5C%20noise%20%5C%20covariance%20%5C%20matrix%20%5C%5C%0Aand%20%5C%20%26matrix%20%5C%20H%20%5C%20for%20%5C%20radar%20%5C%20must%20%5C%20be%20%5C%20linearized%2C%20%5C%5C%0Aso%20%5C%20%26Jacobian%20%5C%20matrix%20%5C%20must%20%5C%20be%20%5C%20calculated%20%5C%5C%0AH_j%20%26%3D%20%5Cbegin%7Bbmatrix%7D%20%5Cfrac%7B%5Cpartial%20%5Crho%7D%7B%5Cpartial%20p_x%7D%20%26%20%5Cfrac%7B%5Cpartial%20%5Crho%7D%7B%5Cpartial%20p_y%7D%20%26%20%5Cfrac%7B%5Cpartial%20%5Crho%7D%7B%5Cpartial%20v_x%7D%20%26%20%5Cfrac%7B%5Cpartial%20%5Crho%7D%7B%5Cpartial%20v_y%7D%5C%5C%20%5Cfrac%7B%5Cpartial%20%5Cvarphi%7D%7B%5Cpartial%20p_x%7D%20%26%20%5Cfrac%7B%5Cpartial%20%5Cvarphi%7D%7B%5Cpartial%20p_y%7D%20%26%20%5Cfrac%7B%5Cpartial%20%5Cvarphi%7D%7B%5Cpartial%20v_x%7D%20%26%20%5Cfrac%7B%5Cpartial%20%5Cvarphi%7D%7B%5Cpartial%20v_y%7D%5C%5C%20%5Cfrac%7B%5Cpartial%20%5Cdot%7B%5Crho%7D%7D%7B%5Cpartial%20p_x%7D%20%26%20%5Cfrac%7B%5Cpartial%20%5Cdot%7B%5Crho%7D%7D%7B%5Cpartial%20p_y%7D%20%26%20%5Cfrac%7B%5Cpartial%20%5Cdot%7B%5Crho%7D%7D%7B%5Cpartial%20v_x%7D%20%26%20%5Cfrac%7B%5Cpartial%20%5Cdot%7B%5Crho%7D%7D%7B%5Cpartial%20v_y%7D%20%5Cend%7Bbmatrix%7D%0A%5Cend%7Balign*%7D%0A" alt="
\begin{align*} 
y &amp;= z - Hx' \\
S &amp;= HP'H^T+R \\
K &amp;= P'H^T S^{-1} \\
x &amp;= x' + Ky \\
P &amp;= (I - KH)P'\\ \\
where \ z &amp;= Measurement \ function, \\
R &amp;= Measurement \ noise \ covariance \ matrix \\
and \ &amp;matrix \ H \ for \ radar \ must \ be \ linearized, \\
so \ &amp;Jacobian \ matrix \ must \ be \ calculated \\
H_j &amp;= \begin{bmatrix} \frac{\partial \rho}{\partial p_x} &amp; \frac{\partial \rho}{\partial p_y} &amp; \frac{\partial \rho}{\partial v_x} &amp; \frac{\partial \rho}{\partial v_y}\\ \frac{\partial \varphi}{\partial p_x} &amp; \frac{\partial \varphi}{\partial p_y} &amp; \frac{\partial \varphi}{\partial v_x} &amp; \frac{\partial \varphi}{\partial v_y}\\ \frac{\partial \dot{\rho}}{\partial p_x} &amp; \frac{\partial \dot{\rho}}{\partial p_y} &amp; \frac{\partial \dot{\rho}}{\partial v_x} &amp; \frac{\partial \dot{\rho}}{\partial v_y} \end{bmatrix}
\end{align*}
" /></p>
