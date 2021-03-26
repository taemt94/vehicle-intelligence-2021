# Week 3 - Kalman Filters, EKF and Sensor Fusion

---

[//]: # (Image References)
[kalman-result]: ./kalman_filter/graph.png
[EKF-results]: ./EKF/plot.png

## Kalman Filter Example

In directory [`./kalman_filter`](./kalman_filter), a sample program for a small-scale demonstration of a Kalman filter is provided. Run the following command to test:

```
$ python testKalman.py
```

This program consists of four modules:

* `testKalman.py` is the module you want to run; it initializes a simple Kalman filter and estimates the position and velocity of an object that is assumed to move at a constant speed (but with measurement error).
* `kalman.py` implements a basic Kalman fitler as described in class.
* `plot.py` generates a plot in the format shown below.
* `data.py` provides measurement and ground truth data used in the example.

The result of running this program with test input data is illustrated below:

![Testing of Kalman Filter Example][kalman-result]

Interpretation of the above results is given in the lecture.

In addition, you can run `inputgen.py` to generate your own sample data. It will be interesting to experiment with a number of data sets with different characteristics (mainly in terms of variance, i.e., noise, involved in control and measurement).

---

## Assignment - EFK & Sensor Fusion Example

In directory [`./EKF`](./EKF), template code is provided for a simple implementation of EKF (extended Kalman filter) with sensor fusion. Run the following command to test:

```
$ python run.py
```

The program consists of five modules:

* `run.py` is the modele you want to run. It reads the input data from a text file ([data.txt](./EKF/data.txt)) and feed them to the filter; after execution summarizes the result using a 2D plot.
* `sensor_fusion.py` processees measurements by (1) adjusting the state transition matrix according to the time elapsed since the last measuremenet, and (2) setting up the process noise covariance matrix for prediction; selectively calls updated based on the measurement type (lidar or radar).
* `kalman_filter.py` implements prediction and update algorithm for EKF. All the other parts are already written, while completing `update_ekf()` is left for assignment. See below.
* `tools.py` provides a function `Jacobian()` to calculate the Jacobian matrix needed in our filter's update algorithm.
*  `plot.py` creates a 2D plot comparing the ground truth against our estimation. The following figure illustrates an example:

![Testing of EKF with Sensor Fusion][EKF-results]

### Assignment

Complete the implementation of EKF with sensor fusion by writing the function `update_ekf()` in the module `kalman_filter`. Details are given in class and instructions are included in comments.


### Report
- EKF(Extended Kalman Filter)는 칼만 필터가 선형 시스템에서만 사용할 수 있다는 한계점을 극복하기 위한 알고리즘이다.
- 비선형 함수를 선형 함수로 근사화함으로써 칼만 필터 알고리즘을 적용할 수 있다.
- 아래의 코드는 EKF 알고리즘을 구현하기 위해 추가한 부분이다.
``` python
def update_ekf(self, z):
        # TODO: Implement EKF update for radar measurements
        # 1. Compute Jacobian Matrix H_j
        H_j = Jacobian(self.x)
        # 2. Calculate S = H_j * P' * H_j^T + R
        S = np.dot(np.dot(H_j, self.P), H_j.T) + self.R
        # 3. Calculate Kalman gain K = H_j * P' * Hj^T + R
        K = np.dot(np.dot(self.P, H_j.T), np.linalg.inv(S))
        # 4. Estimate y = z - h(x')
        px, py, vx, vy = self.x
        rho_est = sqrt(px ** 2 + py ** 2)
        phi_est = atan2(py, px)
        rho_dot_est = (px * vx + py * vy) / sqrt(px ** 2 + py ** 2)
        y = z - np.array([rho_est, phi_est, rho_dot_est], dtype=np.float32)
        # 5. Normalize phi so that it is between -PI and +PI
        phi = y[1]
        while phi > pi:
            phi -= 2 * pi
        while phi < -pi:
            phi += 2 * pi
        y[1] = phi
        # 6. Calculate new estimates
        #    x = x' + K * y
        #    P = (I - K * H_j) * P
        self.x = self.x + np.dot(K, y)
        self.P = self.P - np.dot(np.dot(K, H_j), self.P)
```
1. Compute Jacobian Matrix H_j
   - `Jacobian()` 함수를 사용하여 예측한 state variable의 Jacobian을 계산한다.
2. Calculate S = H_j * P' * H_j^T + R
   - 칼만 게인 K를 계산하기 위해 필요한 변수인 S를 자코비안 H_j, 예측 공분산 P, measurement error에 대한 공분산 R을 사용하여 계산한다.
3. Calculate Kalman gain K = H_j * P' * Hj^T + R
   - 공분산 P, Jacobian H_j, S를 사용하여 칼만 게인 K를 계산한다.
4. Estimate y = z - h(x')
   - Radar가 측정한 measurement 변수 z와 예측한 state variable로부터 구한 measurement 값의 차이 y를 계산한다.
5. Normalize phi so that it is between -PI and +PI
   - measurement 변수는 rho, phi, rho_dot으로 구성되어 있는데, 여기서 phi는 각도(단위: 라디안)이고 각도의 경우 모든 값을 -pi와 pi 사이의 값을 가지도록 할 수 있다.
   - 위의 두개의 while문을 사용하면, -pi와 pi 사이에 있지 않은 각도를 -pi와 pi에 해당하는 각도로 변환한다.
   - 예를 들어, phi가 181도인 경우 첫번째 while문을 통해 -179도로 변환된다.
   - 또 phi가 -181도인 경우 두번째 while문을 통해 +179도로 변환된다.
6. Calculate new estimates
   - 칼만 게인 K, measurement 차이 y 및 예측한 state variable x를 사용하여 새로운 예측 state variable x를 계산한다.
   - 또한 예측 공분산 P, 칼만 게인 K, Jacobian H_j를 사용하여 새로운 예측 공분산 P를 계산한다.
- 위의 과정을 반복하면 예측한 state variable x와 센서의 measurement z를 사용하여 매 time step마다 물체의 위치 예측값을 갱신할 수 있다.