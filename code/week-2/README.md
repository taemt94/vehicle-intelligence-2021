# Week 2 - Markov Localization

---

[//]: # (Image References)
[plot]: ./markov.gif

## Assignment

You will complete the implementation of a simple Markov localizer by writing the following two functions in `markov_localizer.py`:

* `motion_model()`: For each possible prior positions, calculate the probability that the vehicle will move to the position specified by `position` given as input.
* `observation_model()`: Given the `observations`, calculate the probability of this measurement being observed using `pseudo_ranges`.

The algorithm is presented and explained in class.

All the other source files (`main.py` and `helper.py`) should be left as they are.

If you correctly implement the above functions, you expect to see a plot similar to the following:

![Expected Result of Markov Localization][plot]

If you run the program (`main.py`) without any modification to the code, it will generate only the frame of the above plot because all probabilities returned by `motion_model()` are zero by default.


## REPORT
- 이번 과제의 목표는 `Bayes filler algorithm`과 `Markov assumption`을 이용하여 `Markov localization`을 통해 1D 상에서 차량의 위치를 추정하는 것이다.
### 과제 조건
- 총 4그루의 나무(지형지물)가 도로 상에 존재하고, 초기조건으로 차량은 4그루의 나무 중 한 그루의 나무 근처에 위치해있다.
- 차량은 매 스텝마다 1m 씩 이동하며, 차량이 이동한 거리에는 노이즈가 섞여있을 수 있다.
- 차량의 센서는 차량 진행 방향의 지형지물만 감지할 수 있으므로 차량 뒤쪽의 지형지물은 감지하지 못한다.
- 차량의 센서 또한 노이즈가 섞여 있다.
- 이 때 차량이 이동하는 매 스텝마다 차량의 위치를 추정하는 확률 분포를 구한다.

### 과제 설명
- 과제에서 직접 작성해야 할 함수는 `motion_model()`과 `observation_model`이다.
#### `Motion_model()`
- `motion_model()`은 차량의 직전 상태의 위치에 대한 확률과  차량의 이동에 따라 현재 상태의 위치에 대한 예측(Prediction)을 하는 함수이다.
- 작성한 `motion_model()` 함수는 아래와 같다.
  ``` python
  def motion_model(position, mov, priors, map_size, stdev):
      position_prob = 0.0

      for prev_position in range(map_size):
          dist = position - prev_position
          position_prob += norm_pdf(dist, mov, stdev) * priors[prev_position]
          
      return position_prob
  ```
- `position_prob`은 차량의 위치에 대한 예측 확률을 나타낸다.
- `position_prob`은 차량이 위치할 수 있는 모든 위치를 직전 위치(`prev_position`)이라고 했을 때 해당 직전 위치에서 현재 위치(`position`)로 이동할 수 있는 확률을 나타낸다.
- 위에서 `priors[prev_position]`이 직전 위치에 대한 확률을 나타낸다. 
- 또, 과제의 조건에서 직전 위치에서 차량이 이동하여 현재 위치에 도달할 확률은 차량의 현재 위치와 직전 위치의 차이에 대한 정규분포를 따른다고 하였으므로 `norm_pdf()` 함수에 차량의 현재 위치와 직전 위치의 차이인 `dist`를 입력하여 차량의 현재 위치에 대한 확률을 계산하였다.
- 따라서 차량의 현재 위치에 대한 확률(`position_prob`)은 차량의 직전 위치에 대한 확률(`priors[prev_position]`)과 이 때 차량이 직전 위치에서 현재 위치로 이동할 확률(`norm_pdf()`)을 곱한 값을 모든 위치에 대해 합산하여 계산할 수 있다.

#### `Observation_model`
- `Observation_model()`은 매 스텝마다 차량의 센서가 측정한 센서 데이터인 지형지물까지의 거리(`observations`)가 얼마나 정확한지에 대한 확률을 계산하는 함수이다.
- 작성한 `Observation_model()` 함수는 아래와 같다.
  ``` python
  def observation_model(landmarks, observations, pseudo_ranges, stdev):
      distance_prob = 1.0

      if (len(observations) <= len(pseudo_ranges)):
          for i in range(len(observations)):
              distance_prob *= norm_pdf(observations[i], pseudo_ranges[i], stdev ** 2)    
      else:
          distance_prob = 0.0
      return distance_prob
  ```
- 먼저 `observations`의 크기를 통해 `for`문이 동작하게 함으로써 문제의 첫번째 조건을 만족시켰다.
- 문제의 두번째 조건의 경우 `for`문 앞의 조건문을 통해 만족시켰다.
- 두번째 조건에서 `observations`의 길이가 `pseudo_ranges`의 길이보다 긴 경우 `observations`의 값을 전혀 사용할 수 없다고 하였으므로 이 때 `distance_prob`에는 확률값 0.0을 입력하도록 하였다.
- 마지막으로 세번째 조건에서 `observations`에 대한 확률은 평균이 `pseudo_ranges`이고 표준편차가 `1(squared standard deviation of measuremnet)`인 정규분포를 따른다고 하였으므로 norm_pdf() 함수를 사용하여 `observations`에 대한 확률 `distance_prob`을 계산하였다.
- 또한, 문제에서 `observations`가 여러 개일 경우 각각의 확률은 서로 독립이라고 하였으므로 여러 개의 `ovservation`에 대한 확률을 모두 곱하여 `distance_prob`을 구하였다.

### 결과
- 위에서 작성한 `motion_model()` 함수와 `observation_model()` 함수에서 계산한 위치에 대한 예측 확률 `position_prob`과 센서 측정에 대한 확률 `distance_prob`을 곱하면 해당 위치에 차량이 위치하고 있을 확률을 구할 수 있다.
- 따라서 모든 `observations`에 대해 모든 위치에 대한 확률을 각각 구하여 히스토그램을 그리면 위의 `.gif` 이미지와 같다.
- 히스토그램을 보면, 차량이 왼쪽(0)에서 오른쪽(25)으로 이동하고 있음을 예상할 수 있다.
