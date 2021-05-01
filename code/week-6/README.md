# Week 6 - Prediction & Behaviour Planning

---

## Assignment #1

Under the directory [./GNB](./GNB), you are given two Python modules:

* `prediction.py`: the main module you run. The `main()` function does two things: (1) read an input file ([`train.json`](./GNB/train.json)) and train the GNB (Gaussian Naive Bayes) classifier using the data stored in it, and (2) read another input file ([`test.json`](./GNB/test.json)) and make predictions for a number of data points. The accuracy measure is taken and displayed.
* `classifier.py`: main implementation of the GNB classifier. You shall implement two methods (`train()` and `precict()`), which are used to train the classifier and make predictions, respectively.

Both input files ([`train.json`](./GNB/train.json) and [`test.json`](./GNB/test.json)) have the same format, which is a JSON-encoded representation of training data set and test data set, respectively. The format is shown below:

```
{
	"states": [[s_1, d_1, s_dot_1, d_dot_1],
	           [s_2, d_2, s_dot_2, d_dot_2],
	           ...
	           [s_n, d_n, s_dot_n, d_dot_n]
	          ],
	"labels": [L_1, L_2, ..., L_n]
}
```

The array `"states"` have a total of `n` items, each of which gives a (hypothetically) measured state of a vehicle, where `s_i` and `d_i` denote its position in the Frenet coordinate system. In addition, `s_dot_i` and `d_dot_i` give their first derivates, respectively. For each measured state, a label is associated (given in the `"labels"` array) that represents the vehicle's behaviour. The label is one of `"keep"`, `"left"`, and `"right"`, which denote keeping the current lane, making a left turn, and making a right turn, respectively.

The training set has a total of 750 data points, whereas the test set contains 250 data points with the ground truth contained in `"labels"`.

The GNB classifier is trained by computing the mean and variance of each component in the state variable for each observed behaviour. Later it is used to predict the behaviour by computing the Gaussian probability of an observed state for each behaviour and taking the maximum. You are going to implement that functionality. For convcenience, a separate function `gaussian_prob()` is already given in the module `classifier.py`.


---

## Assignment #2

Under the directory [./BP](./BP), you are given four Python modules:

* `simulate_behavior.py`: the main module you run. It instantiates a simple text-based simulation environment and runs it using the configuration specified in the same module.
* `road.py`: `class Road` is implemented here. It captures the state of the simulated road with a number of vehicles (including the ego) running on it, and visualizes it using terminal output.
* `vehicle.py`: `class Vehicle` implements the states of a vehicle and its transition, along with the vehicle's dynamics based on a simple kinematic assumption. Note that a vehicle's trajectory is represented by two instances of object of this class, where the first one gives the current state and the second one predicts the state that the vehicle is going to be in after one timestep.
* `cost_functions.py`: implementation of cost functions governing the state transition of the ego vehicle. The main job required for your assignment is to provide an adequate combination of cost functions by implementing them in this module.

### Task 1

Implement the method `choose_next_state()` in `vehicle.py`. It should

* determine which state transitions are possible from the current state (`successor_states()` function in the same module will be helpful),
* calculate cost for each state transition using the trajectory generated for each behaviour, and
* select the minimum cost trajectory and return it.

Note that you must return a planned trajectory (as described above) instead of the state that the vehicle is going to be in.

### Task 2

In `cost_functions.py`, templates for two different cost functions (`goal_distance_cost()` and `inefficiency_cost()`) are given. They are intended to capture the cost of the trajectory in terms of

* the lateral distance of the vehicle's lane selection from the goal position, and
* the time expected to be taken to reach the goal (because of different lane speeds),

respectively.

Note that the range of cost functions should be carefully defined so that they can be combined by a weighted sum, which is done in the function `calculate_cost()` (to be used in `choose_next_state()` as described above). In computing the weighted sum, a set of weights are used. For example, `REACH_GOAL` and `EFFICIENCY` are already defined (but initialized to zero values). You are going to find out a good combination of weights by an empirical manner.

You are highly encouraged to experiment with your own additional cost functions. In implementing cost functions, a trajectory's summary (defined in `TrajectoryData` and given by `get_helper_data()`) can be useful.

You are also invited to experiment with a number of different simulation settings, especially in terms of

* number of lanes
* lane speed settings (all non-ego vehicles follow these)
* traffic density (governing the number of non-ego vehicles)

and so on.

Remember that our state machine should be geared towards reaching the goal in an *efficient* manner. Try to compare a behaviour that switches to the goal lane as soon as possible (note that the goal position is in the slowest lane in the given setting) and one that follows a faster lane and move to the goal lane as the remaining distance decreases. Observe different behaviour taken by the ego vehicle when different weights are given to different cost functions, and also when other cost metrics (additional cost functions) are used.

### REPORT
#### Assignment 2
- Assignment 2는 text-based의 시뮬레이션을 통해 behaviour planner를 구현하는 assignment이다.
- 해당 과제에서는 FSM transition function(choose_next_state() 메서드)과 다양한 두가지의 cost function을 구현하고 이를 조합하여 total cost가 최소인 FSM을 선택하며 시뮬레이션을 진행하게 된다.
- 각각의 함수에 대한 구현은 아래와 같다.

1. choose_next_state()
   - choose_next_state() 메서드는 현재 차량의 위치에서 이동 가능한 다음 state를 successor_states() 메서드를 통해 구한다.
   - 그리고 이동 가능한 모든 state에 대해 주변 차량들의 이동할 위치에 대한 예측인 predictions을 참고하여 자차의 trajectory를 구하고 이에 대한 cost를 계산한다.
   - 각각의 state에 대한 cost를 costs에 입력한 후, 모든 state에 대해 cost 계산이 완료되면 최소 cost를 가지는 state를 구한다.
   - cost가 최소인 state를 best_next_state로 하여 다음 state를 선택하고 이에 대한 자차의 trajectory를 생성하여 리턴한다.
   ``` python
   def choose_next_state(self, predictions):
       # TODO: implement state transition function based on the cost
       #       associated with each transition.
       possible_successor_states = self.successor_states()
       costs = []
       for state in possible_successor_states:
           trajectory = self.generate_trajectory(state, predictions)
           cost_for_state = calculate_cost(self, trajectory, predictions)
           costs.append({'state': state, 'cost': cost_for_state})

       best_next_state = None
       min_cost = 9999999
       for c in costs:
           if c['cost'] < min_cost:
               min_cost = c['cost']
               best_next_state = c['state']

       next_trajectory = self.generate_trajectory(best_next_state, predictions)
       return next_trajectory
   ```
2. Cost functions
- cost function으로는 goal_distance_cost()와 inefficiency_cost()로 두가지를 사용한다.  
2-1. goal_distance_cost()
  - goal_distance_cost()의 경우 차량이 주행할 때 현재 차량의 lane과 goal_lane에 대한 차이에 대한 cost를 계산한다.
  - 이를 아래의 코드를 통해 구현하였다.
  - 먼저 도착지까지의 거리가 충분히 남은 경우(if goal_dist / vehicle.goal_s > 0.4)에는 goal_distance_cost를 항상 같은 값인 1.0이 되도록 하여 goal_discance_cost가 total cost를 결정하는데 영향을 미치지 못하도록 하였다.
  - 여기서 도착지까지의 남은 거리에 대한 지표로 사용한 값 0.4는 regression tests를 통해서 찾은 값이다.
  - 이렇게 할 경우 차량의 total cost는 inefficiency_cost에 의해서만 결정되어 도착지까지의 거리가 충분히 남은 경우 무조건 fastest lane에서 주행하도록 할 수 있다.
  - 도착지에 점점 가까워지는 경우(elif goal_dist > 0) goal_distance_cost가 차량의 intened_lane과 final_lane이 goal_lane에 가까울수록 작아지도록 하여 goal_distance_cost에 따라 차량이 cost가 작은 방향으로 차선을 바꿀 수 있도록 하였다.
  ``` python
  def goal_distance_cost(vehicle, trajectory, predictions, data):
  intended_lane_dist = vehicle.goal_lane - data[0]
  final_lane_dist = vehicle.goal_lane - data[1]
  goal_dist = data[2]
  if goal_dist / vehicle.goal_s > 0.4:
      cost = 1.0
  elif goal_dist > 0:
      cost = 1 - exp((intended_lane_dist + final_lane_dist) / (goal_dist))
  else:
      cost = 1
  return cost
  ```
2-2. inefficiency cost()
- 다음으로 inefficiency cost()는 차량이 현재 위치한 lane의 속도가 느릴수록 큰 cost를 리턴하는 함수이다.
- goal_distance_cost와 동일하게 도착지까지의 거리가 충분히 남은 경우(if goal_dist / vehicle.goal_s > 0.4)와 그렇지 않은 경우로 나누어 cost를 구하였다.
- 도착지까지의 거리가 충분히 남은 경우에는 intended_lane과 final_lane이 속도가 빠른 lane일수록 cost가 작은 값을 가지도록 하여 항상 속도가 가장 빠른 lane에서 주행하도록 하였다.
- 도착지까지 점점 가까워지는 경우 반대로 intened_lane과 final_lane이 goal_lane과 멀수록 cost가 커지게 하여 cost가 작은 쪽인 goal_lane 쪽으로 차선을 변경할 수 있도록 하였다.
``` python
def inefficiency_cost(vehicle, trajectory, predictions, data):
    intented_lane = data[0]
    final_lane = data[1]
    goal_dist = data[2]
    if goal_dist / vehicle.goal_s > 0.4:
        cost = exp(-(intented_lane + final_lane))
    else:
        cost = 1 - exp(-(intented_lane + final_lane))
    return cost
```