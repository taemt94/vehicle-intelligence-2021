# Week 5 - Path Planning & the A* Algorithm

---

## Examples

We have four small working examples for demonstration of basic path planning algorithms:

* `search.py`: simple shortest path search algorithm based on BFS (breadth first search) - only calculating the cost.
* `path.py`: built on top of the above, generating an optimum (in terms of action steps) path plan.
* `astar.py`: basic implementation of the A* algorithm that employs a heuristic function in expanding the search.
* `policy.py`: computation of the shortest path based on a dynamic programming technique.

These sample source can be run as they are. Explanation and test results are given in the lecture notes.

## Assignment

You will complete the implementation of a simple path planning algorithm based on the dynamic programming technique demonstrated in `policy.py`. A template code is given by `assignment.py`.

The assignmemt extends `policy.py` in two aspects:

* State space: since we now consider not only the position of the vehicle but also its orientation, the state is now represented in 3D instead of 2D.
* Cost function: we define different cost for different actions. They are:
	- Turn right and move forward
	- Move forward
	- Turn left and move forward

This example is intended to illustrate the algorithm's capability of generating an alternative (detouring) path when left turns are penalized by a higher cost than the other two possible actions. When run with the settings defined in `assignment.py` without modification, a successful implementation shall generate the following output,

```
[[' ', ' ', ' ', 'R', '#', 'R'],
 [' ', ' ', ' ', '#', ' ', '#'],
 ['*', '#', '#', '#', '#', 'R'],
 [' ', ' ', ' ', '#', ' ', ' '],
 [' ', ' ', ' ', '#', ' ', ' ']]
```

because of the prohibitively high cost associated with a left turn.

You are highly encouraged to experiment with different (more complex) maps and different cost settings for each type of action.

# Report
- 이번 과제는 dynamic programming을 기반한 경로 탐색 알고리즘을 구현하는 과제이다.
- 강의에서의 dynamic programming 경로 탐색 알고리즘에서 추가된 내용은 차량의 방향(head)을 고려해야 하고, 차량의 action마다 cost가 다르다는 점이다.
- 위를 고려하여 구현한 전체 알고리즘은 아래와 같다.
``` python
def optimum_policy_2D(grid, init, goal, cost):
    value = np.full((4, ) + grid.shape, 999, dtype=np.int32)
    policy = np.full((4,) + grid.shape, -1, dtype=np.int32)
    policy2D = np.full(grid.shape, ' ')

    change = True
    while change:
        change = False
        p = itertools.product(
            range(grid.shape[0]),
            range(grid.shape[1]),
            range(len(forward))
        )
        for y, x, t in p:
            if (y, x) == goal and value[(t, y, x)] > 0:
				# TODO 1.
                value[(t, y, x)] = 0
                policy[(t, y, x)] = 999
                # change = True

            elif grid[(y, x)] == 0:
				# TODO 2.
                for act in action:
                    t2 = (t + act) % 4
                    y2, x2 = y + forward[t2][0], x + forward[t2][1]
                    act_cost = cost[act + 1]
                    if 0 <= y2 < grid.shape[0] and 0 <= x2 < grid.shape[1] and grid[(y2, x2)] == 0:
                        v2 = value[(t2, y2, x2)] + act_cost
                        if v2 < value[(t, y, x)]:
                            value[(t, y, x)] = v2
                            policy[(t, y, x)] = act
                            change = True
	# TODO 3.
    y, x, o = init
    min_value = 999
    while (y, x) != goal:
        for t in range(len(forward)):
            if value[(t, y, x)] < min_value:
                min_value = value[(t, y, x)]
                policy2D[(y, x)] = action_name[policy[(o, y, x)] + 1]

        next_o = (o + policy[o, y, x]) % len(forward)
        next_y, next_x = y + forward[next_o][0], x + forward[next_o][1]
        y, x, o = next_y, next_x, next_o
        if (y, x) == goal:
            policy2D[(y, x)] = '*'
		min_value = 999

    return policy2D
```
---
- TODO에 따라서 알고리즘의 주요 내용을 살펴보면, 먼저 TODO 1은 반복문을 반복하는 과정에서 (y, x)가 도착지인 경우일 때이다.
    ``` python
    if (y, x) == goal and value[(t, y, x)] > 0:
        # TODO 1.
        value[(t, y, x)] = 0
        policy[(t, y, x)] = 999
        change = True
    ```
- (y, x)가 도착지이고, 이 때 value[(t, y, x)] > 0(유효한 값)이면 value[(t, y, x)] = 0을 입력하여 도착지에서의 cost가 0(최소값)이 되도록 하였다.
- policy에는 policy[(t, y, x)] = 999을 입력하였는데, policy는 차량의 action을 나타내는 변수이고 도착지에 도달하였 때 차량이 취할 action은 없으므로 무의미한 값을 나타내도록 999를 입력하였다.
- 또한, value와 policy 변수의 값에 변화가 있으므로 `change = True`를 통해 이후의 while문이 계속해서 반복되도록 하였다.
---
- TODO 2는 `(y, x)`가 도착지가 아닌 도로일 때의 코드를 구현하는 부분이다.
    ``` python
    elif grid[(y, x)] == 0:
        # TODO 2.
        for act in action:
            t2 = (t + act) % 4
            y2, x2 = y + forward[t2][0], x + forward[t2][1]
            act_cost = cost[act + 1]
            if 0 <= y2 < grid.shape[0] and 0 <= x2 < grid.shape[1] and grid[(y2, x2)] == 0:
                v2 = value[(t2, y2, x2)] + act_cost
                if v2 < value[(t, y, x)]:
                    value[(t, y, x)] = v2
                    policy[(t, y, x)] = act
                    change = True
    ```
- for문을 통해 취할 수 있는 모든 action이 반복되도록 하였다.
- for문에서는 먼저 각각의 action을 취할 때 해당 위치 `(y, x)`에서의 차량의 방향이 어떻게 변하는지를 구하여 `t2`에 입력하였다.
- 기존의 방향 `t`에 action `act`를 더한 후 이를 4 방향을 의미하는 4로 나눈 나머지를 `t2`에 입력하였다.
- 예를 들어, `t = 0`으로 차량의 방향이 up일 때 `act = -1`로 차량이 오른쪽으로 회전하면, `t2 = (0 + (-1)) % 4 = 3`이 되어 차량의 방향이 right가 된다.
- 차량의 진행 방향 `t2`를 구하면 `t2` 방향으로 차량이 진행할 것임을 의미하므로, 이를 forward에 인덱싱 값으로 사용하여 진행 후 차량의 위치 `(y2, x2)`를 구하였다.
- `act_cost`는 차량이 해당 action을 취할 때의 cost를 의미한다.
- 이후, 조건문을 통해 차량의 다음 위치 `(y2, x2)`가 유효한 위치라고 판단될 경우 value와 policy를 업데이트한다.
- 먼저 `(y2, x2)`에서의 cost 값 `value[(t2, y2, x2)]`와 `act_cost`를 합하여 `v2`에 입력한다.
- 그리고 `v2 < value[(t, y, x)]`인 경우 차량이 이동할 때 (y2, x2)에서 (y, x)로 이동하는 것이 cost가 더 작은 것을 의미하므로 `v2`와 이 때의 action `act`를 `value[(t, y, x)]`와 `policy[(t, y, x)]`에 각각 입력한다.
- 또한, value와 policy 변수의 값에 변화가 있으므로 `change = True`를 통해 이후의 while문이 계속해서 반복되도록 하였다.
--- 
- TODO 3은 경로 탐색이 완료된 후 `policy2D`가 2차원 상에서 차량의 이동 경로 및 action을 나타내도록 하는 코드이다.
- 먼저 `(y, x, o)`에 시작지 `init`을 입력하여 시작지부터 `policy2D`가 입력되도록 하였다.
- while문은 (y, x)가 도착지가 될 때까지 반복되도록 하였다.
- 경로 탐색의 경우 각 좌표에서의 cost인 `value[(t, y, x)]`가 최소인 경로를 탐색하므로, 각각의 좌표 (y, x)에서 cost가 최소인 값을 찾아 `min_value`에 입력되도록 하였고, 이 때 해당하는 action의 `action_name`을 `policy`로부터 찾아 `policy2D`에 입력하였다.
- 이후 현재 좌표 `(y, x, o)`에서 이동할 좌표 `(next_y, next_x, next_o)`를 구하여 다시 `(y, x, o)`에 대입한 후 while문이 반복되도록 하였다.
- 마지막으로 좌표가 도착지(`goal`)에 도달하였을 경우에는 `policy2D`에 '*'을 입력하도록 하였다.
``` python
# TODO 3.
y, x, o = init
min_value = 999
while (y, x) != goal:
    for t in range(len(forward)):
        if value[(t, y, x)] < min_value:
            min_value = value[(t, y, x)]
            policy2D[(y, x)] = action_name[policy[(o, y, x)] + 1]

    next_o = (o + policy[o, y, x]) % len(forward)
    next_y, next_x = y + forward[next_o][0], x + forward[next_o][1]
    y, x, o = next_y, next_x, next_o
    if (y, x) == goal:
        policy2D[(y, x)] = '*'
    min_value = 999
```

- 아래는 위의 알고리즘을 통해 `cost = (2, 1, 20)`일 때와 `cost = (2, 1, 2)`일 때 실행한 결과를 각각 나타낸 것이다.
- 실행 결과를 보면 `cost = (2, 1, 20)`일 때는 왼방향 회전 후 직진 시 cost가 매우 크므로 cost가 작은 오른방향 회전 후 직진을 하는 경로를 선택하는 반면, `cost = (2, 1, 2)`일 때는 왼방향 회전 후 직진 시에도 cost가 작으므로 왼방향 회전하는 경로를 택한 것을 알 수 있다.
	- cost = (2, 1, 20)
	  ```
	  $ python .\assignment.py
        [[' ' ' ' ' ' 'R' '#' 'R']
         [' ' ' ' ' ' '#' ' ' '#']
         ['*' '#' '#' '#' '#' 'R']
         [' ' ' ' ' ' '#' ' ' ' ']
         [' ' ' ' ' ' '#' ' ' ' ']]
	  ```
	- cost = (2, 1, 2)
	  ```
      $ python .\assignment.py
        [[' ' ' ' ' ' ' ' ' ' ' ']
         [' ' ' ' ' ' ' ' ' ' ' ']
         ['*' '#' '#' 'L' ' ' ' ']
         [' ' ' ' ' ' '#' ' ' ' ']
         [' ' ' ' ' ' '#' ' ' ' ']] 
      ```