# Week 4 - Motion Model & Particle Filters

---

[//]: # (Image References)
[empty-update]: ./empty-update.gif
[example]: ./example.gif

## Assignment

You will complete the implementation of a simple particle filter by writing the following two methods of `class ParticleFilter` defined in `particle_filter.py`:

* `update_weights()`: For each particle in the sample set, calculate the probability of the set of observations based on a multi-variate Gaussian distribution.
* `resample()`: Reconstruct the set of particles that capture the posterior belief distribution by drawing samples according to the weights.

To run the program (which generates a 2D plot), execute the following command:

```
$ python run.py
```

Without any modification to the code, you will see a resulting plot like the one below:

![Particle Filter without Proper Update & Resample][empty-update]

while a reasonable implementation of the above mentioned methods (assignments) will give you something like

![Particle Filter Example][example]

Carefully read comments in the two method bodies and write Python code that does the job.

---
## Report
- 이번 과제는 particle filter를 구현해보는 과제이다.
- update_weights() 함수와 resample() 함수를 task에 따라 구현하였고, 이에 대한 설명은 아래와 같다.
    ``` python
    def update_weights(self, sensor_range, std_landmark_x, std_landmark_y, observations, map_landmarks):
        # TODO: For each particle, do the following:
        # 1. Select the set of landmarks that are visible
        #    (within the sensor range).
        for p in self.particles:
            visible_landmarks = []
            for id, landmark in map_landmarks.items():
                if distance(p, landmark) < sensor_range:
                    visible = {'x': landmark['x'], 'y': landmark['y'], 'id': id}
                    visible_landmarks.append(visible)
    ```
- 먼저 update_(weights)에 대해 살펴보면, 1번에서는 랜드마크 중에서 각각의 파티클의 현재 위치에서 보일 수 있는 랜드마크를 찾아야 한다.
- 이 때 distance() 함수를 사용해 파티클에서 각 랜드마크까지의 거리를 구하였고, 이 거리가 senser_range보다 작은 랜드마크만 visible_landmarks에 append하였다.
    ``` python
        # 2. Transform each observed landmark's coordinates from the
        #    particle's coordinate system to the map's coordinates.
            transformed_observations = []
            for obs in observations:
                tmp_obs = {}
                tmp_obs['x'] = p['x'] + (obs['x'] * np.cos(p['t'])) - \
                                        (obs['y'] * np.sin(p['t']))
                tmp_obs['y'] = p['y'] + (obs['x'] * np.sin(p['t'])) + \
                                        (obs['y'] * np.cos(p['t']))
                transformed_observations.append(tmp_obs)

            if len(visible_landmarks) == 0:
                continue
    ```
- 2번은 observations의 좌표계를 차량의 좌표계에서 절대 좌표계로 변환해주는 task이다.
- 파티클의 위치값과 observations의 좌표 값을 사용하여 좌표계를 변환하였고 이를 transformed_observations에 append하였다.
    ``` python
        # 3. Associate each transformed observation to one of the
        #    predicted (selected in Step 1) landmark positions.
        #    Use self.associate() for this purpose - it receives
        #    the predicted landmarks and observations; and returns
        #    the list of landmarks by implementing the nearest-neighbour
        #    association algorithm.
            assoc_landmarks = self.associate(visible_landmarks, transformed_observations)
            p['assoc'] = [landmark['id'] for landmark in assoc_landmarks]
    ```
- 3번 task는 파티클에서 감지 가능한 랜드마크들이 실제 어떤 랜드마크인지를 associate하는 task이다.
- associate() 메서드를 사용하여 associate된 랜드마크를 assoc_landmarks에 입력하였다.
- 또한 각 파티클의 키 'assoc'의 value에는 assoc_landmarks의 각 랜드마크 id를 리스트로 입력해주었다.
- 랜드마크 id를 value로 입력해주는 이유는 알고리즘에서 get_best_particle() 함수를 통해 리턴된 파티클의 associate된 랜드마크를 plot하는데 사용하기 위해서이다.

    ``` python
        # 4. Calculate probability of this set of observations based on
        #    a multi-variate Gaussian distribution (two variables being
        #    the x and y positions with means from associated positions
        #    and variances from std_landmark_x and std_landmark_y).
        #    The resulting probability is the product of probabilities
        #    for all the observations.
            for t, a in zip(transformed_observations, assoc_landmarks):
                g_x = norm_pdf(t['x'], a['x'], std_landmark_x)
                g_y = norm_pdf(t['y'], a['y'], std_landmark_y)
                gaussian = g_x * g_y
    ```
- 4번은 assoc_landmarks의 각 랜드마크 좌표가 평균, std_landmark_x 및 std_landmark_y가 표준편차일 때 transformed_observations의 각 랜드마크가 해당 위치에 존재할 확률을 multi-variate Gaussian distribution으로 계산하는 task이다.
- 이 때 두 변수인 x 좌표와 y 좌표는 서로 독립이라고 가정하여 각 좌표마다 따로 Gaussian distribution 확률값을 구해주었다.
- 확률을 계산할 때에는 week 2에서 사용하였던 norm_pdf() 함수를 사용하였고, helpers.py에 작성하였다.
- x 좌표와 y 좌표에 대해 구한 확률을 곱하여(서로 독립이므로) 전체 확률 값 gaussian을 계산하였다. 
    ``` python
        # 5. Update the particle's weight by the calculated probability.
                p['w'] *= gaussian
    ```
- 마지막으로 5번 task에서는 위에서 구한 확률값 gaussian을 transformed_observations의 모든 랜드마크에 대해 구하여 p['w']에 weight로 곱하도록 하였다.
---
- 두번째 함수인 resample()에 대한 설명은 아래와 같다.
    ``` python
    def resample(self):
        # TODO: Select (possibly with duplicates) the set of particles
        #       that captures the posteior belief distribution, by
        # 1. Drawing particle samples according to their weights.
        weights = [p['w'] for p in self.particles]
        w_cumsum = np.cumsum(weights)
        w_mean = np.sum(weights) / len(weights)
        weight_idx = np.zeros(len(weights), dtype=np.int8)
        
        w_pointer = 0.0
        i = w_idx = 0
        while i > len(weights):
            if w_cumsum[w_idx] >= w_pointer:
                weight_idx[i] = w_idx
            else:
                weight_idx[i] = w_idx
                w_idx += 1
            w_pointer += w_mean
            i += 1
            
        new_particles = [self.particles[i].copy() for i in weight_idx]
    ```
- 먼저 1번 task는 모든 파티클이 가지고 있는 weight에 따라서 파티클들을 weighted sampling을 하는 task이다.
- 따라서 weight가 큰 파티클은 여러번 sampling되어야 하고 weight가 작은 파티클은 적은 수로 sampling되도록 구현하여야 한다.
- 위의 코드에서 이를 구현한 부분은 while 문인데, while 문을 통해 weight_idx에 weight가 큰 파티클의 인덱스는 여러번 입력하고, weight가 작은 파티클의 인덱스는 적은 수로 입력하거나 또는 해당 파티클을 건너 뛰어 아예 입력하지 않도록 하였다.
- weight_idx에 입력된 파티클의 인덱스를 사용하여 new_particles에 파티클을 weighted sampling하여 입력하였다.
    ``` python
        # 2. Make a copy of the particle; otherwise the duplicate particles
        #    will not behave independently from each other - they are
        #    references to mutable objects in Python.
        #    Finally, self.particles shall contain the newly drawn set of
        #    particles.
        self.particles = []
        self.particles = new_particles.copy()

        return
    ```
- 마지막으로 2번 task에서는 resample된 파티클을 copy() 함수를 사용하여 다시 self.particles에 입력해주었다.
- 위의 코드를 실행하였을 때 particle filter가 잘 동작함을 확인할 수 있었다.