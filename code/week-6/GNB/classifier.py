import numpy as np
import random
from math import sqrt, pi, exp

def gaussian_prob(obs, mu, sig):
    # Calculate Gaussian probability given
    # - observation
    # - mean
    # - standard deviation
    num = (obs - mu) ** 2
    denum = 2 * sig ** 2
    norm = 1 / sqrt(2 * pi * sig ** 2)
    return norm * exp(-num / denum)

# Gaussian Naive Bayes class
class GNB():
    # Initialize classification categories
    def __init__(self):
        self.classes = ['left', 'keep', 'right']
        

    # Given a set of variables, preprocess them for feature engineering.
    def process_vars(self, vars):
        # The following implementation simply extracts the four raw values
        # given by the input data, i.e. s, d, s_dot, and d_dot.
        s, d, s_dot, d_dot = vars

        return s, d, s_dot, d_dot

    # Train the GNB using a combination of X and Y, where
    # X denotes the observations (here we have four variables for each) and
    # Y denotes the corresponding labels ("left", "keep", "right").
    def train(self, X, Y):
        '''
        Collect the data and calculate mean and standard variation
        for each class. Record them for later use in prediction.
        '''
        # TODO: implement code.
        processed_X = [list(self.process_vars(x)) for x in X]

        keep_values = [[0.], [0.], [0.], [0.]]
        right_values = [[0.], [0.], [0.], [0.]]
        left_values = [[0.], [0.], [0.], [0.]]
        for values, label in zip(processed_X, Y):
            if label == 'keep':
                for i, k_v in enumerate(values):
                    keep_values[i].append(k_v)
            elif label == 'right':
                for i, r_v in enumerate(values):
                    right_values[i].append(r_v)
            else:
                for i, l_v in enumerate(values):
                    left_values[i].append(l_v)
        keep_values, right_values, left_values = np.array(keep_values), np.array(right_values), np.array(left_values)
        self.keep_mean = [k_v.mean() for k_v in keep_values]
        self.keep_std = [k_v.std() for k_v in keep_values]

        self.right_mean = [r_v.mean() for r_v in right_values]
        self.right_std = [r_v.std() for r_v in right_values]

        self.left_mean = [l_v.mean() for l_v in left_values]
        self.left_std = [l_v.std() for l_v in left_values]
        
    # Given an observation (s, s_dot, d, d_dot), predict which behaviour
    # the vehicle is going to take using GNB.
    def predict(self, observation):
        '''
        Calculate Gaussian probability for each variable based on the
        mean and standard deviation calculated in the training process.
        Multiply all the probabilities for variables, and then
        normalize them to get conditional probabilities.
        Return the label for the highest conditional probability.
        '''
        # TODO: implement code.
        keep_prob = right_prob = left_prob = 1.
        keep_normalizer = right_normalizer = left_normalizer = 0.

        for i, obs in enumerate(observation):
            keep_prob *= self.gaussian(self.keep_std[i], self.keep_mean[i], obs)
            keep_normalizer += self.gaussian(self.keep_std[i], self.keep_mean[i], obs)

            right_prob *= self.gaussian(self.right_std[i], self.right_mean[i], obs)
            right_normalizer += self.gaussian(self.right_std[i], self.right_mean[i], obs)

            left_prob *= self.gaussian(self.left_std[i], self.left_mean[i], obs)
            left_normalizer += self.gaussian(self.left_std[i], self.left_mean[i], obs)

        keep_prob /= keep_normalizer
        right_prob /= right_normalizer
        left_prob /= left_normalizer
        
        predict_idx = np.argmax([left_prob, keep_prob, right_prob])

        return self.classes[predict_idx]

    def gaussian(self, std, mean, value):
        prob = 1 / sqrt(2 * pi * (std) ** 2) * exp(-(value - mean) ** 2 / (2 * (std) ** 2))
        return prob

