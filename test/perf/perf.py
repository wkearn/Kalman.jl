import numpy as np
import pylab as pl
import sys
import timeit
from pykalman import KalmanFilter

N = int(sys.argv[1])
random_state = np.random.RandomState(0)
transition_matrix = [[1, 0.01], [-0.01, 1]]
transition_offset = [0.0,0.0]
observation_matrix = [1.0,0]
observation_offset = [0.0]
transition_covariance = 1e-10*np.eye(2)
observation_covariance = [0.1]
initial_state_mean = [1.0,0.0]
initial_state_covariance = [[1,0.1],[-0.1,1]]
kf = KalmanFilter(
    transition_matrices=transition_matrix,observation_matrices=observation_matrix, transition_covariance=transition_covariance,
    observation_covariance=observation_covariance, transition_offsets=transition_offset, observation_offsets=observation_offset,
    initial_state_mean=initial_state_mean, initial_state_covariance=initial_state_covariance,
    random_state=random_state
)

ts = np.linspace(0,0.01*1000,1000)
observations = np.cos(ts) + np.sqrt(0.1) * random_state.randn(1000)
states = np.cos(ts)

t = timeit.timeit('filtered_state_estimates = kf.filter(observations)[0]','from __main__ import kf,observations',number=N)

print t
