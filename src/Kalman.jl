module Kalman

using Distributions

include("types.jl")

export State, Observation, KalmanFilter, LinearKalmanFilter, BasicKalmanFilter, Model, ObservationModel, LinearModel, LinearObservationModel

end # module
