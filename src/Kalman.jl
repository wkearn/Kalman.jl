module Kalman

using Distributions

include("types.jl")
include("filter.jl")


export State, 
Observation, 
KalmanFilter, 
LinearKalmanFilter, 
BasicKalmanFilter, 
BasicKalmanFilterP,
Model, 
ObservationModel, 
LinearModel, 
LinearObservationModel, 
predict, 
update,
predictupdate,
EKF

include("ekf.jl")

end # module
