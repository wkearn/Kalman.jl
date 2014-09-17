module Kalman

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

include("types.jl")
include("filter.jl")
include("ekf.jl")

end # module
