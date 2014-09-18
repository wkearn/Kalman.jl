module Kalman

export State, 
Observation, 
KalmanFilter, 
LinearKalmanFilter, 
BasicKalmanFilter, 
Model, 
ObservationModel, 
LinearModel, 
LinearObservationModel, 
predict, 
update,
predictupdate,
predict!,
update!,
predictupdate!,
BasicExtendedKalmanFilter,
EKF

include("types.jl")
include("filter.jl")
include("ekf.jl")

using .EKF

end # module
