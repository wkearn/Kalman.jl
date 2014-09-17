module Kalman

using Distributions

include("types.jl")
include("filter.jl")

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
predictupdate

end # module
