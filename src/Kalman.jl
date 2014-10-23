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
NonlinearModel,
NonlinearObservationModel,
ExtendedKalmanFilter

include("types.jl")
include("filter.jl")
include("ekf.jl")

# Nullable Observations work but only for Julia v0.4
if VERSION >= v"0.4.0-"
    include("nullable.jl")
end

end # module
