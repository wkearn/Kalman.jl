module Kalman

export State,
UnscentedState,
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
ExtendedKalmanFilter,
AdditiveUnscentedKalmanFilter,
UnscentedKalmanFilter,
AdditiveUnscentedModel,
AdditiveUnscentedObservationModel

include("types.jl")
include("filter.jl")
include("ekf.jl")
include("unscented.jl")

# Nullable Observations work but only for Julia v0.4
if VERSION >= v"0.4.0-"
    include("nullable.jl")
end

end # module
