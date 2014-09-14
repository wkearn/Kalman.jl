module EKF

abstract ExtendedKalmanFilter <: KalmanFilter

type NonlinearModel <: Model
    f::Function
    g::Function
    q::Matrix
end

type NonlinearObservationModel <: ObservationModel
    h::Function
    r::Matrix
end

type BasicExtendedKalmanFilter <: ExtendedKalmanFilter
    x::State
    f::NonlinearModel
    z::NonlinearObservationModel
end

end
