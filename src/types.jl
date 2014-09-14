abstract KalmanFilter

abstract LinearKalmanFilter <: KalmanFilter

abstract Model

abstract ObservationModel

type State{T}
    x::Vector{T}
    p::Matrix
end

type LinearModel <: Model
    a::Matrix
    g::Matrix
    q::Matrix
end

type Observation{T}
    y::Vector{T}
end

type LinearObservationModel <: ObservationModel
    h::Matrix
    r::Matrix
end

type BasicKalmanFilter <: LinearKalmanFilter
    x::State
    f::LinearModel
    z::LinearObservationModel
end

type BasicKalmanFilterP <: KalmanFilter
    x1::State
    f::LinearModel
    z::LinearObservationModel
end
