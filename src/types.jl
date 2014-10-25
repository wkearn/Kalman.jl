abstract KalmanFilter

abstract LinearKalmanFilter <: KalmanFilter

abstract Model

abstract ObservationModel

type State{T}
    x::Vector{T}
    p::Matrix
end

Base.(:(==))(x1::State,x2::State) = x1.x==x2.x && x1.p == x2.p

type LinearModel <: Model
    a::Matrix
    g::Matrix
    q::Matrix
end

type Observation{T}
    y::Vector{T}
end

Base.convert(::Type{Observation},y) = Observation([y])

type LinearObservationModel <: ObservationModel
    h::Matrix
    r::Matrix
end

type BasicKalmanFilter <: LinearKalmanFilter
    x::State
    f::LinearModel
    z::LinearObservationModel
end

Base.copy(kf::KalmanFilter) = deepcopy(kf)
