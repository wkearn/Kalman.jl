abstract KalmanFilter

abstract LinearKalmanFilter <: KalmanFilter

abstract Model

abstract ObservationModel

abstract AbstractState

type State{T} <: AbstractState
    x::Vector{T}
    p::Matrix
end

Base.(:(==))(x1::State,x2::State) = x1.x==x2.x && x1.p == x2.p

type LinearModel <: Model
    a::Matrix
    g::Matrix
    q::Matrix
end

## This is the first function a Kalman filter needs to 
# implement: Apply its model to its state to generate a
# new state

function ap(f::LinearModel,x::State)
    x1 = f.a*x.x
    p1 = f.a*x.p*f.a' + f.g*f.q*f.g'
    State(x1,p1)
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


## This is the second of two functions a Kalman filter needs to implement
# Return three matrices: 
# 1. The residual (y-Hx) (`res`)
# 2. The cross-covariance between state and measurement (`ph`)
# 3. The innovation covariance matrix
function covs(kf::BasicKalmanFilter,y::Observation)
    res = y.y - kf.z.h * kf.x.x
    ph = kf.x.p * kf.z.h'
    s = kf.z.h * kf.x.p * kf.z.h' + kf.z.r
    (res,ph,s)
end
