module EKF

using Kalman
using ForwardDiff

import Kalman.predict, Kalman.update, Kalman.predictupdate

export ExtendedKalmanFilter,
NonlinearModel,
NonlinearObservationModel,
BasicExtendedKalmanFilter

abstract ExtendedKalmanFilter <: KalmanFilter

type NonlinearModel <: Model
    f::Function
    j::Function
    g::Function
    q::Matrix
    function NonlinearModel(f::Function,g::Function,q::Matrix)
        j = forwarddiff_jacobian(f,Float64,fadtype=:typed)
        new(f,j,g,q)
    end
end

type NonlinearObservationModel <: ObservationModel
    h::Function
    j::Function
    r::Matrix
    function NonlinearObservationModel(h::Function,r::Matrix)
        j = forwarddiff_jacobian(h,Float64,fadtype=:typed)
        new(h,j,r)
    end
end

type BasicExtendedKalmanFilter <: ExtendedKalmanFilter
    x::State
    f::NonlinearModel
    z::NonlinearObservationModel
end

type BasicExtendedKalmanFilterP <: ExtendedKalmanFilter
    x::State
    f::NonlinearModel
    z::NonlinearObservationModel
end

function predict(kf::BasicExtendedKalmanFilter)
    x1 = ap(kf.f,kf.x)
    BasicExtendedKalmanFilterP(x1,kf.f,kf.z)
end

function ap(f::NonlinearModel,x::State)
    x1 = f.f(x.x)
    F = f.j(x.x)
    G = f.g(x.x)
    p1 = F*x.p*F' + G*f.q*G'
    State(x1,p1)
end

function update(kf::BasicExtendedKalmanFilterP,y::Observation)
    res = y.y - kf.z.h(kf.x.x)
    H = kf.z.j(kf.x.x)'
    s = H * kf.x.p * H' + kf.z.r
    k = kf.x.p * H' * inv(s)
    xn = kf.x.x + k * res
    pn = kf.x.p - k * H * kf.x.p
    BasicExtendedKalmanFilter(State(xn,pn),kf.f,kf.z)
end

function predictupdate(kf::BasicExtendedKalmanFilter,y::Observation)
    update(predict(kf),y)
end

end # module
