using ForwardDiff

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

function ap(f::NonlinearModel,x::State)
    x1 = f.f(x.x)
    F = f.j(x.x)
    G = f.g(x.x)
    p1 = F*x.p*F' + G*f.q*G'
    State(x1,p1)
end

function covs(kf::BasicExtendedKalmanFilter,y::Observation)
    res = y.y - kf.z.h(kf.x.x)
    H = kf.z.j(kf.x.x)'
    ph = kf.x.p * H'
    s = H * kf.x.p * H' + kf.z.r
    (res,ph,s)
end

