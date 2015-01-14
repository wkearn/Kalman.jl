#######
# Universal Kalman filtering methods

function predict!(kf::KalmanFilter)
    kf.x = ap(kf.f,kf.x)
    kf
end

function predict(kf::KalmanFilter)
    predict!(copy(kf))
end

function update(kf::KalmanFilter,y::Observation)
    update!(copy(kf),y)
end

function update!(kf::KalmanFilter,y::Observation)
    (res,ph,s) = covs(kf,y)
    xn = kf.x.x + ph * (s\res)
    pn = kf.x.p - ph * (s'\ph')
    
    # This is an ugly hack which works for now
    if typeof(kf.x) <: AbstractUnscentedState
        kf.x = UnscentedState(xn,pn,kf.x.α,kf.x.β,kf.x.κ)
    else
        kf.x = State(xn,pn)
    end
    kf
end

function predictupdate(kf::KalmanFilter,y::Observation)
    update(predict(kf),y)
end

function predictupdate!(kf::KalmanFilter,y::Observation)
    update!(predict!(kf),y)
end
