function predict(kf::BasicKalmanFilter)
    x1 = ap(kf.f,kf.x)
    BasicKalmanFilter(x1,kf.f,kf.z)
end

function predict!(kf::BasicKalmanFilter)
    kf.x = ap(kf.f,kf.x)
    kf
end

function ap(f::LinearModel,x::State)
    x1 = f.a*x.x
    p1 = f.a*x.p*f.a' + f.g*f.q*f.g'
    State(x1,p1)
end


# Unscented filters require that we explicitly
# calculate the innovation matrix and use it
# to update the covariance. Since we calculate
# s here, there is no reason not to use the same
# method

function covs(kf::BasicKalmanFilter,y::Observation)
    res = y.y - kf.z.h * kf.x.x
    ph = kf.x.p * kf.z.h'
    s = kf.z.h * kf.x.p * kf.z.h' + kf.z.r
    (res,ph,s)
end

function update(kf::BasicKalmanFilter,y::Observation)
    (res,ph,s) = covs(kf,y)
    k = ph * inv(s)
    xn = kf.x.x + k * res
    pn = kf.x.p - k * s * k'
    BasicKalmanFilter(State(xn,pn),kf.f,kf.z)
end

function update!(kf::BasicKalmanFilter,y::Observation)
    (res,ph,s) = covs(kf,y)
    k = ph * inv(s)
    xn = kf.x.x + k * res
    pn = kf.x.p - k * s * k'
    kf.x = State(xn,pn)
    kf
end


function predictupdate(kf::BasicKalmanFilter,y::Observation)
    update(predict(kf),y)
end

function predictupdate!(kf::BasicKalmanFilter,y::Observation)
    update!(predict!(kf),y)
end

