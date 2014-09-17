function predict(kf::BasicKalmanFilter)
    x1 = ap(kf.f,kf.x)
    BasicKalmanFilterP(x1,kf.f,kf.z)
end

function ap(f::LinearModel,x::State)
    x1 = f.a*x.x
    p1 = f.a*x.p*f.a' + f.g*f.q*f.g'
    State(x1,p1)
end

function update(kf::BasicKalmanFilterP,y::Observation)
    res = y.y - kf.z.h * kf.x1.x
    s = kf.z.h * kf.x1.p * kf.z.h' + kf.z.r
    k = kf.x1.p * kf.z.h' * inv(s)
    xn = kf.x1.x + k * res
    pn = kf.x1.p - k * kf.z.h * kf.x1.p
    BasicKalmanFilter(State(xn,pn),kf.f,kf.z)
end

function predictupdate(kf::BasicKalmanFilter,y::Observation)
    update(predict(kf),y)
end
