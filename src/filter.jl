function predict(filt::BasicKalmanFilter)
    x1 = ap(filt.f,filt.x)
    BasicKalmanFilterP(x1,filt.f,filt.z)
end

function ap(f::LinearModel,x::State)
    x1 = f.a*x.x
    p1 = f.a*x.p*f.a' + f.g*f.q*f.g'
    State(x1,p1)
end

function update(filt::BasicKalmanFilterP,y::Observation)
    res = y.y - filt.z.h * filt.x1.x
    s = filt.z.h * filt.x1.p * filt.z.h' + filt.z.r
    k = filt.x1.p * filt.z.h' * inv(s)
    xn = filt.x1.x + k * res
    pn = filt.x1.p - k * filt.z.h * filt.x1.p
    BasicKalmanFilter(State(xn,pn),filt.f,filt.z)
end


