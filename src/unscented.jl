include("unscentedtypes.jl")

## Sigma points and weights


# Predict and update functions

function predict(kf::AdditiveUnscentedKalmanFilter)
    σp = map(kf.f.f,kf.σ) # Run sigmas through the func.
    xhat = dot(kf.wm,σp) # Weight and sum for the mean
    pu = map(x->(x-xhat)*(x-xhat)',σp) # Find unweigh. cov
    phat = dot(kf.wc,pu) + kf.f.q # Weight cov. and add process noise
    AdditiveUnscentedKalmanFilter(State(xhat,phat),kf.f,kf.z,σp,kf.α,kf.β,kf.κ,kf.wm,kf.wc)
end

function Kalman.predict!(kf::AdditiveUnscentedKalmanFilter)
    σp = map(kf.f.f,kf.σ) # Run them through the func.
    xhat = dot(kf.wm,σp) # Weight and sum for the mean
    pu = map(x->(x-xhat)*(x-xhat)',σp) # Find unweigh. cov
    phat = dot(kf.wc,pu) + kf.f.q # Weight cov. and add process noise
    kf.x = State(xhat,phat)
    kf.σ = σp
    kf
end

function Kalman.update(kf::AdditiveUnscentedKalmanFilter,y::Observation)
    yp = map(kf.z.h,kf.σ) # Run sigmas through h
    yhat = dot(kf.wm,yp) # Weight to find mean
    resx = map(x->x-kf.x.x,kf.σ) # I think that's right
    resy = map(y->y-yhat,yp)
    # Next line is confusing
    pyy = dot(kf.wc,map(y->y*y',resy)) + r
    pxy = dot(kf.wc,map((x,y)->x*y',resx,resy))
    k = pxy*inv(pyy)    # Kalman gain
    xk = xhat + k*(y[i]-yhat) # New state
    pk = phat - k*pyy*k' # New state covariance
    AdditiveUnscentedKalmanFilter(State(xk,pk),kf.f,kf.z,kf.α,kf.β,kf.κ)
end

function Kalman.update!(kf::AdditiveUnscentedKalmanFilter,y::Observation)
    yp = map(kf.z.h,kf.σ) # Run sigmas through h
    yhat = dot(kf.wm,yp) # Weight to find mean
    resx = map(x->x-kf.x.x,kf.σ) # I think that's right
    resy = map(y->y-yhat,yp)
    # Next line is confusing
    pyy = dot(kf.wc,map(y->y*y',resy)) + kf.z.r
    pxy = dot(kf.wc,map((x,y)->x*y',resx,resy))
    k = pxy*inv(pyy)    # Kalman gain
    xk = kf.x.x + k*(y.y-yhat) # New state
    pk = kf.x.p - k*pyy*k' # New state covariance
    kf.x = State(xk,pk)
    kf.σ = sigma(kf.x,kf.α,kf.κ)
    kf
end

function Kalman.predictupdate!(kf::AdditiveUnscentedKalmanFilter,y::Observation)
    update!(predict!(kf),y)
end


