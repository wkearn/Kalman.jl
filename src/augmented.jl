#### 
# We only need different models from the UKF for dispatch

type AugmentedUnscentedModel <: Model
    f::Function
    q::Matrix
end

type AugmentedUnscentedObservationModel <: Model
    h::Function
    r::Matrix
end

type AugmentedUnscentedKalmanFilter <: KalmanFilter
    x::AugmentedUnscentedState
    f::AugmentedUnscentedModel
    z::AugmentedUnscentedObservationModel
end

function ap(f::AugmentedUnscentedModel,s::UnscentedState)
    n = length(s.x)
    (x,p) = augment(s,f.q)
    (σn,wm,wc) = sigma(x,p,s.α,s.β,s.κ)
    σs = zeros(σn)
    xn = zeros(x)
    for i in 1:size(σn,2)
        σs[:,i] = f.f(σn[:,i])
        xn += wm[i] * σs[:,i]
    end
    pn = zeros(p)
    for i in 1:size(σn,2)
        pn += wc[i] * (σs[:,i]-xn)*(σs[:,i]-xn)'
    end
    return AugmentedUnscentedState(xn[1:n],pn[1:n,1:n],s.α,s.β,s.κ)
end

function covs(kf::AugmentedUnscentedKalmanFilter,y::Observation)
    n = size(kf.x.x,1)
    m = size(kf.z.r,1)
    (x,p) = augment(kf.x,kf.z.r)
    (σn,wm,wc) = sigma(x,p,kf.x.α,kf.x.β,kf.x.κ)
    yp = zeros(m,size(σn,2))
    yhat = zeros(m)

    for i in 1:size(σn,2)
        yp[:,i] = kf.z.h(σn[:,i])
        yhat += wm[i] * yp[:,i]
    end

    resx = zeros(σn[1:n,:])
    resy = zeros(yp)
    
    for i in 1:size(σn,2)
        resx[:,i] = σn[1:n,i]-kf.x.x
        resy[:,i] = yp[:,i]-yhat
    end

    res = y.y-yhat
    ph = zeros(n,m)
    s = zeros(m,m)
    
    for i in 1:size(σn,2)
        ph += wc[i]*(resx[:,i]*resy[:,i]')
        s += wc[i]*(resy[:,i]*resy[:,i]')
    end

    return res,ph,s
end

function augment(s::UnscentedState,R::Matrix)
    n = length(s.x)
    m = size(R,1)
    x = [s.x,zeros(m)]
    p = [s.p zeros(n,m);
         zeros(m,n) R]
    return x,p
end
