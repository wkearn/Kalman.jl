include("unscentedtypes.jl")

# Predict and update functions

# Now a target for abstraction
function ap(f::AdditiveUnscentedModel,s::UnscentedState)
    σn,wm,wc = sigma(s)
    σs = zeros(σn)
    xn = zeros(s.x)
    for i in 1:size(σn,2)
        σs[:,i] = f.f(σn[:,i])
        xn += wm[i] * σs[:,i]
    end
    pn = zeros(s.p)
    for i in 1:size(σn,2)
        pn += wc[i] * (σs[:,i]-xn)*(σs[:,i]-xn)'
    end
    pn += f.q
    UnscentedState(xn,pn,s.α,s.β,s.κ)
end

function covs(kf::AdditiveUnscentedKalmanFilter,y::Observation)
    n = size(kf.x.x,1)
    m = size(kf.z.r,1)
    σn,wm,wc = sigma(kf.x.x,kf.x.p,kf.x.α,kf.x.β,kf.x.κ)
    k = size(σn,2)
    yp = zeros(m,k)
    yhat = zeros(m)
    for i in 1:k
        yp[:,i] = kf.z.h(σn[:,i])
        yhat += wm[i] * yp[:,i]
    end

    resx = zeros(σn)
    resy = zeros(yp)

    for i in 1:k
        resx[:,i] = σn[:,i]-kf.x.x
        resy[:,i] = yp[:,i]-yhat
    end

    res = y.y-yhat
    
    ph = zeros(n,m)
    s = zeros(m,m)
    for i in 1:k
        ph += wc[i]*(resx[:,i]*resy[:,i]')
        s += wc[i]*(resy[:,i]*resy[:,i]')
    end

    return res,ph,s
end

