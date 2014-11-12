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
    yp = map(kf.z.h,kf.x.σ)
    yhat = dot(kf.x.wm,yp)

    resx = map(x->x-kf.x.x,kf.x.σ)
    resy = map(y->y-yhat,yp)

    res = y.y-yhat
    ph = dot(kf.x.wc,map((x,z)->x*z',resx,resy))
    s = dot(kf.x.wc,map(x->x*x',resy)) + kf.z.r

    return res,ph,s
end

