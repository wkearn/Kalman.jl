include("unscentedtypes.jl")

# Predict and update functions

# Now a target for abstraction
function ap(f::AdditiveUnscentedModel,s::UnscentedState)
    σs = map(f.f,s.σ)
    x = dot(s.wm,σs)
    p = dot(s.wc,map(y->(y-x)*(y-x)',s.σ)) + f.q
    UnscentedState(σs,s.α,s.β,s.κ,s.wm,s.wc)
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

