module Augmented

using Kalman

export AugmentedUnscentedState,
AugmentedUnscentedModel,
AugmentedUnscentedObservationModel,
Observation,
AugmentedUnscentedKalmanFilter,
covs,
ap,
sigma,
augment

type AugmentedUnscentedState{T} <: AbstractState
    x::Vector{T}
    p::Matrix
    α::Real
    β::Real
    κ::Real
end

#AugmentedUnscentedState(x::Vector,p::Matrix,α::Real,β::Real,κ::Real) = AugmentedUnscentedState(x,p,α,β,κ)

type AugmentedUnscentedModel <: Model
    f::Function
    q::Matrix
end

type AugmentedUnscentedObservationModel <: Model
    h::Function
    r::Matrix
end

type AugmentedUnscentedKalmanFilter
    x::AugmentedUnscentedState
    f::AugmentedUnscentedModel
    z::AugmentedUnscentedObservationModel
end

function ap(f::AugmentedUnscentedModel,s::AugmentedUnscentedState)
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
    (x,p) = augment(kf.x,kf.z.r)
    (σn,wm,wc) = sigma(x,p,kf.x.α,kf.x.β,kf.x.κ)
    yp = zeros(σn)
    yhat = zeros(x)

    for i in 1:size(σn,2)
        yp[:,i] = kf.z.h(σn[:,i])
        yhat += wm[i] * yp[:,i]
    end
    
    resx = map(x->x-kf.x.x,σn)
    resy = map(y->y-yhat,yp)

    res = y.y-yhat
    ph = dot(wc,map((x,z)->x*z',resx,resy))
    s = dot(wc,map(x->x*x',resy))

    return res,ph,s
end

function sigma(x::Vector,p::Matrix,α,β,κ)
    n = length(x)
    σ = zeros(n,2n+1)
    wm = zeros(2n+1)
    wc = zeros(2n+1)
    λ = α^2*(n+κ)-n
    σ[:,1] = x
    wm[1] = λ/(n+λ)
    wc[1] = wm[1] + (1-α^2+β)
    wm[2:end] = 1/(2*(n+λ))
    wc[2:end] = wm[2:end]
    sp = sqrt(n+λ)*full(chol(p))
    for i = 2:n+1
        σ[:,i] = x + sp[:,i-1]
        σ[:,i+n] = x - sp[:,i-1]
    end
    (σ,wm,wc)
end

function sigma(s::AugmentedUnscentedState)
    n = length(s.x)
    σ = zeros(n,2n+1)
    wm = zeros(2n+1)
    wc = zeros(2n+1)
    λ = s.α^2*(n+s.κ)-n
    σ[:,1] = s.x
    wm[1] = λ/(n+λ)
    wc[1] = wm[1] + (1-s.α^2+s.β)
    wm[2:end] = 1/(2*(n+λ))
    wc[2:end] = wm[2:end]
    sp = sqrt(n+λ)*full(chol(s.p))
    for i = 2:n+1
        σ[:,i] = s.x + sp[:,i-1]
        σ[:,i+n] = s.x - sp[:,i-1]
    end
    (σ,wm,wc)
end

function sigmaweights(n,α,β,κ)
    wm = zeros(2n+1)
    wc = zeros(2n+1)
    λ = α^2*(n+κ)-n
    wm[1] = λ/(n+λ)
    wc[1] = wm[1] + (1-α^2+β)
    for i = 2:2n+1
        wm[i] = 1/(2*(n+λ))
        wc[i] = wm[i]
    end
    (wm,wc)
end

function augment(s::AugmentedUnscentedState,R::Matrix)
    n = length(s.x)
    m = size(R,1)
    x = [s.x,zeros(m)]
    p = [s.p zeros(n,m);
         zeros(m,n) R]
    return x,p
end

    
end
