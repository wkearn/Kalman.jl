#######
# Unscented Kalman filter types and filtering interface

abstract UnscentedKalmanFilter <: KalmanFilter
abstract AbstractUnscentedState <: AbstractState

type UnscentedState{T} <: AbstractUnscentedState
    x::Vector{T}
    p::Matrix
    α::Real
    β::Real
    κ::Real
end

type AdditiveUnscentedObservationModel <: ObservationModel
    h::Function
    r::Matrix
end

type AdditiveUnscentedModel <: Model
    f::Function
    q::Matrix
end

## ap function for AdditiveUnscentedModels and UnscentedStates

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

type AdditiveUnscentedKalmanFilter <: UnscentedKalmanFilter
    x::UnscentedState
    f::AdditiveUnscentedModel
    z::AdditiveUnscentedObservationModel
end


function AdditiveUnscentedKalmanFilter(x::Vector,p::Matrix,f::AdditiveUnscentedModel,z::AdditiveUnscentedObservationModel,α::Real,β::Real,κ::Real)
    s = UnscentedState(x,p,α,β,κ)
    AdditiveUnscentedKalmanFilter(s,f,z)
end

### covs function for additive unscented Kalman filters

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

    s += kf.z.r
    return res,ph,s
end
#######
# Sigma point generating functions

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

function sigma(s::UnscentedState)
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
