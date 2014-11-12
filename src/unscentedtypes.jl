typealias Sigmas{T} Array{Array{T,1},1}

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

type AdditiveUnscentedKalmanFilter <: UnscentedKalmanFilter
    x::UnscentedState
    f::AdditiveUnscentedModel
    z::AdditiveUnscentedObservationModel
end


function AdditiveUnscentedKalmanFilter(x::Vector,p::Matrix,f::AdditiveUnscentedModel,z::AdditiveUnscentedObservationModel,α::Real,β::Real,κ::Real)
    s = UnscentedState(x,p,α,β,κ)
    AdditiveUnscentedKalmanFilter(s,f,z)
end

function AdditiveUnscentedKalmanFilter(x::State,f::AdditiveUnscentedModel,z::AdditiveUnscentedObservationModel,α::Real,β::Real,κ::Real)
    s = UnscentedState(x,α,β,κ)
    AdditiveUnscentedKalmanFilter(s,f,z)
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

function sigma(x::Vector,p::Matrix,α,κ)
    n = length(x)
    σ = fill(zeros(x),2n+1)
    λ = sqrt(n+α^2*(n+κ)-n)
    σ[1] = x
    sp = λ*full(chol(p))
    for i = 2:n+1
        σ[i] = x + sp[:,i-1]
        σ[i+n] = x - sp[:,i-1]
    end
    σ
end

sigma(x::State,α,κ) = sigma(x.x,x.p,α,κ)

function sigma(kf::AdditiveUnscentedKalmanFilter; recalc=true)
    recalc ? sigma(kf.x,kf.α,kf.κ) : kf.σ
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
