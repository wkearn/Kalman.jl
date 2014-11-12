typealias Sigmas{T} Array{Array{T,1},1}

abstract UnscentedKalmanFilter <: KalmanFilter
abstract AbstractUnscentedState <: AbstractState


type UnscentedState{T} <: AbstractUnscentedState
    x::Vector{T}
    p::Matrix
    σ::Sigmas{T}
    α::Real
    β::Real
    κ::Real
    wm::Vector
    wc::Vector
end

function UnscentedState{T}(x::Vector{T},p::Matrix,α::Real,β::Real,κ::Real)
    σ = sigma(x,p,α,κ)
    (wm,wc) = sigmaweights(length(x),α,β,κ)
    UnscentedState(x,p,σ,α,β,κ,wm,wc)
end

function UnscentedState{T}(x::State{T},α::Real,β::Real,κ::Real)
    σ = sigma(x,α,κ)
    (wm,wc) = sigmaweights(length(x.x),α,β,κ)
    UnscentedState(x.x,x.p,σ,α,β,κ,wm,wc)
end

function UnscentedState{T}(σ::Sigmas{T},α::Real,β::Real,κ::Real,wm::Vector,wc::Vector)
    x = dot(wm,σ)
    p = dot(wc,map(y->(y-x)*(y-x)',σ))
    UnscentedState(x,p,σ,α,β,κ,wm,wc)
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
