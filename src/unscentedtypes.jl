typealias Sigmas Array{Array{Float64,1},1}

abstract UnscentedKalmanFilter <: KalmanFilter

type AdditiveUnscentedObservationModel <: ObservationModel
    h::Function
    r::Matrix
end

type AdditiveUnscentedModel <: Model
    f::Function
    q::Matrix
end

type AdditiveUnscentedKalmanFilter <: UnscentedKalmanFilter
    x::State
    f::AdditiveUnscentedModel
    z::AdditiveUnscentedObservationModel
    σ::Sigmas
    α::Real
    β::Real
    κ::Real
    wm::Vector
    wc::Vector
end

function AdditiveUnscentedKalmanFilter(x::State,f::AdditiveUnscentedModel,z::AdditiveUnscentedObservationModel,α::Real,β::Real,κ::Real)
        σ = sigma(x,α,κ)
        (wm,wc) = sigmaweights(length(x.x),α,β,κ)
        AdditiveUnscentedKalmanFilter(x,f,z,σ,α,β,κ,wm,wc)
end

function AdditiveUnscentedKalmanFilter(x::State,f::AdditiveUnscentedModel,z::AdditiveUnscentedObservationModel,α::Real,β::Real,κ::Real,wm::Vector,wc::Vector)
    σ = sigma(x,α,κ)
    AdditiveUnscentedKalmanFilter(x,f,z,σ,α,β,κ,wm,wc)
end

function sigma(x::Vector,p::Matrix,α,κ)
    n = length(x)
    σ = fill(zeros(x),2n+1)
    γ = sqrt(n+α^2*(n+κ)-n)
    σ[1] = x
    sp = γ*chol(p)
    for i = 2:n+1
        σ[i] = x + sp[:,i-1]
        σ[i+n] = x - sp[:,i-1]
    end
    σ
end

sigma(x::State,α,κ) = sigma(x.x,x.p,α,κ)
sigma(kf::AdditiveUnscentedKalmanFilter) = sigma(kf.x,kf.α,kf.κ)

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
