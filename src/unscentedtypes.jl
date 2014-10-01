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
    function AdditiveUnscentedKalmanFilter(x::State,f::AdditiveUnscentedModel,z::AdditiveUnscentedObservationModel,α::Real,β::Real,κ::Real)
        σ = sigma(x,α,κ)
        (wm,wc) = sigmaweights(length(x.x),α,β,κ)
        new(x,f,z,σ,α,β,κ,wm,wc)
    end
end
