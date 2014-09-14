function predict(x::BasicKalmanFilter)
    x1 = apply(x.f,x.x)
    # Needs to apply f to the covariance matrix
end

function Base.apply(f::LinearModel,x::State)
    f.a.*x + rand(gmvnormal(f.q))
end

function update(x::BasicKalmanFilterP)

end
