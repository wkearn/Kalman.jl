# Filters using Nullable types to represent missing measurements

function update{T}(kf::KalmanFilter,y::Nullable{Vector{T}})
    if isnull(y.y)
        return kf
    else
        return update(kf,Observation(get(y)))
    end
end

function update!{T}(kf::KalmanFilter,y::Nullable{Vector{T}})
    if isnull(y)
        return kf
    else
        update!(kf,Observation(get(y)))
        return kf
    end
end

function predictupdate{T}(kf::BasicKalmanFilter,y::Nullable{Vector{T}})
    update(predict(kf),y)
end

function predictupdate!{T}(kf::BasicKalmanFilter,y::Nullable{Vector{T}})
    update!(predict!(kf),y)
end


