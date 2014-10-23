# Filters using Nullable types to represent missing measurements

function update{T}(kf::BasicKalmanFilter,y::Nullable{Vector{T}})
    if isnull(y.y)
        return kf
    else
        res = get(y) - kf.z.h * kf.x.x
        s = kf.z.h * kf.x.p*kf.z.h' + kf.z.r
        k = kf.x.p * kf.z.h' * inv(s)
        xn = kf.x.x + k*res
        pn = kf.x.p - k * kf.z.h * kf.x.p
        return BasicKalmanFilter(State(xn,pn),kf.f,kf.z)
    end
end

function update!{T}(kf::BasicKalmanFilter,y::Nullable{Vector{T}})
    if isnull(y)
        return kf
    else
        res = get(y) - kf.z.h * kf.x.x
        s = kf.z.h * kf.x.p * kf.z.h' + kf.z.r
        k = kf.x.p*kf.z.h' * inv(s)
        xn = kf.x.x + k*res
        pn = kf.x.p - k * kf.z.h * kf.x.p
        kf.x = State(xn,pn)
        return kf
    end
end

function predictupdate{T}(kf::BasicKalmanFilter,y::Nullable{Vector{T}})
    update(predict(kf),y)
end

function predictupdate!{T}(kf::BasicKalmanFilter,y::Nullable{Vector{T}})
    update!(predict!(kf),y)
end


