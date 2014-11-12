using Kalman
using Base.Test

s = UnscentedState([1.0, 0.0],4*eye(2),0.1,2.0,0.0)
su,wm,wc = sigma(s)

@test s.x == [1.0,0.0]
@test s.p == [4.0 0; 0 4.0]
@test_approx_eq su[:,5] [1.0,-sqrt(0.08)]

xt = zeros(s.x)
pt = zeros(s.p)

for i in 1:size(su,2)
    xt += wm[i] * su[:,i]
    pt += wc[i] * (su[:,i]-s.x)*(su[:,i]-s.x)'
end

@test_approx_eq xt s.x
@test_approx_eq pt s.p

dt = 0.01

function fu(x)
    x1 = zeros(x)
    x1[1] = x[1] + dt*x[2]
    x1[2] = x[2] - dt*x[1]
    x1
end

fm = AdditiveUnscentedModel(fu,1e-10*eye(2))

hu(x) = [x[1]]

zm = AdditiveUnscentedObservationModel(hu,[0.1]')

kf = AdditiveUnscentedKalmanFilter(s,fm,zm)
