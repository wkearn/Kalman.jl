using Kalman
using Base.Test

s = UnscentedState([1.0, 0.0],4*eye(2),0.1,2.0,0.0)

@test s.x == [1.0,0.0]
@test s.p == [4.0 0; 0 4.0]
@test_approx_eq s.σ[5] [1.0,-sqrt(0.08)]

@test_approx_eq dot(s.wm,s.σ) s.x
@test_approx_eq dot(s.wc,map(x->(x-s.x)*(x-s.x)',s.σ)) s.p

const dt = 0.01

function f(x)
    x1 = zeros(x)
    x1[1] = x[1] + dt*x[2]
    x1[2] = x[2] - dt*x[1]
    x1
end

fm = AdditiveUnscentedModel(f,1e-10*eye(2))

h(x) = x[1]

zm = AdditiveUnscentedObservationModel(h,[0.1]')

kf = AdditiveUnscentedKalmanFilter(s,fm,zm)
