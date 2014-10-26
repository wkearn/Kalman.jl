using Kalman
using Base.Test

s = UnscentedState([1.0, 0.0],4*eye(2),0.1,2.0,0.0)

@test s.x == [1.0,0.0]
@test s.p == [4.0 0; 0 4.0]
@test_approx_eq s.σ[5] [1.0,-sqrt(0.08)]
