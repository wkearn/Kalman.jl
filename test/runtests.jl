using Kalman
using Base.Test

@test super(BasicKalmanFilter) == LinearKalmanFilter

s = State([1.,0],[0.5 0; 0 1e-10])

@test typeof(s) == State{Float64}
@test s.x == [1,0]
@test s.p == [0.5 0; 0 1e-10]

o = Observation([2.],[0.5]')

@test typeof(o) == Observation{Float64}
@test o.y == [2]
@test o.r == [0.5]'
