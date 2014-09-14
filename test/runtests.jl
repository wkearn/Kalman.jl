using Kalman
using Base.Test

@test super(BasicKalmanFilter) == LinearKalmanFilter

s = State([1.,0],[0.5 0; 0 1e-10])

@test typeof(s) == State{Float64}
@test s.x == [1,0]
@test s.p == [0.5 0; 0 1e-10]

o = Observation([2.])

@test typeof(o) == Observation{Float64}
@test o.y == [2]

x0 = State([0.,1],[0.5 0; 0 1e-10])
a = [1 0.01; 0 1]
g = [0 0; 0 1]
q = [1e-10 0; 0 1e-10]
f = LinearModel(a,g,q)
h = [1 0]
r = [0.5]'
z = LinearObservationModel(h,r)
filt = BasicKalmanFilter(x0,f,z)
predict(filt)

