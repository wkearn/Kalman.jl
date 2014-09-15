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

x0 = State([1.,0],[0.5 0; 0 1e-10])
a = [1 0.01; 0.01*(-1) 1]
g = [0 0; 0 1]
q = [1e-10 0; 0 1e-10]
f = LinearModel(a,g,q)
h = [1 0]
r = [0.5]'
z = LinearObservationModel(h,r)

dt = 0.01
t = [0:dt:10]
x = zeros(length(t))
m = cos(t)

kf0 = BasicKalmanFilter(x0,f,z)
kf = kf0
x[1] = kf.x.x[1]

y = map(t->Observation(m[t]+rand(Distributions.gmvnormal(kf.z.r))),1:length(t))

for i in 2:length(t)-1
    kf2 = predict(kf)
    kf = update(kf2,y[i])
    x[i+1] = kf.x.x[1]
end

@test predictupdate(kf0,y[1]).x == update(predict(kf0),y[1]).x
