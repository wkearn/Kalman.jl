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

filt0 = BasicKalmanFilter(x0,f,z)
filt = filt0
x[1] = filt.x.x[1]

y = map(t->Observation(m[t]+rand(Distributions.gmvnormal(filt.z.r))),1:length(t))

for i in 2:length(t)-1
    filt2 = predict(filt)
    filt = update(filt2,y[i])
    x[i+1] = filt.x.x[1]
end


