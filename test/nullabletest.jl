using Kalman, Base.Test

x0 = State([1.,0],[0.5 0; 0 1e-10])
dt = 0.01
a = [1 dt; dt*(-1) 1]
g = [0 0; 0 1]
q = [1e-10 0; 0 1e-10]
f = LinearModel(a,g,q)
h = [1 0]
r = [0.5]'
z = LinearObservationModel(h,r)

t = [0:dt:10]
x = zeros(length(t))
m = cos(t)

kf0 = BasicKalmanFilter(x0,f,z)

kf = kf0

kf1 = deepcopy(kf)

predict!(kf)
predict!(kf1)

@test kf1.x == kf.x

update!(kf,Observation([0.95]))
update!(kf1,Nullable([0.95]))

@test kf1.x == kf.x

predict!(kf)
predict!(kf1)

update!(kf,Observation([0.98]))
update!(kf1,Nullable{Vector{Float64}}())

@test kf1.x != kf.x


