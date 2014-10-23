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

update!(kf1,Nullable([0.98]))

@test kf1.x == kf.x

## Extended Kalman Filter with Nullables

k = 1.0
p0 = 0.1*k
r = 10.0
dt = 0.0005

logit(p0,k,x) = (k*p0*exp(x))/(k+p0*(exp(x)-1))

function an(x,k,r,deltaT)
    x1 = zeros(x)
    x1[1] = x[1]
    x1[2] = logit(x[2],k,r*deltaT)
    return x1
end

fekf(x) = an(x,k,x[1],dt)
gekf(x) = eye(2)
hekf(x) = x[2]

Q = [1e-10 0.0; 0.0 1e-10]
R = [1e-2]'

x0 = State([5.0,0.1],[1e2 0.0; 0.0 1e-10])

ekff = NonlinearModel(fekf,gekf,Q)
z = NonlinearObservationModel(hekf,R)

kf0 = BasicExtendedKalmanFilter(x0,ekff,z)
kf = kf0

kf1 = deepcopy(kf)

predict!(kf)
predict!(kf1)

@test kf1.x == kf.x

update!(kf,Observation([0.1]))
update!(kf1,Nullable([0.1]))

@test kf1.x == kf.x

predict!(kf)
predict!(kf1)

update!(kf,Observation([0.09]))
update!(kf1,Nullable{Vector{Float64}}())

@test kf1.x != kf.x

update!(kf1,Nullable([0.09]))

@test kf1.x == kf.x
