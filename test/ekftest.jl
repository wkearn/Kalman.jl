using Kalman
using Kalman.EKF

k = 1.0
p0 = 0.1*k
r = 10.0
deltaT = 0.0005
logit(p0,k,x) = (k*p0*exp(x))/(k+p0*(exp(x)-1))

function an(x,k,r,deltaT)
    x1 = zeros(x)
    x1[1] = x[1]
    x1[2] = logit(x[2],k,r*deltaT)
    return x1
end

fekf(x) = an(x,k,x[1],deltaT)
gekf(x) = eye(2)
hekf(x) = x[2]

Q = [1e-10 0.0; 0.0 1e-10]
R = [1e-2]'

x0 = State([5.0,0.1],[1e2 0.0; 0.0 1e-10])

ekff = NonlinearModel(fekf,gekf,Q)
z = NonlinearObservationModel(hekf,R)

kf0 = BasicExtendedKalmanFilter(x0,ekff,z,true)
kf = kf0

t = [0:deltaT:deltaT*300]
z = map(t->logit(p0,k,r*t),t)
ys = z.+R[1]*randn(301)

ps = zeros(t)
rs = zeros(t)

for i in 1:length(t)
    kf = predictupdate(kf,Observation([ys[i]]))
    ps[i] = kf.x.x[2]
    rs[i] = kf.x.x[1]
end
