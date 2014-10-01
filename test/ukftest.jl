@test super(AdditiveUnscentedKalmanFilter) == UnscentedKalmanFilter

x0 = State([0.,5],0.5*eye(2))
function fu(x::Vector,dt::Float64)
    x1 = zeros(x)
    x1[1] = x[1] + dt*-x[2]
    x1[2] = x[2] + dt*(-0.2*(1-x[1]^2)*x[2]+x[1])
    x1
end

xs = fill(zeros(x0.x),4000)
xs[1] = [1.4,0]
x = xs[1]

for i = 2:4000
    x = fu(x,0.01)
    xs[i] = x
end
y = xs .+ map(y->[0.5 0; 0 1e-3]*randn(2),1:4000)

hu(x::Vector) = x

fm = AdditiveUnscentedModel(x->fu(x,0.01),1e-3*eye(2))
zm = AdditiveUnscentedObservationModel(hu,[0.5 0; 0 1e-3])

kf = AdditiveUnscentedKalmanFilter(x0,fm,zm,0.1,2.0,0.0)

xs1 = 0.0*ones(4000)
xs2 = 5.0*ones(4000)
ps = 0.5*ones(4000)

for i = 2:4000
    predictupdate!(kf,Observation([y[i]]))
    xs1[i] = kf.x.x[1]
    xs2[i] = kf.x.x[2]
    ps[i] = kf.x.p[1]
end
