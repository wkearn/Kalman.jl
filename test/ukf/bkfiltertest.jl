include("filtertest.jl")

a = [1.0 0.01;
     -0.01 1]
q = 1e-10*eye(2)
g = eye(2)

bfm = LinearModel(a,g,q)

hz = [1.0 0.0]
r = [0.1]'

bzm = LinearObservationModel(hz,r)

bkf = BasicKalmanFilter(State([1.0,0.0],4.0*eye(2)),bfm,bzm)

bkf1 = predict(bkf)
bkf2 = update(bkf1,y)

@test_approx_eq_eps bkf2.x.x kf.x.x 2e-10
@test_approx_eq_eps bkf2.x.p kf.x.p 2e-10

