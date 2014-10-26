include("typestest.jl")

kf0 = copy(kf)

kf1 = predict(kf)
predict!(kf)

@test kf1.x.x == kf.x.x 
@test_approx_eq kf.x.x [1.0,-0.01]
@test kf1.x.p == kf.x.p
@test kf1.x.σ == kf.x.σ

y = Observation([1.01])

kf2 = update(kf,y)
update!(kf,y)

@test kf2.x.x == kf.x.x
@test kf2.x.p == kf.x.p
@test kf2.x.σ == kf.x.σ
