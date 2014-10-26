include("typestest.jl")

kf0 = copy(kf)
kf1 = copy(kf)

predict!(kf)
kf2 = predict(kf1)

@test kf2.x.x == kf.x.x
@test kf2.x.p == kf.x.p
@test kf2.x.σ == kf.x.σ
