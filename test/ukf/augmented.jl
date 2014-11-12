include("bkfiltertest.jl")

x1,p1 = augment(s,1e-10*eye(2))
@test x1 == [1.0,0.0,0.0,0.0]
@test p1 == [4*eye(2,2) zeros(2,2);
             zeros(2,2) 1e-10*eye(2)]

function fa(x::Vector)
    x1 = zeros(2)
    x1[1] = x[1] + 0.01*x[2] + x[3]
    x1[2] = x[2] - 0.01*x[1] + x[4]
    x1
end

fam = AugmentedUnscentedModel(fa,1e-10*eye(2))

function ha(x)
    x1 = zeros(1)
    x1[1] = x[1] + x[3]
end

zam = AugmentedUnscentedObservationModel(ha,[0.1]')

kaf = AugmentedUnscentedKalmanFilter(s,fam,zam)

kaf1 = predict(kaf)
kaf2 = update(kaf1,y)

@test_approx_eq kaf2.x.x kf2.x.x
@test_approx_eq kaf2.x.p kf2.x.p
