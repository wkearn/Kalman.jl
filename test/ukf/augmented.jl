using Base.Test
include("../../src/augmented.jl")
using Augmented

x = [1.0,0.0]
p = 4.0*eye(2)
s = AugmentedUnscentedState(x,p,0.1,2.0,0.0)

(su,wm,wc) = sigma(s)

@test su[:,1] == x

x1,p1 = augment(s,1e-10*eye(2))
@test x1 == [1.0,0.0,0.0,0.0]
@test p1 == [p zeros(2,2);
             zeros(2,2) 1e-10*eye(2)]

function f(x::Vector)
    x1 = zeros(x)
    x1[1] = x[1] + 0.01*x[2] + x[3]
    x1[2] = x[2] - 0.01*x[1] + x[4]
    x1
end

fm = AugmentedUnscentedModel(f,1e-10*eye(2))

(xn,pn) = ap(fm,s)
