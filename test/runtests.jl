using Kalman
using Base.Test

include("bkftest.jl")
include("ekftest.jl")
include("ukf/augmented.jl")

if VERSION >= v"0.4.0-"
    include("nullabletest.jl")
end
